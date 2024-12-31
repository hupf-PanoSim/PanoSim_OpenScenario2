#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Welcome to CARLA scenario_runner

This is the main script to be executed when running a scenario.
It loads the scenario configuration, loads the scenario and manager,
and finally triggers the scenario execution.
"""

from __future__ import print_function

import glob
import traceback
import argparse
from argparse import RawTextHelpFormatter
from datetime import datetime
import importlib
import inspect
import os
import signal
import sys
import time
import json

from srunner.scenariomanager.data_provider import PanoSimDataProvider, PanoSimClient, PanoSimVector3D, PanoSimVehicleControl
from srunner.scenariomanager.scenario_manager import ScenarioManager
from srunner.tools.osc2_helper import OSC2Helper
from srunner.scenarios.osc2_scenario import OSC2Scenario
from srunner.scenarioconfigs.osc2_scenario_configuration import OSC2ScenarioConfiguration

from BusAccessor import *

# pip install antlr4-python3-runtime==4.10

# Version of scenario_runner
VERSION = '0.9.13'


class ScenarioRunner(object):

    """
    This is the core scenario runner module. It is responsible for
    running (and repeating) a single scenario or a list of scenarios.

    Usage:
    scenario_runner = ScenarioRunner(args)
    scenario_runner.run()
    del scenario_runner
    """

    ego_vehicles = []

    # Tunable parameters
    client_timeout = 10.0  # in seconds
    wait_for_world = 20.0  # in seconds
    frame_rate = 20.0      # in Hz

    # CARLA world and scenario handlers
    world = None
    manager = None

    finished = False

    additional_scenario_module = None

    agent_instance = None
    module_agent = None

    def __init__(self, args):
        """
        Setup CARLA client and world
        Setup ScenarioManager
        """
        self._args = args

        if args.timeout:
            self.client_timeout = float(args.timeout)

        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
        self.client = PanoSimClient(args.host, int(args.port))
        self.client.set_timeout(self.client_timeout)
        # dist = pkg_resources.get_distribution("carla")
        # if LooseVersion(dist.version) < LooseVersion('0.9.15'):
        #     raise ImportError("CARLA version 0.9.15 or newer required. CARLA version found: {}".format(dist))

        # Load agent if requested via command line args
        # If something goes wrong an exception will be thrown by importlib (ok here)
        if self._args.agent is not None:
            module_name = os.path.basename(args.agent).split('.')[0]
            sys.path.insert(0, os.path.dirname(args.agent))
            self.module_agent = importlib.import_module(module_name)

        # Create the ScenarioManager
        self.manager = ScenarioManager(self._args.debug, self._args.sync, self._args.timeout)

        # Create signal handler for SIGINT
        self._shutdown_requested = False
        if sys.platform != 'win32':
            signal.signal(signal.SIGHUP, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._start_wall_time = datetime.now()

    def destroy(self):
        """
        Cleanup and delete actors, ScenarioManager and CARLA world
        """

        self._cleanup()
        if self.manager is not None:
            del self.manager
        if self.world is not None:
            del self.world
        if self.client is not None:
            del self.client

    def _signal_handler(self, signum, frame):
        """
        Terminate scenario ticking when receiving a signal interrupt
        """
        self._shutdown_requested = True
        if self.manager:
            self.manager.stop_scenario()
            self._cleanup()
            if not self.manager.get_running_status():
                raise RuntimeError("Timeout occurred during scenario execution")

    def _get_scenario_class_or_fail(self, scenario):
        """
        Get scenario class by scenario name
        If scenario is not supported or not found, exit script
        """

        # Path of all scenario at "srunner/scenarios" folder + the path of the additional scenario argument
        scenarios_list = glob.glob("{}/srunner/scenarios/*.py".format(os.getenv('SCENARIO_RUNNER_ROOT', "./")))
        scenarios_list.append(self._args.additionalScenario)

        for scenario_file in scenarios_list:

            # Get their module
            module_name = os.path.basename(scenario_file).split('.')[0]
            sys.path.insert(0, os.path.dirname(scenario_file))
            scenario_module = importlib.import_module(module_name)

            # And their members of type class
            for member in inspect.getmembers(scenario_module, inspect.isclass):
                if scenario in member:
                    return member[1]

            # Remove unused Python paths
            sys.path.pop(0)

        print("Scenario '{}' not supported ... Exiting".format(scenario))
        sys.exit(-1)

    def _cleanup(self):
        """
        Remove and destroy all actors
        """
        if self.finished:
            return

        self.finished = True

        # Simulation still running and in synchronous mode?
        if self.world is not None and self._args.sync:
            try:
                # Reset to asynchronous mode
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                self.world.apply_settings(settings)
                self.client.get_trafficmanager(int(self._args.trafficManagerPort)).set_synchronous_mode(False)
            except RuntimeError:
                sys.exit(-1)

        self.manager.cleanup()

        PanoSimDataProvider.cleanup()

        for i, _ in enumerate(self.ego_vehicles):
            if self.ego_vehicles[i]:
                if not self._args.waitForEgo and self.ego_vehicles[i] is not None and self.ego_vehicles[i].is_alive:
                    print("Destroying ego vehicle {}".format(self.ego_vehicles[i].id))
                    self.ego_vehicles[i].destroy()
                self.ego_vehicles[i] = None
        self.ego_vehicles = []

        if self.agent_instance:
            self.agent_instance.destroy()
            self.agent_instance = None

    def _prepare_ego_vehicles(self, ego_vehicles):
        """
        Spawn or update the ego vehicles
        """

        if not self._args.waitForEgo:
            for vehicle in ego_vehicles:
                self.ego_vehicles.append(PanoSimDataProvider.request_new_actor(vehicle.model,
                                                                             vehicle.transform,
                                                                             vehicle.rolename,
                                                                             random_location=vehicle.random_location,
                                                                             color=vehicle.color,
                                                                             actor_category=vehicle.category))
        else:
            ego_vehicle_missing = True
            while ego_vehicle_missing:
                self.ego_vehicles = []
                ego_vehicle_missing = False
                for ego_vehicle in ego_vehicles:
                    ego_vehicle_found = False
                    carla_vehicles = PanoSimDataProvider.get_world().get_actors().filter('vehicle.*')
                    for carla_vehicle in carla_vehicles:
                        if carla_vehicle.attributes['role_name'] == ego_vehicle.rolename:
                            ego_vehicle_found = True
                            self.ego_vehicles.append(carla_vehicle)
                            break
                    if not ego_vehicle_found:
                        ego_vehicle_missing = True
                        break

            for i, _ in enumerate(self.ego_vehicles):
                self.ego_vehicles[i].set_transform(ego_vehicles[i].transform)
                self.ego_vehicles[i].set_target_velocity(PanoSimVector3D())
                self.ego_vehicles[i].set_target_angular_velocity(PanoSimVector3D())
                self.ego_vehicles[i].apply_control(PanoSimVehicleControl())
                PanoSimDataProvider.register_actor(self.ego_vehicles[i], ego_vehicles[i].transform)

        # sync state
        if PanoSimDataProvider.is_sync_mode():
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def _analyze_scenario(self, config):
        """
        Provide feedback about success/failure of a scenario
        """

        # Create the filename
        current_time = str(datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        junit_filename = None
        json_filename = None
        config_name = config.name
        if self._args.outputDir != '':
            config_name = os.path.join(self._args.outputDir, config_name)

        if self._args.junit:
            junit_filename = config_name + current_time + ".xml"
        if self._args.json:
            json_filename = config_name + current_time + ".json"
        filename = None
        if self._args.file:
            filename = config_name + current_time + ".txt"

        if not self.manager.analyze_scenario(self._args.output, filename, junit_filename, json_filename):
            print("All scenario tests were passed successfully!")
        else:
            print("Not all scenario tests were successful")
            if not (self._args.output or filename or junit_filename):
                print("Please run with --output for further information")

    def _record_criteria(self, criteria, name):
        """
        Filter the JSON serializable attributes of the criterias and
        dumps them into a file. This will be used by the metrics manager,
        in case the user wants specific information about the criterias.
        """
        file_name = name[:-4] + ".json"

        # Filter the attributes that aren't JSON serializable
        with open('temp.json', 'w', encoding='utf-8') as fp:

            criteria_dict = {}
            for criterion in criteria:

                criterion_dict = criterion.__dict__
                criteria_dict[criterion.name] = {}

                for key in criterion_dict:
                    if key != "name":
                        try:
                            key_dict = {key: criterion_dict[key]}
                            json.dump(key_dict, fp, sort_keys=False, indent=4)
                            criteria_dict[criterion.name].update(key_dict)
                        except TypeError:
                            pass

        os.remove('temp.json')

        # Save the criteria dictionary into a .json file
        with open(file_name, 'w', encoding='utf-8') as fp:
            json.dump(criteria_dict, fp, sort_keys=False, indent=4)

    def _load_and_wait_for_world(self, town, ego_vehicles=None):
        """
        Load a new CARLA world and provide data to PanoSimDataProvider
        """

        if self._args.reloadWorld:
            self.world = self.client.load_world(town)
        else:
            # if the world should not be reloaded, wait at least until all ego vehicles are ready
            ego_vehicle_found = False
            if self._args.waitForEgo:
                while not ego_vehicle_found and not self._shutdown_requested:
                    vehicles = self.client.get_world().get_actors().filter('vehicle.*')
                    for ego_vehicle in ego_vehicles:
                        ego_vehicle_found = False
                        for vehicle in vehicles:
                            if vehicle.attributes['role_name'] == ego_vehicle.rolename:
                                ego_vehicle_found = True
                                break
                        if not ego_vehicle_found:
                            print("Not all ego vehicles ready. Waiting ... ")
                            time.sleep(1)
                            break

        self.world = self.client.get_world()

        if self._args.sync:
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 1.0 / self.frame_rate
            self.world.apply_settings(settings)

        PanoSimDataProvider.set_client(self.client)
        PanoSimDataProvider.set_world(self.world)

        # Wait for the world to be ready
        if PanoSimDataProvider.is_sync_mode():
            self.world.tick()
        else:
            self.world.wait_for_tick()

        # map_name = PanoSimDataProvider.get_map().name.split('/')[-1]
        # if map_name not in (town, "OpenDriveMap"):
        #     print("The CARLA server uses the wrong map: {}".format(map_name))
        #     print("This scenario requires to use map: {}".format(town))
        #     return False

        return True

    def _load_and_run_scenario(self, userData):
        result = False
        if not self._load_and_wait_for_world(userData['config'].town, userData['config'].ego_vehicles):
            self._cleanup()
            return False

        if self._args.agent:
            agent_class_name = self.module_agent.__name__.title().replace('_', '')
            try:
                self.agent_instance = getattr(self.module_agent, agent_class_name)(self._args.agentConfig)
                userData['config'].agent = self.agent_instance
            except Exception as e:
                traceback.print_exc()
                print("Could not setup required agent due to {}".format(e))
                self._cleanup()
                return False

        PanoSimDataProvider.set_traffic_manager_port(int(self._args.trafficManagerPort))

        print("\n\nPreparing scenario: " + userData['config'].name)
        try:
            self._prepare_ego_vehicles(userData['config'].ego_vehicles)
            userData['scenario'] = OSC2Scenario(world=self.world, ego_vehicles=self.ego_vehicles, config=userData['config'], osc2_file=self._args.openscenario2, timeout=100000)
        except Exception as exception:                  # pylint: disable=broad-except
            print("The scenario cannot be loaded")
            traceback.print_exc()
            print(exception)
            self._cleanup()
            return False

        try:
            self.manager.load_scenario(userData['scenario'], self.agent_instance)
            result = True
        except Exception as e:              # pylint: disable=broad-except
            traceback.print_exc()
            print(e)
            result = False

        return result


    def _run_osc2(self, userData):
        """
        Run a scenario based on ASAM OpenSCENARIO 2.0.
        https://www.asam.net/static_downloads/public/asam-openscenario/2.0.0/welcome.html
        """
        # Load the scenario configurations provided in the config file
        if os.path.isfile(self._args.openscenario2):
            userData['config'] = OSC2ScenarioConfiguration(self._args.openscenario2, self.client)
            self._load_and_run_scenario(userData)
        else:
            print("File does not exist")
            self._cleanup()
            return False



def ModelStart(userData):
    description = ("CARLA Scenario Runner: Setup, Run and Evaluate scenarios using CARLA\nCurrent version: " + VERSION)

    # pylint: disable=line-too-long
    parser = argparse.ArgumentParser(description=description, formatter_class=RawTextHelpFormatter)
    parser.add_argument('-v', '--version', action='version', version='%(prog)s ' + VERSION)
    parser.add_argument('--host', default='127.0.0.1', help='IP of the host server (default: localhost)')
    parser.add_argument('--port', default='2000', help='TCP port to listen to (default: 2000)')
    parser.add_argument('--timeout', default="10.0", help='Set the CARLA client timeout value in seconds')
    parser.add_argument('--trafficManagerPort', default='8000', help='Port to use for the TrafficManager (default: 8000)')
    parser.add_argument('--trafficManagerSeed', default='0', help='Seed used by the TrafficManager (default: 0)')
    parser.add_argument('--sync', action='store_true', help='Forces the simulation to run synchronously')
    parser.add_argument('--list', action="store_true", help='List all supported scenarios and exit')

    parser.add_argument('--scenario', help='Name of the scenario to be executed. Use the preposition \'group:\' to run all scenarios of one class, e.g. ControlLoss or FollowLeadingVehicle')
    parser.add_argument('--openscenario', help='Provide an OpenSCENARIO definition')
    parser.add_argument('--openscenarioparams', help='Overwrited for OpenSCENARIO ParameterDeclaration')
    parser.add_argument('--openscenario2', help='Provide an openscenario2 definition')
    parser.add_argument('--route', help='Run a route as a scenario', type=str)
    parser.add_argument('--route-id', help='Run a specific route inside that \'route\' file', default='', type=str)
    parser.add_argument('--agent', help="Agent used to execute the route. Not compatible with non-route-based scenarios.")
    parser.add_argument('--agentConfig', type=str, help="Path to Agent's configuration file", default="")

    parser.add_argument('--output', action="store_true", help='Provide results on stdout')
    parser.add_argument('--file', action="store_true", help='Write results into a txt file')
    parser.add_argument('--junit', action="store_true", help='Write results into a junit file')
    parser.add_argument('--json', action="store_true", help='Write results into a JSON file')
    parser.add_argument('--outputDir', default='', help='Directory for output files (default: this directory)')

    parser.add_argument('--configFile', default='', help='Provide an additional scenario configuration file (*.xml)')
    parser.add_argument('--additionalScenario', default='', help='Provide additional scenario implementations (*.py)')

    parser.add_argument('--debug', action="store_true", help='Run with debug output')
    parser.add_argument('--reloadWorld', action="store_true", help='Reload the CARLA world before starting a scenario (default=True)')
    parser.add_argument('--record', type=str, default='', help='Path were the files will be saved, relative to SCENARIO_RUNNER_ROOT.\nActivates the CARLA recording feature and saves to file all the criteria information.')
    parser.add_argument('--randomize', action="store_true", help='Scenario parameters are randomized')
    parser.add_argument('--repetitions', default=1, type=int, help='Number of scenario executions')
    parser.add_argument('--waitForEgo', action="store_true", help='Connect the scenario to an existing ego vehicle')

    arguments = parser.parse_args()

    arguments.additionalScenario = ''
    arguments.agent = None
    arguments.agentConfig = ''
    arguments.configFile = ''
    arguments.debug = False
    arguments.file = False
    arguments.host = '127.0.0.1'
    arguments.json = False
    arguments.junit = False
    arguments.list = False
    scenario = os.environ['PanoSimDatabaseHome']
    scenario += '/Plugin/Disturbance/Library/srunner/examples/'
    scenario += userData['parameters']['scenario']
    scenario += '.osc'
    # arguments.openscenario2 = 'D:\\WorkFiles\\github\\OpenScenario2Player\\srunner\\examples/cut_in_and_slow_right.osc'
    arguments.openscenario2 = scenario
    arguments.openscenarioparams = None
    arguments.output = True
    arguments.outputDir = ''
    arguments.port = '2000'
    arguments.randomize = False
    arguments.record = ''
    arguments.reloadWorld = False
    arguments.repetitions = 1
    arguments.route = None
    arguments.route_id = ''
    arguments.scenario = None
    arguments.sync = False
    arguments.timeout = '10.0'
    arguments.trafficManagerPort = '8000'
    arguments.trafficManagerSeed = '0'
    arguments.waitForEgo = False

    OSC2Helper.wait_for_ego = arguments.waitForEgo

    if arguments.openscenarioparams and not arguments.openscenario:
        print("WARN: Ignoring --openscenarioparams when --openscenario is not specified")

    if arguments.route:
        arguments.reloadWorld = True

    if arguments.agent:
        arguments.sync = True

    scenario_runner = None

    userData["bus_traffic"] = DoubleBusReader(userData["busId"], "traffic", "time@i,100@[,id@i,type@b,shape@i,x@f,y@f,z@f,yaw@f,pitch@f,roll@f,speed@f")

    Format = 'time@i,temperature@f,pressure@f,humidity@f,precipitation_type@b,particle_size@f,particle_capacity@f,\
        falling_alpha@f,falling_beta@f,falling_speed@f,fog_visibility@f,lighting_alpha@f,lighting_beta@f,\
        lighting_intensity@f,lighting_ambient@f,street_light@b,vehicle_light@b,skybox@b,friction@f,wetness@f,snow@f'
    userData['weather'] = BusAccessor(userData['busId'], 'weather', Format)
    PanoSimDataProvider._bus_weather = userData['weather']

    try:
        userData['scenario_runner'] = ScenarioRunner(arguments)
        userData['scenario_runner']._run_osc2(userData)
    except Exception:
        traceback.print_exc()

    finally:
        if scenario_runner is not None:
            scenario_runner.destroy()
            del scenario_runner

def ModelOutput(userData):
    PanoSimDataProvider._timestamp = userData['time']
    try:
        if userData['time'] == 10:
            userData['scenario_runner'].manager.scenario_start()
        else:
            userData['scenario_runner'].manager.ModelOutput(userData)
    except Exception as e:
        traceback.print_exc()
        print(e)

def ModelTerminate(userData):
    userData['scenario_runner'].manager.ModelTerminate()
    userData['scenario_runner']._analyze_scenario(userData['config'])
    userData['scenario'].remove_all_actors()
    userData['scenario_runner']._cleanup()
