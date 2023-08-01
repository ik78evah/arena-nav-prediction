import traceback
import rospy
import rospkg
import yaml
import os
import sys
import inspect


currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

from constants import Constants

from environments.environment_factory import EnvironmentFactory
from environments.gazebo_environment import GazeboEnvironment
from environments.flatland_navpred_environment import FlatlandNavPredEnvironment
from environments.flatland_environment import FlatlandRandomModel
from manager.map_manager import MapManager
from manager.obstacle_manager import ObstacleManager
from manager.obstacle_navpred_manager import ObstacleNavPredManager
from manager.robot_manager import RobotManager
from tasks.task_factory import TaskFactory
from tasks.random import RandomTask
from tasks.scenario import ScenarioTask
from tasks.staged import StagedRandomTask
from tasks.navpred import NavPredTask
from utils import Utils

from map_distance_server.srv import GetDistanceMap


def get_predefined_task(namespace, mode, environment=None, **kwargs):
    """
    Gets the task based on the passed mode
    """
    if environment == None:
        environment = EnvironmentFactory.instantiate(Utils.get_environment())(namespace)

    rospy.wait_for_service("/distance_map")

    service_client_get_map = rospy.ServiceProxy("/distance_map", GetDistanceMap)

    map_response = service_client_get_map()

    map_manager = MapManager(map_response)

    environment.map_manager = map_manager
    
    obstacle_manager = None
    if mode == "navpred":
        obstacle_manager = ObstacleNavPredManager(namespace, map_manager, environment)
    else:
        obstacle_manager = ObstacleManager(namespace, map_manager, environment)

    robot_managers = create_robot_managers(namespace, map_manager, environment)

    # For every robot
    # - Create a unique namespace name
    # - Create a robot manager
    # - Launch the robot.launch file

    task = TaskFactory.instantiate(
        mode,
        obstacle_manager,
        robot_managers,
        map_manager,
        namespace=namespace,
        **kwargs
    )

    return task

def create_robot_managers(namespace, map_manager, environment):
    # Read robot setup file
    robot_setup_file = rospy.get_param('/robot_setup_file', "")

    if robot_setup_file == "":
        robots = create_default_robot_list(
            rospy.get_param("/model"),
            rospy.get_param("/local_planner", ""),
            rospy.get_param("/agent_name", "")
        )
    else:
        robots = read_robot_setup_file(robot_setup_file)

    if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
        return [RobotManager(namespace, map_manager, environment, robots[0])]

    robot_managers = []

    for robot in robots:
        amount = robot["amount"]

        for r in range(0, amount):
            name = f"{robot['model']}_{r}_{len(robot_managers)}"  

            robot_managers.append(
                RobotManager(namespace + "/" + name, map_manager, environment, robot)
            )

    return robot_managers


def read_robot_setup_file(setup_file):
    try:
        with open(
            os.path.join(rospkg.RosPack().get_path("task-generator"), "robot_setup", setup_file),
            "r"
        ) as file:
            return yaml.safe_load(file)["robots"]
    except:
        traceback.print_exc()
        rospy.signal_shutdown()


def create_default_robot_list(robot_model, planner, agent):
    return [{
        "model": robot_model,
        "planner": planner,
        "agent": agent,
        "amount": 1
    }]