# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import os

from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.objects.cuboid import FixedCuboid
from omni.isaac.core.objects import VisualCylinder
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats

from omni.isaac.motion_generation import RmpFlow, ArticulationMotionPolicy
from omni.isaac.motion_generation.interface_config_loader import (
    get_supported_robot_policy_pairs,
    load_supported_motion_policy_config,
)

class FrankaRmpFlowExample():
    def __init__(self):
        self._rmpflow = None
        self._articulation_rmpflow = None

        self._articulation = None
        self._target = None

        self._dbg_mode = False

    def load_example_assets(self):
        # Add the Franka and target to the stage
        # The position in which things are loaded is also the position in which they 

        robot_prim_path = "/ur5e"
        path_to_robot_usd = "/home/aist/Documents/TeleSign/Extension_TeleSign/TeleSign Extension/RmpFlow_Example_python/UR5e_TeleSign.usd"

        add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        self._articulation = Articulation(robot_prim_path)
        
        #add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", "/World/target")
        #self._target = XFormPrim("/World/target", scale=[.04,.04,.04],)
        self._target = VisualCylinder("/World/target",scale=[0.04, 0.04, 0.06],color=np.array([0.1,0.0,0.0]))
        
        #self._parent = FixedCuboid("/World/obstacleFolder",scale=[0.0, 0.0, 0.0], position=np.array([0.0, 0.55, -0.1]))

        self._obstacle = FixedCuboid("/World/obstacleFolder/obstacle",name="Obstacle",scale=[1.0, 1.36, 0.2],position=np.array([0.0, 0.55, -0.1]),color=np.array([0.,0.,0.1]))
        self._obstacleTwo = FixedCuboid("/World/obstacleFolder/obstacleTwo",name="ObstacleTwo",scale=[0.23, 0.31, 0.22],position=np.array([-0.114, 0.5285, 0.1158]),color=np.array([0.,0.,0.1]))
        self._obstacleThree = FixedCuboid("/World/obstacleFolder/BoxStacle",name="BoxStacle",scale=[0.33, 0.46, 0.1],position=np.array([0.2, 0.55302, 0.05358]),color=np.array([0.2,0.05,0.025]))
        self._obstacleFour = FixedCuboid("/World/obstacleFolder/obstacleFour",name="ObstacleFour",scale=[1.2, 0.05, 1.85],position=np.array([0.0, -0.2565, 0.17187]),color=np.array([0.1,0.1,0.1]))
        self._obstacleFive = FixedCuboid("/World/obstacleFolder/obstacleFive",name="ObstacleFive",scale=[0.05, 0.9, 1.85],position=np.array([-0.57, 0.27527, 0.172]),color=np.array([0.1, 0.1 ,0.1]))

        # Return assets that were added to the stage so that they can be registered with the core.World
        return self._articulation, self._target, self._obstacle, self._obstacleTwo, self._obstacleThree, self._obstacleFour, self._obstacleFive

    def setup(self):
        # Loading RMPflow can be done quickly for supported robots
        print("Supported Robots with a Provided RMPflow Config:", list(get_supported_robot_policy_pairs().keys()))
        rmp_config = load_supported_motion_policy_config("UR5e","RMPflow")

        #Initialize an RmpFlow object
        self._rmpflow = RmpFlow(**rmp_config)
        self._rmpflow.add_obstacle(self._obstacle)
        self._rmpflow.add_obstacle(self._obstacleTwo)
        self._rmpflow.add_obstacle(self._obstacleThree)
        self._rmpflow.add_obstacle(self._obstacleFour)
        self._rmpflow.add_obstacle(self._obstacleFive)

        if self._dbg_mode:
            self._rmpflow.set_ignore_state_updates(True)
            self._rmpflow.visualize_collision_spheres()

            # Set the robot gains to be deliberately poor
            bad_proportional_gains = self._articulation.get_articulation_controller().get_gains()[0]/50
            self._articulation.get_articulation_controller().set_gains(kps = bad_proportional_gains)

        #Use the ArticulationMotionPolicy wrapper object to connect rmpflow to the Franka robot articulation.
        self._articulation_rmpflow = ArticulationMotionPolicy(self._articulation,self._rmpflow)

        self._target.set_world_pose(np.array([0.394, 0.26, 0.487]),euler_angles_to_quats([-1.57,np.pi,0]))

    def update(self, step: float):
        # Step is the time elapsed on this frame
        target_position, target_orientation = self._target.get_world_pose()

        self._rmpflow.set_end_effector_target(
            target_position, target_orientation
        )

        # Track any movements of the cube obstacle
        self._rmpflow.update_world()

        #Track any movements of the robot base
        robot_base_translation,robot_base_orientation = self._articulation.get_world_pose()
        self._rmpflow.set_robot_base_pose(robot_base_translation,robot_base_orientation)

        action = self._articulation_rmpflow.get_next_articulation_action(step)
        self._articulation.apply_action(action)

    def reset(self):
        # Rmpflow is stateless unless it is explicitly told not to be
        if self._dbg_mode:
            # RMPflow was set to roll out robot state internally, assuming that all returned
            # joint targets were hit exactly.
            self._rmpflow.reset()
            self._rmpflow.visualize_collision_spheres()

        self._target.set_world_pose(np.array([0.394, 0.26, 0.487]),euler_angles_to_quats([-1.57,np.pi,0]))

