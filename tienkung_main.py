

CONFIG = {
    "width": 1280,
    "height": 720,
    "sync_loads": True,
    "headless": False,
    "renderer": "RayTracedLighting",
    # "display_options": 3286,
}
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(CONFIG)  # we can also run as headless.

from omni.isaac.dynamic_control import _dynamic_control

import copy
import math
import numpy as np
from omni.isaac.core.utils.stage import is_stage_loading
from omni.isaac.core import World
from omni.isaac.core.prims import XFormPrim
import omni
from motion.p2p_trajectory import P2P_Trajectory
from omni.isaac.core.utils.viewports import set_camera_view
np.set_printoptions(precision=4)

import numpy as np
from scipy.spatial.transform import Rotation as R

class My_Simulation():

    def __init__(
            self,
            environment_path: str = "./usd_model/kienkung_with_hand_rendered.usd",
            debug=False
    ) -> None:
        self.debug = debug


        omni.usd.get_context().open_stage(environment_path)
        omni.usd.get_context().disable_save_to_recent_files()
        self.physics_dt = 1. / 100.
        self.my_world = World(physics_dt=self.physics_dt,
                              rendering_dt=1.0 / 60.,
                              stage_units_in_meters=1.0)

        while is_stage_loading():
            simulation_app.update()

        simulation_app.update()
        simulation_app.update()

        set_camera_view(eye=[2.5, 1.3, 2.4], 
                        target=[0., 0, 0.82], 
                        camera_prim_path="/OmniverseKit_Persp")
        
        self.my_robot = XFormPrim(
            prim_path="/World/humanoid/waist_link"
        )

        robot_pos, _robot_quat = self.my_robot.get_world_pose()
        robot_quat = np.zeros(4)
        robot_quat[-1], robot_quat[0:3] = _robot_quat[0], _robot_quat[1:]

        _transform_w_r = np.identity(4)
        _transform_w_r

        _transform_w_r[:3,:3] = R.from_quat(robot_quat).as_matrix()
        _transform_w_r[:3,-1] = robot_pos

        self.robot_position = copy.deepcopy(_transform_w_r)

        self.stp_cnt = 0
   
        self.tcp_pose = np.zeros(6)
        self.gripper_mag = 1.
        self.initialized = False
        self.num_joints = 0

     
        self.init_left_hand_mag = np.array([0., 0., 0., 0., 0., 0.])
        self.init_right_hand_mag = np.array([0., 0., 0., 0., 0., 0.])
        self.left_hand_position_cmd = []
        self.left_hand_position_cmd.append(self.init_left_hand_mag)
        self.right_hand_position_cmd = []
        self.right_hand_position_cmd.append(self.init_right_hand_mag)
        self.gripper_enable = True
        self.left_hand_joint_positions = np.zeros(12)
        self.right_hand_joint_positions = np.zeros(12)
        self.head_position_cmd = np.zeros(3)

        self.return_home_pose()
        
        self.action_counter = 1
        self.init_counter = 0

    def return_home_pose(self) -> bool:
        self.init_time = 5.
        self.init_steps = int(self.init_time/self.physics_dt)
        self.left_arm_joint_positions = np.zeros(7)
        self.right_arm_joint_positions = np.zeros(7)
        self.left_leg_joint_positions = np.zeros(6)
        self.right_leg_joint_positions = np.zeros(6)
        self.left_hand_mag_cmd = np.zeros(6)
        self.right_hand_mag_cmd = np.zeros(6)
        
        self.init_left_arm_joint_angles = np.radians(
            [-0, 0, -0, -20, -0, 0, 0 ])
        self.init_right_arm_joint_angles = np.radians(
            [0, -0, 0, 20, -0, 0, 0 ])
        self.init_left_leg_joint_angles = np.array([0., 0., -0.5, 1.0, -0.5, 0.])
        self.init_right_leg_joint_angles = np.array([0., 0., -0.5, 1.0, -0.5, 0.])
        self.init_head_start = np.radians([0., 0., 0.])
        self.init_head_end = np.radians([0., -10., 0.])

        self.left_arm_init_pose = []
        self.right_arm_init_pose = []
        self.left_hand_init_pose = []
        self.right_hand_init_pose = []
        self.head_init_pose = []
        self.left_leg_init_pose = []
        self.right_leg_init_pose = []
        
        
        
        
        for idx in range(6):
            self.left_leg_init_pose.append(P2P_Trajectory(self.left_leg_joint_positions[idx], 
                                                          self.init_left_leg_joint_angles[idx], 
                                                            0., 0., 0., 0., 1., self.init_time))
            self.right_leg_init_pose.append(P2P_Trajectory(self.right_leg_joint_positions[idx], 
                                                          self.init_right_leg_joint_angles[idx], 
                                                            0., 0., 0., 0., 1., self.init_time))
            
        for idx in range(7):
            self.left_arm_init_pose.append(P2P_Trajectory(self.left_arm_joint_positions[idx], 
                                                          self.init_left_arm_joint_angles[idx], 
                                                            0., 0., 0., 0., 1., self.init_time))
            self.right_arm_init_pose.append(P2P_Trajectory(self.right_arm_joint_positions[idx], 
                                                          self.init_right_arm_joint_angles[idx], 
                                                            0., 0., 0., 0., 1., self.init_time))
            
        for idx in range(3):
            self.head_init_pose.append(P2P_Trajectory(0., 
                                                          self.init_head_end[idx], 
                                                            0., 0., 0., 0., 1., self.init_time))
        
        self.head_position_cmd_list = []
        self.left_arm_position_cmd_list = []
        self.right_arm_position_cmd_list = []
        self.left_leg_position_cmd_list = []
        self.right_leg_position_cmd_list = []

        for i in range(self.init_steps):
            _cmd_joint_positions = np.zeros(7)
            _cmd_head_positions = np.zeros(3)
            for idx in range(7):
                _cmd_joint_positions[idx] = self.left_arm_init_pose[
                    idx].get_point(i*self.physics_dt)[0]
            self.left_arm_position_cmd_list.append(copy.deepcopy(_cmd_joint_positions))
            for idx in range(3):
                _cmd_head_positions[idx] = self.head_init_pose[
                    idx].get_point(i*self.physics_dt)[0]
            self.head_position_cmd_list.append(copy.deepcopy(_cmd_head_positions))
                 
        for i in range(self.init_steps):
            _cmd_joint_positions = np.zeros(7)
            for idx in range(7):
                _cmd_joint_positions[idx] = self.right_arm_init_pose[
                    idx].get_point(i*self.physics_dt)[0]
            self.right_arm_position_cmd_list.append(copy.deepcopy(_cmd_joint_positions))

        for i in range(self.init_steps):
            _cmd_joint_positions = np.zeros(6)
            for idx in range(6):
                _cmd_joint_positions[idx] = self.right_leg_init_pose[
                    idx].get_point(i*self.physics_dt)[0]
            self.right_leg_position_cmd_list.append(copy.deepcopy(_cmd_joint_positions))
        
        for i in range(self.init_steps):
            _cmd_joint_positions = np.zeros(6)
            for idx in range(6):
                _cmd_joint_positions[idx] = self.left_leg_init_pose[
                    idx].get_point(i*self.physics_dt)[0]
            self.left_leg_position_cmd_list.append(copy.deepcopy(_cmd_joint_positions))
        
        self.enable_robot_move = True
        return True

    def play(self, num_steps: int = 6000) -> None:
        omni.timeline.get_timeline_interface().stop()
        omni.timeline.get_timeline_interface().play()

        for _ in range(num_steps):
            self.my_world.step()
            if self.num_joints == 0:
                self.dc = _dynamic_control.acquire_dynamic_control_interface()
                self.art = self.dc.get_articulation("/World/humanoid/pelvis")
                self.dc.wake_up_articulation(self.art)
                self.num_joints = self.dc.get_articulation_dof_count(self.art)

                self.initialized = False
            else:
                if not self.initialized:
                    set_camera_view(eye=[2.5, 1.3, 2.4], 
                        target=[0., 0, 0.82], 
                        camera_prim_path="/OmniverseKit_Persp")
                    self.left_arm_joint_handles = []
                    self.right_arm_joint_handles = []
                    self.left_hand_handles = []
                    self.right_hand_handles = []
                    self.left_leg_joint_handles = []
                    self.right_leg_joint_handles = []
                    self.head_handles = []
                
                    for i in range(7):
                        self.left_arm_joint_handles.append(self.dc.find_articulation_dof(self.art, f"L_Joint_{i+1}"))

                    for i in range(7):
                        self.right_arm_joint_handles.append(self.dc.find_articulation_dof(self.art, f"R_Joint_{i+1}"))

                    self.left_leg_joint_handles.append(self.dc.find_articulation_dof(self.art, "hip_roll_l_joint"))
                    self.left_leg_joint_handles.append(self.dc.find_articulation_dof(self.art, "hip_yaw_l_joint"))
                    self.left_leg_joint_handles.append(self.dc.find_articulation_dof(self.art, "hip_pitch_l_joint"))
                    self.left_leg_joint_handles.append(self.dc.find_articulation_dof(self.art, "knee_pitch_l_joint"))
                    self.left_leg_joint_handles.append(self.dc.find_articulation_dof(self.art, "ankle_pitch_l_joint"))
                    self.left_leg_joint_handles.append(self.dc.find_articulation_dof(self.art, "ankle_roll_l_joint"))

                    self.right_leg_joint_handles.append(self.dc.find_articulation_dof(self.art, "hip_roll_r_joint"))
                    self.right_leg_joint_handles.append(self.dc.find_articulation_dof(self.art, "hip_yaw_r_joint"))
                    self.right_leg_joint_handles.append(self.dc.find_articulation_dof(self.art, "hip_pitch_r_joint"))
                    self.right_leg_joint_handles.append(self.dc.find_articulation_dof(self.art, "knee_pitch_r_joint"))
                    self.right_leg_joint_handles.append(self.dc.find_articulation_dof(self.art, "ankle_pitch_r_joint"))
                    self.right_leg_joint_handles.append(self.dc.find_articulation_dof(self.art, "ankle_roll_r_joint"))

                    # Thumb
                    self.left_hand_handles.append(self.dc.find_articulation_dof(self.art, "L_Joint_40"))
                    self.left_hand_handles.append(self.dc.find_articulation_dof(self.art, "L_Joint_41"))
                    self.left_hand_handles.append(self.dc.find_articulation_dof(self.art, "L_Joint_42"))
                    self.left_hand_handles.append(self.dc.find_articulation_dof(self.art, "L_Joint_43"))
                    # Index Finger
                    self.left_hand_handles.append(self.dc.find_articulation_dof(self.art, "L_Joint_00"))
                    self.left_hand_handles.append(self.dc.find_articulation_dof(self.art, "L_Joint_01"))
                    # Middle Finger
                    self.left_hand_handles.append(self.dc.find_articulation_dof(self.art, "L_Joint_10"))
                    self.left_hand_handles.append(self.dc.find_articulation_dof(self.art, "L_Joint_11"))
                    # Ring Finger
                    self.left_hand_handles.append(self.dc.find_articulation_dof(self.art, "L_Joint_20"))
                    self.left_hand_handles.append(self.dc.find_articulation_dof(self.art, "L_Joint_21"))
                    # Little Finger
                    self.left_hand_handles.append(self.dc.find_articulation_dof(self.art, "L_Joint_30"))
                    self.left_hand_handles.append(self.dc.find_articulation_dof(self.art, "L_Joint_31"))

                    # Thumb
                    self.right_hand_handles.append(self.dc.find_articulation_dof(self.art, "R_Joint_40"))
                    self.right_hand_handles.append(self.dc.find_articulation_dof(self.art, "R_Joint_41"))
                    self.right_hand_handles.append(self.dc.find_articulation_dof(self.art, "R_Joint_42"))
                    self.right_hand_handles.append(self.dc.find_articulation_dof(self.art, "R_Joint_43"))
                    # Index Finger
                    self.right_hand_handles.append(self.dc.find_articulation_dof(self.art, "R_Joint_00"))
                    self.right_hand_handles.append(self.dc.find_articulation_dof(self.art, "R_Joint_01"))
                    # Middle Finger
                    self.right_hand_handles.append(self.dc.find_articulation_dof(self.art, "R_Joint_10"))
                    self.right_hand_handles.append(self.dc.find_articulation_dof(self.art, "R_Joint_11"))
                    # Ring Finge
                    self.right_hand_handles.append(self.dc.find_articulation_dof(self.art, "R_Joint_20"))
                    self.right_hand_handles.append(self.dc.find_articulation_dof(self.art, "R_Joint_21"))
                    # Little Finger
                    self.right_hand_handles.append(self.dc.find_articulation_dof(self.art, "R_Joint_30"))
                    self.right_hand_handles.append(self.dc.find_articulation_dof(self.art, "R_Joint_31"))


                    self.head_handles.append(self.dc.find_articulation_dof(self.art, "head_Yaw"))
                    self.head_handles.append(self.dc.find_articulation_dof(self.art, "head_Pitch"))
                    self.head_handles.append(self.dc.find_articulation_dof(self.art, "head_Roll"))


                    self.my_world.add_physics_callback("joint_control",
                                           callback_fn=self.joint_callback)
                    self.my_world.add_physics_callback("step_callback",
                                                    callback_fn=self.step_callback)
                    
                    self.initialized = True
                    self.sim_time = 0
                    self.tele_operation_cnt = 0
                    self.motion_circle_num = 0
                    self.euler0_old = np.zeros(3)
                    self.head_cmd_tele = self.init_head_end
                    self.left_arm_cmd_tele = self.init_left_arm_joint_angles
                    self.right_arm_cmd_tele = self.init_right_arm_joint_angles
                    self.left_hand_cmd_tele = self.init_left_hand_mag
                    self.right_hand_cmd_tele = self.init_right_hand_mag

            self.stp_cnt += 1

        omni.timeline.get_timeline_interface().stop()

    def step_callback(self, step_size):
        """ Print the step size to see if the simulation setting is fine
            Params
            ======
            step_size (float): shouldn't be set manually
        """
        # print("simulate with step: ", step_size)
        self.sim_time += self.physics_dt
        return

    def shut_down(self, kill_instantly: bool = True) -> None:
        """ defines how you prefer when all simulation steps are executed
            Params
            ======
            kill_instantly (bool): if True the simulation app will be closed when simulation is finished
                                   if False the simulation app will not close and you can further exam the simulation setups
        """
        if kill_instantly:
            del self.my_world
            self.thread.join()
            simulation_app.close()
            #self.thread.join()
        else:
            while simulation_app.is_running():
                simulation_app.update()

    def joint_callback(self, step_size) -> None:
        if self.initialized:
            if self.enable_robot_move and self.left_arm_position_cmd_list != []:

                self.left_arm_joint_positions = self.left_arm_position_cmd_list[0]
                self.left_arm_position_cmd_list.pop(0)
            

            if self.enable_robot_move and self.right_arm_position_cmd_list != []:
                self.right_arm_joint_positions = self.right_arm_position_cmd_list[0]
                self.right_arm_position_cmd_list.pop(0)

            if self.enable_robot_move and self.right_leg_position_cmd_list != []:
                self.right_leg_joint_positions = self.right_leg_position_cmd_list[0]
                self.right_leg_position_cmd_list.pop(0)
            
            if self.enable_robot_move and self.left_leg_position_cmd_list != []:
                self.left_leg_joint_positions = self.left_leg_position_cmd_list[0]
                self.left_leg_position_cmd_list.pop(0)


            

            if self.enable_robot_move and self.head_position_cmd_list != []:
                self.head_position_cmd = self.head_position_cmd_list[0]
                self.head_position_cmd_list.pop(0)

            
            if (self.left_hand_position_cmd != []):
                self.left_gripper_mag_cmd = self.left_hand_position_cmd[0]

                self.left_hand_joint_positions[0] = math.radians(95.) * self.left_gripper_mag_cmd[0]
                self.left_hand_joint_positions[1] = math.radians(20.) - math.radians(80.) * self.left_gripper_mag_cmd[1]
                self.left_hand_joint_positions[2] = math.radians(-30.) * self.left_gripper_mag_cmd[1]
                self.left_hand_joint_positions[3] = math.radians(0.)
                self.left_hand_joint_positions[4] = math.radians(-60.) * self.left_gripper_mag_cmd[2]
                self.left_hand_joint_positions[5] = math.radians(-90.) * self.left_gripper_mag_cmd[2]
                self.left_hand_joint_positions[6] = math.radians(-60.) * self.left_gripper_mag_cmd[3]
                self.left_hand_joint_positions[7] = math.radians(-90.) * self.left_gripper_mag_cmd[3]
                self.left_hand_joint_positions[8] = math.radians(-60.) * self.left_gripper_mag_cmd[4]
                self.left_hand_joint_positions[9] = math.radians(-90.) * self.left_gripper_mag_cmd[4]
                self.left_hand_joint_positions[10] = math.radians(-60.) * self.left_gripper_mag_cmd[5]
                self.left_hand_joint_positions[11] = math.radians(-90.) * self.left_gripper_mag_cmd[5]

                self.left_hand_position_cmd.pop(0)
          
            if (self.right_hand_position_cmd != []):
                self.right_gripper_mag_cmd = self.right_hand_position_cmd[0]

                self.right_hand_joint_positions[0] = math.radians(-95.) * self.right_gripper_mag_cmd[0]
                self.right_hand_joint_positions[1] = math.radians(-20.) + math.radians(80.) * self.right_gripper_mag_cmd[1]
                self.right_hand_joint_positions[2] = math.radians(-30.) * self.right_gripper_mag_cmd[1]
                self.right_hand_joint_positions[3] = math.radians(0.)
                self.right_hand_joint_positions[4] = math.radians(-60.) * self.right_gripper_mag_cmd[2]
                self.right_hand_joint_positions[5] = math.radians(-90.) * self.right_gripper_mag_cmd[2]
                self.right_hand_joint_positions[6] = math.radians(-60.) * self.right_gripper_mag_cmd[3]
                self.right_hand_joint_positions[7] = math.radians(-90.) * self.right_gripper_mag_cmd[3]
                self.right_hand_joint_positions[8] = math.radians(-60.) * self.right_gripper_mag_cmd[4]
                self.right_hand_joint_positions[9] = math.radians(-90.) * self.right_gripper_mag_cmd[4]
                self.right_hand_joint_positions[10] = math.radians(-60.) * self.right_gripper_mag_cmd[5]
                self.right_hand_joint_positions[11] = math.radians(-90.) * self.right_gripper_mag_cmd[5]

                self.right_hand_position_cmd.pop(0)


            for idx in range(7):
                self.dc.set_dof_position_target(self.left_arm_joint_handles[idx], 
                                                self.left_arm_joint_positions[idx])
                
            for idx in range(7):
                self.dc.set_dof_position_target(self.right_arm_joint_handles[idx], 
                                                self.right_arm_joint_positions[idx])
                
            for idx in range(12):
                self.dc.set_dof_position_target(self.left_hand_handles[idx], 
                                                self.left_hand_joint_positions[idx])
            for idx in range(12):
                self.dc.set_dof_position_target(self.right_hand_handles[idx], 
                                                self.right_hand_joint_positions[idx])    
            
            for idx in range(3):
                self.dc.set_dof_position_target(self.head_handles[idx], 
                                                self.head_position_cmd[idx])
                
            for idx in range(6):
                self.dc.set_dof_position_target(self.left_leg_joint_handles[idx], 
                                                self.left_leg_joint_positions[idx])
                
            for idx in range(6):
                self.dc.set_dof_position_target(self.right_leg_joint_handles[idx], 
                                                self.right_leg_joint_positions[idx])



if __name__ == "__main__":

    sim = My_Simulation()
    sim.play(num_steps=10000)

    simulation_app.close()
