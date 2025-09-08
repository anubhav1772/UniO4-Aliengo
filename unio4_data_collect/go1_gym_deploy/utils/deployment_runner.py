import copy
import time
import os

import math
import select

import numpy as np
import torch
import lcm

import threading

from go1_gym_deploy.utils.logger import MultiLogger
# from go1_gym_deploy.utils.T265_reader import RealSensePose
from go1_gym_deploy.utils.HDF5_recorder import HDF5_recorder
# from go1_gym_deploy.lcm_types.camera_message_lcmt import camera_message_lcmt
from go1_gym_deploy.lcm_types.PositionGravityState import PositionGravityState

from tqdm import tqdm

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")

class DeploymentRunner:
    def __init__(self, experiment_name="unnamed", se=None, log_root="."):
        self.agents = {}
        self.policy = None
        self.command_profile = None
        self.logger = MultiLogger()
        self.se = se
        self.vision_server = None

        self.log_root = log_root
        self.init_log_filename()
        self.control_agent_name = None
        self.command_agent_name = None

        self.triggered_commands = {i: None for i in range(4)} # command profiles for each action button on the controller
        self.button_states = np.zeros(4)

        self.is_currently_probing = False
        self.is_currently_logging = [False, False, False, False]

        self.hdf5_recorder = HDF5_recorder()
        # self.T265_reader = RealSensePose()

    def init_log_filename(self):
        datetime = time.strftime("%Y/%m_%d/%H_%M_%S")

        for i in range(100):
            try:
                os.makedirs(f"{self.log_root}/{datetime}_{i}")
                self.log_filename = f"{self.log_root}/{datetime}_{i}/log.pkl"
                return
            except FileExistsError:
                continue

    def add_open_loop_agent(self, agent, name):
        self.agents[name] = agent
        self.logger.add_robot(name, agent.env.cfg)

    def add_control_agent(self, agent, name):
        self.control_agent_name = name
        self.agents[name] = agent
        self.logger.add_robot(name, agent.env.cfg)

    def add_vision_server(self, vision_server):
        self.vision_server = vision_server

    def set_command_agents(self, name):
        self.command_agent = name

    def add_policy(self, policy):
        self.policy = policy

    def add_command_profile(self, command_profile):
        self.command_profile = command_profile
    
    def poll(self, cb=None):
        t = time.time()
        try:
            while True:
                timeout = 0.01
                rfds, wfds, efds = select.select([lc.fileno()], [], [], timeout)
                if rfds:
                    # print("message received!")
                    lc.handle()
                    # print(f'Freq {1. / (time.time() - t)} Hz'); t = time.time()
                else:
                    continue
                    # print(f'waiting for message... Freq {1. / (time.time() - t)} Hz'); t = time.time()
                #    if cb is not None:
                #        cb()
        except KeyboardInterrupt:
            pass
    
    def spin(self):
        self.run_thread = threading.Thread(target=self.poll, daemon=False)
        self.run_thread.start()

    # ORIGINAL
    # def calibrate(self, wait=True, low=False, count=None):
    #         # first, if the robot is not in nominal pose, move slowly to the nominal pose
    #         # print("agents_keys => "+str(self.agents.keys())) 
    #         # agents_keys => dict_keys(['hardware_closed_loop'])
    #         for agent_name in self.agents.keys():
    #             if hasattr(self.agents[agent_name], "get_obs"):
    #                 agent = self.agents[agent_name]
    #                 # get_obs() fetches the current environment observations 
    #                 # and stores in agent obs' variables (e.g., in agent.gravity_vector, agent.dof_vel, agent.dof_pos, etc.)
    #                 agent.get_obs()                 
    #                 joint_pos = agent.dof_pos
    #                 if low:
    #                     # Puts the legs in a crouched / recovery pose
    #                     # Used after first episode
    #                     # Thigh is raised (0.3) & Knee is bent backward (-0.7)
    #                     final_goal = np.array([0., 0.3, -0.7,  # FL
    #                                         0., 0.3, -0.7,  # FR
    #                                         0., 0.3, -0.7,  # RL
    #                                         0., 0.3, -0.7,  # RR
    #                                         ])
    #                 else:
    #                     # Used at start of the first episode
    #                     final_goal = np.zeros(12)
    #                 nominal_joint_pos = agent.default_dof_pos

    #                 # print(f"About to calibrate; the robot will stand [Press R2 to calibrate]")
    #                 if wait:
    #                     print("About to calibrate; the robot will stand [Press R2 to calibrate]")
    #                 else:
    #                     if low:
    #                         print("Quick recovery calibration (low posture) without waiting...")
    #                     else:
    #                         print("Calibrating robot automatically (standard posture, no wait)...")
                    
    #                 if(not wait): # wait = False
    #                     #print("Dog shuaidao!!!!")
    #                     print("The dog has fallen") # when wait is False, the count!=0, should we save the hdf5 file
    #                     self.hdf5_recorder.save_file() 
    #                 elif (wait and count!=0): # save only when episode ends or robot falls (not at the first step)
    #                     print("Normally record")
    #                     self.hdf5_recorder.save_file()
    #                 while wait:
    #                     self.button_states = self.command_profile.get_buttons()
    #                     if self.command_profile.state_estimator.right_lower_right_switch_pressed:
    #                         self.command_profile.state_estimator.right_lower_right_switch_pressed = False
    #                         break
                    
    #                 cal_action = np.zeros((agent.num_envs, agent.num_actions))
    #                 target_sequence = []
    #                 target = joint_pos - nominal_joint_pos
    #                 while np.max(np.abs(target - final_goal)) > 0.01:
    #                     # clip range (-0.05, 0.05) is basically setting the maximum allowed step size 
    #                     # in either direction (positive or negative). \
    #                     # Limits the values to stay within the boundaries
    #                     # np.clip(values, -0.05, 0.05) => values greater than 0.05 becomes 0.05, 
    #                     # while lower than -0.05 becomes -0.05. Values in between [-0.05, 0.05] remain unchanged
    #                     target -= np.clip((target - final_goal), -0.05, 0.05)
    #                     target_sequence += [copy.deepcopy(target)]
    #                 for target in target_sequence:
    #                     next_target = target
    #                     if isinstance(agent.cfg, dict):
    #                         hip_reduction = agent.cfg["control"]["hip_scale_reduction"]
    #                         action_scale = agent.cfg["control"]["action_scale"]
    #                     else:
    #                         hip_reduction = agent.cfg.control.hip_scale_reduction
    #                         action_scale = agent.cfg.control.action_scale

    #                     # >>> x = np.array([1, 2, 3, 4, 5, 6, 7, 9], 'float')
    #                     # >>> x
    #                     #     array([1., 2., 3., 4., 5., 6., 7., 9.])
    #                     # >>> x[[0, 3, 5]]/=2
    #                     # >>> x
    #                     #     array([0.5, 2. , 3. , 2. , 5. , 3. , 7. , 9. ])
    #                     next_target[[0, 3, 6, 9]] /= hip_reduction # here, 0, 3, 6, 9 array indices are for hip
    #                     next_target = next_target / action_scale
    #                     cal_action[:, 0:12] = next_target
    #                     is_calibrated = False
    #                     agent.step(torch.from_numpy(cal_action), calibrated=is_calibrated)
    #                     agent.get_obs()
    #                     time.sleep(0.05)

    #                 print("Starting pose calibrated [Press R2 to start controller]")
    #                 while True:
    #                     self.button_states = self.command_profile.get_buttons()
    #                     if self.command_profile.state_estimator.right_lower_right_switch_pressed:
    #                         self.command_profile.state_estimator.right_lower_right_switch_pressed = False
    #                         break

    #                 for agent_name in self.agents.keys():
    #                     obs = self.agents[agent_name].reset()
    #                     if agent_name == self.control_agent_name:
    #                         control_obs = obs

    #         return control_obs

    # MODIFIED
    def calibrate(self, wait=True, low=False, start_controller=True):
        # first, if the robot is not in nominal pose, move slowly to the nominal pose
        for agent_name in self.agents.keys():
            if hasattr(self.agents[agent_name], "get_obs"):
                agent = self.agents[agent_name]
                agent.get_obs()
                joint_pos = agent.dof_pos
                if low:
                    final_goal = np.array([0., 0.3, -0.7,
                                           0., 0.3, -0.7,
                                           0., 0.3, -0.7,
                                           0., 0.3, -0.7,])
                else:
                    final_goal = np.zeros(12)
                nominal_joint_pos = agent.default_dof_pos

                print(f"About to calibrate; the robot will stand [Press R2 to calibrate]")
                
                if(not wait):
                    print("Dog reset!!!!")
                    self.hdf5_recorder.save_file()
                else:
                    print("Normally record")
                    self.hdf5_recorder.save_file()
                while wait:
                    self.button_states = self.command_profile.get_buttons()
                    if self.command_profile.state_estimator.right_lower_right_switch_pressed:
                        self.command_profile.state_estimator.right_lower_right_switch_pressed = False
                        break
                
                cal_action = np.zeros((agent.num_envs, agent.num_actions))
                target_sequence = []
                target = joint_pos - nominal_joint_pos
                while np.max(np.abs(target - final_goal)) > 0.01:
                    target -= np.clip((target - final_goal), -0.05, 0.05)
                    target_sequence += [copy.deepcopy(target)]
                for target in target_sequence:
                    next_target = target
                    if isinstance(agent.cfg, dict):
                        hip_reduction = agent.cfg["control"]["hip_scale_reduction"]
                        action_scale = agent.cfg["control"]["action_scale"]
                    else:
                        hip_reduction = agent.cfg.control.hip_scale_reduction
                        action_scale = agent.cfg.control.action_scale

                    next_target[[0, 3, 6, 9]] /= hip_reduction
                    next_target = next_target / action_scale
                    cal_action[:, 0:12] = next_target
                    is_calibrated = False
                    agent.step(torch.from_numpy(cal_action), calibrated=is_calibrated)
                    agent.get_obs()
                    time.sleep(0.05)
                if start_controller:
                    print("Starting pose calibrated [Press R2 to start controller]")
                    while True:
                        self.button_states = self.command_profile.get_buttons()
                        if self.command_profile.state_estimator.right_lower_right_switch_pressed:
                            self.command_profile.state_estimator.right_lower_right_switch_pressed = False
                            break

                    for agent_name in self.agents.keys():
                        obs = self.agents[agent_name].reset()
                        if agent_name == self.control_agent_name:
                            control_obs = obs

                    return control_obs

    # MODIFIED VERSION: ROBOT SIT AFTER EPISODE ENDS
    def run(self, num_log_steps=1000, max_steps=10000, logging=True):
        assert self.control_agent_name is not None, "cannot deploy, runner has no control agent!"
        assert self.policy is not None, "cannot deploy, runner has no policy!"
        assert self.command_profile is not None, "cannot deploy, runner has no command profile!"

        # TODO: add basic test for comms

        for agent_name in self.agents.keys():
            obs = self.agents[agent_name].reset()
            if agent_name == self.control_agent_name:
                control_obs = obs

        self.gravity = None

        self.camera_data = None

        control_obs = self.calibrate(wait=True)
        obs_record = control_obs["obs"][0,:].detach().cpu().numpy().tolist()
        count = 0
        valuable_count = 0
        # now, run control loop
        try:
            with tqdm(total=max_steps) as pbar:
                iterations = 0
                while count < max_steps:
                    done = False

                    if count != 0:
                        control_obs = self.calibrate(wait=False, low=True)
                    obs_record = control_obs["obs"][0,:].detach().cpu().numpy().tolist()
                    history_obs = control_obs["obs_history"][0,:].detach().cpu().numpy()
                    
                    time_before_append = time.time()
                    # if(not self.T265_reader.appendPoseData(obs_record)):
                    #     count = valuable_count
                    #     self.T265_reader.reset()
                    #     continue

                    obs_camera_subscribtion = lc.subscribe("camera_python", self._camera_cb)
                    self.spin()
                    if (self.camera_data is not None and len(self.camera_data.tolist()) != 18):
                        continue
                    
                    time_after_append = time.time()
                    # print("befor while not done delta time {}s".format(time_after_append-time_before_append))

                    while not done:
                        is_calibrated = True
                        policy_info = {}
                        # action, a_logprob = self.policy(control_obs["obs_history"].to(self.device))
                        action = self.policy(control_obs, policy_info)
                        act_record = action[0, :12].detach().cpu().numpy()

                        #cat next observation
                        for agent_name in self.agents.keys():
                            obs, ret, _, info = self.agents[agent_name].step(action, calibrated=is_calibrated)
                            if agent_name == self.control_agent_name:
                                next_control_obs, control_ret, control_done, control_info = obs, ret, _, info
                                next_obs_record = next_control_obs["obs"][0,:].detach().cpu().numpy().tolist()
                                next_history_obs = next_control_obs["obs_history"][0,:].detach().cpu().numpy()
                                #check t265 and cat into obs
                        
                        time_before_append = time.time()

                        # if(not self.T265_reader.appendPoseData(next_obs_record)):
                        #     count = valuable_count
                        #     break
                        if (self.camera_data is not None and len(self.camera_data.tolist()) != 18):
                            continue

                        time_after_append = time.time()
                        if(time_after_append-time_before_append>0.1):
                            count = valuable_count
                            # print("delta time {}s".format(time_after_append-time_before_append))
                            break

                        # bad orientation emergency stop
                        rpy = self.agents[self.control_agent_name].se.get_rpy()
                        
                        if abs(rpy[0]) > 1.6 or abs(rpy[1]) > 1.6:
                            valuable_count = count
                            done = True
                        else:
                            done = False
                        if count == max_steps - 1:
                            done = True

                        if done and count != (max_steps - 1):
                            dw = True
                        else:
                            dw = False

                        obs_camera = self.camera_data.tolist()
                        while len(obs_camera) != 18: # not necessary
                            continue
                        
                        # print(obs_camera)

                        obs_record.extend(obs_camera)
                        if len(next_obs_record) == 58:
                            next_obs_record.extend(obs_camera)
                        # print(f"obs_record shape: {len(obs_record)}")
                        self.hdf5_recorder.record_step(state=np.array(obs_record), action=act_record, next_state = np.array(next_obs_record), done=done)
                        count += 1
                        obs_record = next_obs_record[:58]
                        control_obs = next_control_obs
                        history_obs = next_history_obs
                        iterations += 1
                        pbar.update(1)  # 更新进度条

            # finally, return to the nominal pose
            _ = self.calibrate(wait=False, low=True, start_controller = False)
            # self.logger.save(self.log_filename)

            # return self.hdf5_recorder

        except KeyboardInterrupt:
            self.logger.save(self.log_filename)


    def _camera_cb(self, channel, data):
        msg = PositionGravityState.decode(data)
        self.gravity = np.array(msg.gravity)
        self.camera_data = np.array(msg.data)
        #print(self.camera_data)

    # ORIGINAL VERSION: ROBOT DOESNOT SIT BUT KEEPS STANDING
    # def run(self, num_log_steps=1000, max_steps=10000, logging=True):
    #     assert self.control_agent_name is not None, "cannot deploy, runner has no control agent!"
    #     assert self.policy is not None, "cannot deploy, runner has no policy!"
    #     assert self.command_profile is not None, "cannot deploy, runner has no command profile!"

    #     # TODO: add basic test for comms

    #     for agent_name in self.agents.keys():
    #         obs = self.agents[agent_name].reset()
    #         #print("agent obs: "+str(obs))
    #         if agent_name == self.control_agent_name:
    #             control_obs = obs

    #     self.gravity = None

    #     self.camera_data = None

    #     # All joints set to 0.0
    #     # Standing straight / Neutral pose
    #     # Used at the start of the first episode
    #     control_obs = self.calibrate(wait=True, count=0)
    #     print('printing control obs returned after calibration step: '+str(control_obs))
    #     obs_record = control_obs["obs"][0,:].detach().cpu().numpy().tolist()
    #     count = 0
    #     # now, run control loop
    #     try:
    #         while count < max_steps:
    #             done = False
    #             print('dog reset after press R2')

    #             if count != 0:
    #                 # Skip waiting and go to a crouched, safe posture quickly
    #                 control_obs = self.calibrate(wait=False, low=True)
    #                 obs_record = control_obs["obs"][0,:].detach().cpu().numpy().tolist()
    #             #print('obs_len: {}'.format(len(obs_record)))
    #             time_before_append = time.time()
    #             # T265 Tracking Camera
    #             # -------------------------------------
    #             """
    #             if(not self.T265_reader.appendPoseData(obs_record)):
    #                 self.T265_reader.reset()
    #                 continue
    #             """
    #             # obs_camera_subscribtion = lc.subscribe("POSITION_GRAVITY_STATE", self._camera_cb)
    #             obs_camera_subscribtion = lc.subscribe("camera_python", self._camera_cb)
    #             self.spin()
    #             if (self.camera_data is not None and len(self.camera_data.tolist()) != 18):
    #                 # obs_camera_subscribtion = lc.subscribe("POSITION_GRAVITY_STATE", self._camera_cb)
    #                 # self.T265_reader.reset()
    #                 continue
                
    #             # -------------------------------------
    #             time_after_append = time.time()
    #             print("before while not done delta time {}s".format(time_after_append-time_before_append))

    #             # Keep running the loop until done becomes True
    #             while not done:
    #                 is_calibrated = True
    #                 policy_info = {}
    #                 action = self.policy(control_obs, policy_info)
    #                 act_record = action[0, :12].detach().cpu().numpy()

    #                 #cat next observation
    #                 for agent_name in self.agents.keys():
    #                     obs, ret, _, info = self.agents[agent_name].step(action, calibrated=is_calibrated)
    #                     if agent_name == self.control_agent_name:
    #                         next_control_obs, control_ret, control_done, control_info = obs, ret, _, info
    #                         next_obs_record = next_control_obs["obs"][0,:].detach().cpu().numpy().tolist()
    #                         #check t265 and cat into obs
    #                 time_before_append = time.time()
                    
    #                 # T265 Tracking Camera
    #                 # -------------------------------------
    #                 """
    #                 if(not self.T265_reader.appendPoseData(next_obs_record)):
    #                     break
    #                 """
    #                 if (self.camera_data is not None and len(self.camera_data.tolist()) != 18):
    #                     break
    #                 # -------------------------------------
    #                 time_after_append = time.time()
                    
    #                 if(time_after_append-time_before_append>0.1):
    #                     print("delta time {}s".format(time_after_append-time_before_append))
    #                     break

    #                 # bad orientation emergency stop
    #                 rpy = self.agents[self.control_agent_name].se.get_rpy()
                    
    #                 if abs(rpy[0]) > 1.6 or abs(rpy[1]) > 1.6 or (count == max_steps):
    #                     done = True
    #                 else:
    #                     done = False
                    
    #                 obs_camera = self.camera_data.tolist()
    #                 while len(obs_camera) != 18: # not necessary
    #                     continue

    #                 print('$'*20)
    #                 print("camera observations: "+str(obs_camera))
    #                 print('#'*20)
    #                 obs_record.extend(obs_camera) ##
    #                 self.hdf5_recorder.record_step(state=np.array(obs_record), action=act_record, next_state = np.array(next_obs_record), done=done)
    #                 # self.hdf5_recorder.save_file()
    #                 count += 1
    #                 print('count------------------------: {}'.format(count))
    #                 obs_record = next_obs_record
    #                 control_obs = next_control_obs

    #                 if count == max_steps:
    #                     # finally, return to the nominal pose
    #                     control_obs = self.calibrate(wait=True, count=count)
    #                     self.logger.save(self.log_filename)
    #                     count = 0
            
    #         # control_obs = self.calibrate(wait=True, count=count)
    #         # self.logger.save(self.log_filename)

    #     except KeyboardInterrupt:
    #         self.logger.save(self.log_filename)

    # def run(self, num_log_steps=1000, max_steps=10000, logging=True):
    #     assert self.control_agent_name is not None, "cannot deploy, runner has no control agent!"
    #     assert self.policy is not None, "cannot deploy, runner has no policy!"
    #     assert self.command_profile is not None, "cannot deploy, runner has no command profile!"

    #     # TODO: add basic test for comms

    #     for agent_name in self.agents.keys():
    #         obs = self.agents[agent_name].reset()
    #         #print("agent obs: "+str(obs))
    #         if agent_name == self.control_agent_name:
    #             control_obs = obs

    #     self.gravity = None

    #     self.camera_data = None

    #     # All joints set to 0.0
    #     # Standing straight / Neutral pose
    #     # Used at the start of the first episode
    #     control_obs = self.calibrate(wait=True, count=0)
    #     print('printing control obs returned after calibration step: '+str(control_obs))
    #     obs_record = control_obs["obs"][0,:].detach().cpu().numpy().tolist()
    #     count = 0
    #     # now, run control loop
    #     try:
    #         while count < max_steps:
    #             done = False
    #             print('dog reset after press R2')

    #             if count != 0:
    #                 # Skip waiting and go to a crouched, safe posture quickly
    #                 control_obs = self.calibrate(wait=False, low=True)
    #                 obs_record = control_obs["obs"][0,:].detach().cpu().numpy().tolist()
    #             #print('obs_len: {}'.format(len(obs_record)))
    #             time_before_append = time.time()
    #             # T265 Tracking Camera
    #             # -------------------------------------
    #             """
    #             if(not self.T265_reader.appendPoseData(obs_record)):
    #                 self.T265_reader.reset()
    #                 continue
    #             """
    #             # obs_camera_subscribtion = lc.subscribe("POSITION_GRAVITY_STATE", self._camera_cb)
    #             obs_camera_subscribtion = lc.subscribe("camera_python", self._camera_cb)
    #             self.spin()
    #             if (self.camera_data is not None and len(self.camera_data.tolist()) != 18):
    #                 # obs_camera_subscribtion = lc.subscribe("POSITION_GRAVITY_STATE", self._camera_cb)
    #                 # self.T265_reader.reset()
    #                 continue
                
    #             # -------------------------------------
    #             time_after_append = time.time()
    #             print("before while not done delta time {}s".format(time_after_append-time_before_append))

    #             fall_flag = False
    #             # Keep running the loop until done becomes True
    #             while not done:
    #                 is_calibrated = True
    #                 policy_info = {}
    #                 action = self.policy(control_obs, policy_info)
    #                 act_record = action[0, :12].detach().cpu().numpy()

    #                 #cat next observation
    #                 for agent_name in self.agents.keys():
    #                     obs, ret, _, info = self.agents[agent_name].step(action, calibrated=is_calibrated)
    #                     if agent_name == self.control_agent_name:
    #                         next_control_obs, control_ret, control_done, control_info = obs, ret, _, info
    #                         next_obs_record = next_control_obs["obs"][0,:].detach().cpu().numpy().tolist()
    #                         #check t265 and cat into obs
    #                 time_before_append = time.time()
                    
    #                 # T265 Tracking Camera
    #                 # -------------------------------------
    #                 """
    #                 if(not self.T265_reader.appendPoseData(next_obs_record)):
    #                     break
    #                 """
    #                 if (self.camera_data is not None and len(self.camera_data.tolist()) != 18):
    #                     break
    #                 # -------------------------------------
    #                 time_after_append = time.time()
                    
    #                 if(time_after_append-time_before_append>0.1):
    #                     print("delta time {}s".format(time_after_append-time_before_append))
    #                     break

    #                 # bad orientation emergency stop
    #                 rpy = self.agents[self.control_agent_name].se.get_rpy()
                    
    #                 if abs(rpy[0]) > 1.6 or abs(rpy[1]) > 1.6:
    #                     done = True
    #                     fall_flag = True
    #                 elif count == max_steps:
    #                     done = True
    #                 else:
    #                     done = False
                    
    #                 obs_camera = self.camera_data.tolist()
    #                 while len(obs_camera) != 18: # not necessary
    #                     continue

    #                 print('$'*20)
    #                 print("camera observations: "+str(obs_camera))
    #                 print('#'*20)
    #                 obs_record.extend(obs_camera) ##
    #                 self.hdf5_recorder.record_step(state=np.array(obs_record), action=act_record, next_state = np.array(next_obs_record), done=done)
    #                 # self.hdf5_recorder.save_file()
    #                 count += 1
    #                 print('count------------------------: {}'.format(count))
    #                 obs_record = next_obs_record
    #                 control_obs = next_control_obs
                
    #             # In case of fall, save the episode till that point and exit
    #             if fall_flag:
    #                 # self.hdf5_recorder.save_file()
    #                 break

    #         # Finally, return to the nominal pose
    #         control_obs = self.calibrate(wait=True, count=count)
    #         self.logger.save(self.log_filename)

    #     except KeyboardInterrupt:
    #         self.logger.save(self.log_filename)


    
























# import copy
# import time
# import os

# import numpy as np
# import torch

# import lcm

# from go1_gym_deploy.utils.logger import MultiLogger
# # from go1_gym_deploy.utils.T265_reader import RealSensePose
# from go1_gym_deploy.utils.HDF5_recorder import HDF5_recorder
# # from go1_gym_deploy.lcm_types.camera_message_lcmt import camera_message_lcmt
# from go1_gym_deploy.lcm_types.PositionGravityState import PositionGravityState

# lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")

# class DeploymentRunner:
#     def __init__(self, experiment_name="unnamed", se=None, log_root="."):
#         self.agents = {}
#         self.policy = None
#         self.command_profile = None
#         self.logger = MultiLogger()
#         self.se = se
#         self.vision_server = None

#         self.log_root = log_root
#         self.init_log_filename()
#         self.control_agent_name = None
#         self.command_agent_name = None

#         self.triggered_commands = {i: None for i in range(4)} # command profiles for each action button on the controller
#         self.button_states = np.zeros(4)

#         self.is_currently_probing = False
#         self.is_currently_logging = [False, False, False, False]

#         self.hdf5_recorder = HDF5_recorder()
#         # self.T265_reader = RealSensePose()

#     def init_log_filename(self):
#         datetime = time.strftime("%Y/%m_%d/%H_%M_%S")

#         for i in range(100):
#             try:
#                 os.makedirs(f"{self.log_root}/{datetime}_{i}")
#                 self.log_filename = f"{self.log_root}/{datetime}_{i}/log.pkl"
#                 return
#             except FileExistsError:
#                 continue

#     def add_open_loop_agent(self, agent, name):
#         self.agents[name] = agent
#         self.logger.add_robot(name, agent.env.cfg)

#     def add_control_agent(self, agent, name):
#         self.control_agent_name = name
#         self.agents[name] = agent
#         self.logger.add_robot(name, agent.env.cfg)

#     def add_vision_server(self, vision_server):
#         self.vision_server = vision_server

#     def set_command_agents(self, name):
#         self.command_agent = name

#     def add_policy(self, policy):
#         self.policy = policy

#     def add_command_profile(self, command_profile):
#         self.command_profile = command_profile
    
#     def _camera_cb(self, channel, data):
#         msg = PositionGravityState.decode(data)

#         self.gravity = np.array(msg.gravity)

#         self.camera_data = np.array(msg.data)

#     def calibrate(self, wait=True, low=False):
#         # first, if the robot is not in nominal pose, move slowly to the nominal pose
#         # print("agents_keys => "+str(self.agents.keys())) 
#         # agents_keys => dict_keys(['hardware_closed_loop'])
#         for agent_name in self.agents.keys():
#             if hasattr(self.agents[agent_name], "get_obs"):
#                 agent = self.agents[agent_name]
#                 # get_obs() fetches the current environment observations 
#                 # and stores in agent obs' variables (e.g., in agent.gravity_vector, agent.dof_vel, agent.dof_pos, etc.)
#                 agent.get_obs()                 
#                 joint_pos = agent.dof_pos
#                 if low:
#                     final_goal = np.array([0., 0.3, -0.7,
#                                            0., 0.3, -0.7,
#                                            0., 0.3, -0.7,
#                                            0., 0.3, -0.7,])
#                 else:
#                     final_goal = np.zeros(12)
#                 nominal_joint_pos = agent.default_dof_pos

#                 print(f"About to calibrate; the robot will stand [Press R2 to calibrate]")
                
#                 if(not wait):
#                     #print("Dog shuaidao!!!!")
#                     print("The dog has fallen")
#                     self.hdf5_recorder.save_file()
#                 else:
#                     print("Normally record")
#                     self.hdf5_recorder.save_file()
#                 while wait:
#                     self.button_states = self.command_profile.get_buttons()
#                     if self.command_profile.state_estimator.right_lower_right_switch_pressed:
#                         self.command_profile.state_estimator.right_lower_right_switch_pressed = False
#                         break
                
#                 cal_action = np.zeros((agent.num_envs, agent.num_actions))
#                 target_sequence = []
#                 target = joint_pos - nominal_joint_pos
#                 while np.max(np.abs(target - final_goal)) > 0.01:
#                     # clip range (-0.05, 0.05) is basically setting the maximum allowed step size 
#                     # in either direction (positive or negative). \
#                     # Limits the values to stay within the boundaries
#                     # np.clip(values, -0.05, 0.05) => values greater than 0.05 becomes 0.05, 
#                     # while lower than -0.05 becomes -0.05. Values in between [-0.05, 0.05] remain unchanged
#                     target -= np.clip((target - final_goal), -0.05, 0.05)
#                     target_sequence += [copy.deepcopy(target)]
#                 for target in target_sequence:
#                     next_target = target
#                     if isinstance(agent.cfg, dict):
#                         hip_reduction = agent.cfg["control"]["hip_scale_reduction"]
#                         action_scale = agent.cfg["control"]["action_scale"]
#                     else:
#                         hip_reduction = agent.cfg.control.hip_scale_reduction
#                         action_scale = agent.cfg.control.action_scale

#                     # >>> x = np.array([1, 2, 3, 4, 5, 6, 7, 9], 'float')
#                     # >>> x
#                     #     array([1., 2., 3., 4., 5., 6., 7., 9.])
#                     # >>> x[[0, 3, 5]]/=2
#                     # >>> x
#                     #     array([0.5, 2. , 3. , 2. , 5. , 3. , 7. , 9. ])
#                     next_target[[0, 3, 6, 9]] /= hip_reduction # here, 0, 3, 6, 9 array indices are for hip
#                     next_target = next_target / action_scale
#                     cal_action[:, 0:12] = next_target
#                     is_calibrated = False
#                     agent.step(torch.from_numpy(cal_action), calibrated=is_calibrated)
#                     agent.get_obs()
#                     time.sleep(0.05)

#                 print("Starting pose calibrated [Press R2 to start controller]")
#                 while True:
#                     self.button_states = self.command_profile.get_buttons()
#                     if self.command_profile.state_estimator.right_lower_right_switch_pressed:
#                         self.command_profile.state_estimator.right_lower_right_switch_pressed = False
#                         break

#                 for agent_name in self.agents.keys():
#                     obs = self.agents[agent_name].reset()
#                     if agent_name == self.control_agent_name:
#                         control_obs = obs

#         return control_obs


#     def run(self, num_log_steps=1000000000, max_steps=100000000, logging=True):
#         assert self.control_agent_name is not None, "cannot deploy, runner has no control agent!"
#         assert self.policy is not None, "cannot deploy, runner has no policy!"
#         assert self.command_profile is not None, "cannot deploy, runner has no command profile!"

#         # TODO: add basic test for comms
#         #print(50*'^')
#         #print(self.control_agent_name)
#         #print(self.agents.keys())
#         for agent_name in self.agents.keys():
#             obs = self.agents[agent_name].reset()
#             #print("agent obs: "+str(obs))
#             if agent_name == self.control_agent_name:
#                 control_obs = obs
#         #print(50*'-')

#         control_obs = self.calibrate(wait=True)
#         print('printing control obs returned after calibration step: '+str(control_obs))
#         obs_record = control_obs["obs"][0,:].detach().cpu().numpy().tolist()
#         count = 0
#         # now, run control loop
#         try:
#             while count < max_steps:
#                 done = False
#                 print('dog reset after press r2')

#                 if count != 0:
#                     control_obs = self.calibrate(wait=False, low=True)
#                     obs_record = control_obs["obs"][0,:].detach().cpu().numpy().tolist()
#                 print('obs_len: {}'.format(len(obs_record)))
#                 time_before_append = time.time()
#                 # T265 Tracking Camera
#                 # -------------------------------------
#                 # if(not self.T265_reader.appendPoseData(obs_record)):
#                 #     self.T265_reader.reset()
#                 #     continue
#                 # -------------------------------------
                
#                 if len(self.camera_data.tolist()) != 18:
#                     obs_camera_subscribtion = lc.subscribe("POSITION_GRAVITY_STATE", self._camera_cb)
#                     # self.T265_reader.reset()
#                     continue

#                 time_after_append = time.time()
#                 print("before while not done delta time {}s".format(time_after_append-time_before_append))

#                 while not done:
#                     is_calibrated = True
#                     policy_info = {}
#                     action = self.policy(control_obs, policy_info)
#                     act_record = action[0, :12].detach().cpu().numpy()

#                     #cat next observation
#                     for agent_name in self.agents.keys():
#                         obs, ret, _, info = self.agents[agent_name].step(action, calibrated=is_calibrated)
#                         if agent_name == self.control_agent_name:
#                             next_control_obs, control_ret, control_done, control_info = obs, ret, _, info
#                             next_obs_record = next_control_obs["obs"][0,:].detach().cpu().numpy().tolist()
#                             #check t265 and cat into obs
#                     time_before_append = time.time()
                    
#                     # T265 Tracking Camera
#                     # -------------------------------------
#                     # if(not self.T265_reader.appendPoseData(next_obs_record)):
#                     #     break
#                     # -------------------------------------

#                     if len(self.camera_data.tolist()) != 18:
#                         break

#                     time_after_append = time.time()
                    
#                     if(time_after_append-time_before_append>0.1):
#                         print("delta time {}s".format(time_after_append-time_before_append))
#                         break

#                     # bad orientation emergency stop
#                     rpy = self.agents[self.control_agent_name].se.get_rpy()
                    
#                     if abs(rpy[0]) > 1.6 or abs(rpy[1]) > 1.6:
#                         done = True
#                     else:
#                         done = False

#                     self.hdf5_recorder.record_step(state=np.array(obs_record), action=act_record, next_state = np.array(next_obs_record), done=done)
#                     count += 1
#                     print('count------------------------: {}'.format(count))
#                     obs_record = next_obs_record
#                     control_obs = next_control_obs


#             # finally, return to the nominal pose
#             control_obs = self.calibrate(wait=True)
#             self.logger.save(self.log_filename)

#         except KeyboardInterrupt:
#             self.logger.save(self.log_filename)


    
