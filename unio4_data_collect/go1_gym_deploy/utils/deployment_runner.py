import copy
import time
import os

import numpy as np
import torch

from go1_gym_deploy.utils.logger import MultiLogger
#from go1_gym_deploy.utils.T265_reader import RealSensePose
from go1_gym_deploy.utils.HDF5_recorder import HDF5_recorder

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
        #self.T265_reader = RealSensePose()

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

    def calibrate(self, wait=True, low=False):
        # first, if the robot is not in nominal pose, move slowly to the nominal pose
        # print("agents_keys => "+str(self.agents.keys())) 
        # agents_keys => dict_keys(['hardware_closed_loop'])
        for agent_name in self.agents.keys():
            if hasattr(self.agents[agent_name], "get_obs"):
                agent = self.agents[agent_name]
                # get_obs() fetches the current environment observations 
                # and stores in agent obs' variables (e.g., in agent.gravity_vector, agent.dof_vel, agent.dof_pos, etc.)
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
                    #print("Dog shuaidao!!!!")
                    print("The dog has fallen")
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
                    # clip range (-0.05, 0.05) is basically setting the maximum allowed step size 
                    # in either direction (positive or negative). \
                    # Limits the values to stay within the boundaries
                    # np.clip(values, -0.05, 0.05) => values greater than 0.05 becomes 0.05, 
                    # while lower than -0.05 becomes -0.05. Values in between [-0.05, 0.05] remain unchanged
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

                    # >>> x = np.array([1, 2, 3, 4, 5, 6, 7, 9], 'float')
                    # >>> x
                    #     array([1., 2., 3., 4., 5., 6., 7., 9.])
                    # >>> x[[0, 3, 5]]/=2
                    # >>> x
                    #     array([0.5, 2. , 3. , 2. , 5. , 3. , 7. , 9. ])
                    next_target[[0, 3, 6, 9]] /= hip_reduction # here, 0, 3, 6, 9 array indices are for hip
                    next_target = next_target / action_scale
                    cal_action[:, 0:12] = next_target
                    agent.step(torch.from_numpy(cal_action))
                    agent.get_obs()
                    time.sleep(0.05)

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


    def run(self, num_log_steps=1000000000, max_steps=100000000, logging=True):
        assert self.control_agent_name is not None, "cannot deploy, runner has no control agent!"
        assert self.policy is not None, "cannot deploy, runner has no policy!"
        assert self.command_profile is not None, "cannot deploy, runner has no command profile!"

        # TODO: add basic test for comms
        #print(50*'^')
        #print(self.control_agent_name)
        #print(self.agents.keys())
        for agent_name in self.agents.keys():
            obs = self.agents[agent_name].reset()
            #print("agent obs: "+str(obs))
            if agent_name == self.control_agent_name:
                control_obs = obs
        #print(50*'-')

        control_obs = self.calibrate(wait=True)
        obs_record = control_obs["obs"][0,:].detach().cpu().numpy().tolist()
        count = 0
        # now, run control loop
        try:
            while count < max_steps:
                done = False
                print('dog reset after press r2')

                if count != 0:
                    control_obs = self.calibrate(wait=False, low=True)
                    obs_record = control_obs["obs"][0,:].detach().cpu().numpy().tolist()
                print('obs_len: {}'.format(len(obs_record)))
                time_before_append = time.time()
                # if(not self.T265_reader.appendPoseData(obs_record)):
                #     self.T265_reader.reset()
                #     continue
                time_after_append = time.time()
                print("befor while not done delta time {}s".format(time_after_append-time_before_append))

                while not done:
                    policy_info = {}
                    action = self.policy(control_obs, policy_info)
                    act_record = action[0, :12].detach().cpu().numpy()

                    #cat next observation
                    for agent_name in self.agents.keys():
                        obs, ret, _, info = self.agents[agent_name].step(action)
                        if agent_name == self.control_agent_name:
                            next_control_obs, control_ret, control_done, control_info = obs, ret, _, info
                            next_obs_record = next_control_obs["obs"][0,:].detach().cpu().numpy().tolist()
                            #check t265 and cat into obs
                    time_before_append = time.time()

                    # if(not self.T265_reader.appendPoseData(next_obs_record)):
                    #     break

                    time_after_append = time.time()
                    
                    if(time_after_append-time_before_append>0.1):
                        print("delta time {}s".format(time_after_append-time_before_append))
                        break

                    # bad orientation emergency stop
                    rpy = self.agents[self.control_agent_name].se.get_rpy()
                    
                    if abs(rpy[0]) > 1.6 or abs(rpy[1]) > 1.6:
                        done = True
                    else:
                        done = False

                    self.hdf5_recorder.record_step(state=np.array(obs_record), action=act_record, next_state = np.array(next_obs_record), done=done)
                    count += 1
                    print('count------------------------: {}'.format(count))
                    obs_record = next_obs_record
                    control_obs = next_control_obs


            # finally, return to the nominal pose
            control_obs = self.calibrate(wait=True)
            self.logger.save(self.log_filename)

        except KeyboardInterrupt:
            self.logger.save(self.log_filename)


    