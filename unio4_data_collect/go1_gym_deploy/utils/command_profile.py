import torch
from go1_gym_deploy.utils.generate_path import VelocityProfile      

class CommandProfile:
    def __init__(self, dt, max_time_s=10.):
        self.dt = dt
        self.max_timestep = int(max_time_s / self.dt)
        self.commands = torch.zeros((self.max_timestep, 9))
        self.start_time = 0
        self.gait_active = False

    def get_command(self, t):
        timestep = int((t - self.start_time) / self.dt)
        timestep = min(timestep, self.max_timestep - 1)
        return self.commands[timestep, :]

    def get_buttons(self):
        return [0, 0, 0, 0]

    def reset(self, reset_time):
        self.start_time = reset_time


class ConstantAccelerationProfile(CommandProfile):
    def __init__(self, dt, max_speed, accel_time, zero_buf_time=0):
        super().__init__(dt)
        zero_buf_timesteps = int(zero_buf_time / self.dt)
        accel_timesteps = int(accel_time / self.dt)
        self.commands[:zero_buf_timesteps] = 0
        self.commands[zero_buf_timesteps:zero_buf_timesteps + accel_timesteps, 0] = torch.arange(0, max_speed,
                                                                                                 step=max_speed / accel_timesteps)
        self.commands[zero_buf_timesteps + accel_timesteps:, 0] = max_speed


class ElegantForwardProfile(CommandProfile):
    def __init__(self, dt, max_speed, accel_time, duration, deaccel_time, zero_buf_time=0):
        import numpy as np

        zero_buf_timesteps = int(zero_buf_time / dt)
        accel_timesteps = int(accel_time / dt)
        duration_timesteps = int(duration / dt)
        deaccel_timesteps = int(deaccel_time / dt)

        total_time_s = zero_buf_time + accel_time + duration + deaccel_time

        super().__init__(dt, total_time_s)

        x_vel_cmds = [0] * zero_buf_timesteps + [*np.linspace(0, max_speed, accel_timesteps)] + \
                     [max_speed] * duration_timesteps + [*np.linspace(max_speed, 0, deaccel_timesteps)]

        self.commands[:len(x_vel_cmds), 0] = torch.Tensor(x_vel_cmds)


class ElegantYawProfile(CommandProfile):
    def __init__(self, dt, max_speed, zero_buf_time, accel_time, duration, deaccel_time, yaw_rate):
        import numpy as np

        zero_buf_timesteps = int(zero_buf_time / dt)
        accel_timesteps = int(accel_time / dt)
        duration_timesteps = int(duration / dt)
        deaccel_timesteps = int(deaccel_time / dt)

        total_time_s = zero_buf_time + accel_time + duration + deaccel_time

        super().__init__(dt, total_time_s)

        x_vel_cmds = [0] * zero_buf_timesteps + [*np.linspace(0, max_speed, accel_timesteps)] + \
                     [max_speed] * duration_timesteps + [*np.linspace(max_speed, 0, deaccel_timesteps)]

        yaw_vel_cmds = [0] * zero_buf_timesteps + [0] * accel_timesteps + \
                       [yaw_rate] * duration_timesteps + [0] * deaccel_timesteps

        self.commands[:len(x_vel_cmds), 0] = torch.Tensor(x_vel_cmds)
        self.commands[:len(yaw_vel_cmds), 2] = torch.Tensor(yaw_vel_cmds)


class ElegantGaitProfile(CommandProfile):
    def __init__(self, dt, filename):
        import numpy as np
        import json

        with open(f'../command_profiles/{filename}', 'r') as file:
                command_sequence = json.load(file)

        len_command_sequence = len(command_sequence["x_vel_cmd"])
        total_time_s = int(len_command_sequence / dt)

        super().__init__(dt, total_time_s)

        self.commands[:len_command_sequence, 0] = torch.Tensor(command_sequence["x_vel_cmd"])
        self.commands[:len_command_sequence, 2] = torch.Tensor(command_sequence["yaw_vel_cmd"])
        self.commands[:len_command_sequence, 3] = torch.Tensor(command_sequence["height_cmd"])
        self.commands[:len_command_sequence, 4] = torch.Tensor(command_sequence["frequency_cmd"])
        self.commands[:len_command_sequence, 5] = torch.Tensor(command_sequence["offset_cmd"])
        self.commands[:len_command_sequence, 6] = torch.Tensor(command_sequence["phase_cmd"])
        self.commands[:len_command_sequence, 7] = torch.Tensor(command_sequence["bound_cmd"])
        self.commands[:len_command_sequence, 8] = torch.Tensor(command_sequence["duration_cmd"])

class RCControllerProfile(CommandProfile):
    def __init__(self, dt, state_estimator, x_scale=1.0, y_scale=1.0, yaw_scale=1.0, probe_vel_multiplier=1.0, max_steps=250):
        super().__init__(dt)
        self.state_estimator = state_estimator
        self.x_scale = x_scale
        self.y_scale = y_scale
        self.yaw_scale = yaw_scale

        self.probe_vel_multiplier = probe_vel_multiplier

        self.triggered_commands = {i: None for i in range(4)}  # command profiles for each action button on the controller
        self.currently_triggered = [0, 0, 0, 0]
        self.button_states = [0, 0, 0, 0]
        self.max_steps = max_steps

    def get_command(self, t, probe=False):
        import random

        command = self.state_estimator.get_command()

        self.path = VelocityProfile(T=self.max_steps/50., t=t)

        command[0] = command[0] * self.x_scale
        command[1] = command[1] * self.y_scale
        command[2] = command[2] * self.yaw_scale

        print("x scale "+str(self.x_scale))
        print("y scale "+str(self.y_scale))
        print("yaw scale "+str(self.yaw_scale))
        print("vel: "+str(command[:4]))

        reset_timer = False

        if probe:
            command[0] = command[0] * self.probe_vel_multiplier
            command[2] = command[2] * self.probe_vel_multiplier

        # check for action buttons
        prev_button_states = self.button_states[:]
        self.button_states = self.state_estimator.get_buttons()
        for button in range(4):
            if self.triggered_commands[button] is not None:
                if self.button_states[button] == 1 and prev_button_states[button] == 0:
                    if not self.currently_triggered[button]:
                        # reset the triggered action
                        self.triggered_commands[button].reset(t)
                        # reset the internal timing variable
                        reset_timer = True
                        self.currently_triggered[button] = True
                    else:
                        self.currently_triggered[button] = False
                # execute the triggered action
                if self.currently_triggered[button] and t < self.triggered_commands[button].max_timestep:
                    command = self.triggered_commands[button].get_command(t)
        # commands x, y, yaw
        # if t <= 4:
        #     command[0] = 0.25 * t
        # if t > 4 and t <= 8:
        #     command[0] = 1
        # if t > 8:
        #     command[0] = 1 - 0.25 * (t - 8)

        ###########################################
        # if t%12 <= 6:
        #     command[0] = 1 - t%12/6
        # if t%12 > 6:
        #     command[0] = -(t%12 - 6)/6
        # command[0] = 0.5*command[0] # x
        # print(f"time {t}, command {command[0]}")
        ###########################################

        # GENERATE PATH
        # [x_vel_cmd, y_vel_cmd, yaw_vel_cmd] = self.path.generate_trapezoid(v_x=0.4, v_y=round(random.uniform(0.1, 0.5), 10), v_omega=0.0, acc_ratio=0.5)
        [x_vel_cmd, y_vel_cmd, yaw_vel_cmd] = self.path.generate_symmetric_ramp(v_x=0.5)
        # [x_vel_cmd, y_vel_cmd, yaw_vel_cmd] = self.path.generate_circle_omni(radius=round(random.uniform(0.5, 1), 10), clockwise=True)
        # [x_vel_cmd, y_vel_cmd, yaw_vel_cmd] = self.path.generate_circle_forward(radius=0.7, linear_speed=0.7, clockwise=False)
        # [x_vel_cmd, y_vel_cmd, yaw_vel_cmd] = self.path.generate_circle_forward(radius=round(random.uniform(0.4, 0.8), 10), linear_speed=round(random.uniform(0.5, 1.2), 10), clockwise=False)

        
        command[0] = x_vel_cmd
        command[1] = y_vel_cmd
        command[2] = yaw_vel_cmd
        command[3] = 0.25
        command[4] = 3.0
        # env.commands[5:8] = gait
        # env.commands[8] = 0.5
        # env.commands[9] = footswing_height_cmd
        command[10] = -0.1745
        # env.commands[11] = roll_cmd
        # env.commands[12] = stance_width_cmd

        ## command[1] = 0.0 # y
        ## command[2] = 0.0 # yaw
        ## command[3] = 0.1 # height
        ## command[4] = 3.0 # freq
        ## command[10] = 0.1745 # 10 deg UP front
        return command, reset_timer

    def add_triggered_command(self, button_idx, command_profile):
        self.triggered_commands[button_idx] = command_profile

    def get_buttons(self):
        return self.state_estimator.get_buttons()

class RCControllerProfileAccel(RCControllerProfile):
    def __init__(self, dt, state_estimator, x_scale=1.0, y_scale=1.0, yaw_scale=1.0):
        super().__init__(dt, state_estimator, x_scale=x_scale, y_scale=y_scale, yaw_scale=yaw_scale)
        self.x_scale, self.y_scale, self.yaw_scale = self.x_scale / 100., self.y_scale / 100., self.yaw_scale / 100.
        self.velocity_command = torch.zeros(3)

    def get_command(self, t):

        accel_command = self.state_estimator.get_command()
        self.velocity_command[0] = self.velocity_command[0]  + accel_command[0] * self.x_scale
        self.velocity_command[1] = self.velocity_command[1]  + accel_command[1] * self.y_scale
        self.velocity_command[2] = self.velocity_command[2]  + accel_command[2] * self.yaw_scale

        # check for action buttons
        prev_button_states = self.button_states[:]
        self.button_states = self.state_estimator.get_buttons()
        for button in range(4):
            if self.button_states[button] == 1 and self.triggered_commands[button] is not None:
                if prev_button_states[button] == 0:
                    # reset the triggered action
                    self.triggered_commands[button].reset(t)
                # execute the triggered action
                return self.triggered_commands[button].get_command(t)

        return self.velocity_command[:]

    def add_triggered_command(self, button_idx, command_profile):
        self.triggered_commands[button_idx] = command_profile

    def get_buttons(self):
        return self.state_estimator.get_buttons()

class KeyboardProfile(CommandProfile):
    # for control via keyboard inputs to isaac gym visualizer
    def __init__(self, dt, isaac_env, x_scale=1.0, y_scale=1.0, yaw_scale=1.0):
        super().__init__(dt)
        from isaacgym.gymapi import KeyboardInput
        self.gym = isaac_env.gym
        self.viewer = isaac_env.viewer
        self.x_scale = x_scale
        self.y_scale = y_scale
        self.yaw_scale = yaw_scale
        self.gym.subscribe_viewer_keyboard_event(self.viewer, KeyboardInput.KEY_UP, "FORWARD")
        self.gym.subscribe_viewer_keyboard_event(self.viewer, KeyboardInput.KEY_DOWN, "REVERSE")
        self.gym.subscribe_viewer_keyboard_event(self.viewer, KeyboardInput.KEY_LEFT, "LEFT")
        self.gym.subscribe_viewer_keyboard_event(self.viewer, KeyboardInput.KEY_RIGHT, "RIGHT")

        self.keyb_command = [0, 0, 0]
        self.command = [0, 0, 0]

    def get_command(self, t):
        events = self.gym.query_viewer_action_events(self.viewer)
        events_dict = {event.action: event.value for event in events}
        print(events_dict)
        if "FORWARD" in events_dict and events_dict["FORWARD"] == 1.0: self.keyb_command[0] = 1.0
        if "FORWARD" in events_dict and events_dict["FORWARD"] == 0.0: self.keyb_command[0] = 0.0
        if "REVERSE" in events_dict and events_dict["REVERSE"] == 1.0: self.keyb_command[0] = -1.0
        if "REVERSE" in events_dict and events_dict["REVERSE"] == 0.0: self.keyb_command[0] = 0.0
        if "LEFT" in events_dict and events_dict["LEFT"] == 1.0: self.keyb_command[1] = 1.0
        if "LEFT" in events_dict and events_dict["LEFT"] == 0.0: self.keyb_command[1] = 0.0
        if "RIGHT" in events_dict and events_dict["RIGHT"] == 1.0: self.keyb_command[1] = -1.0
        if "RIGHT" in events_dict and events_dict["RIGHT"] == 0.0: self.keyb_command[1] = 0.0

        self.command[0] = self.keyb_command[0] * self.x_scale
        self.command[1] = self.keyb_command[2] * self.y_scale
        self.command[2] = self.keyb_command[1] * self.yaw_scale

        print(self.command)

        return self.command


if __name__ == "__main__":
    cmdprof = ConstantAccelerationProfile(dt=0.2, max_speed=4, accel_time=3)
    print(cmdprof.commands)
    print(cmdprof.get_command(2))

