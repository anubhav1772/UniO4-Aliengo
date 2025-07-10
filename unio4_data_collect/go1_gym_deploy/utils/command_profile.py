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
    
    # def get_command(self, t, probe=False):
    #     command = self.state_estimator.get_command()
    #     print('$'*30)
    #     print(command)
    #     print('*'*30)
    #     command[0] = command[0] * self.x_scale
    #     command[1] = command[1] * self.y_scale
    #     command[2] = command[2] * self.yaw_scale
    #     # command[9] = 
    #     # command
    #     # command[12] = 
    #     reset_timer = False
    #     se = self.state_estimator
    #     # --- Handle new buttons for velocity control ---
    #     # Button to cycle linear velocity X (-1 ... 1)
    #     if hasattr(se, 'button_lin_x_pressed') and se.button_lin_x_pressed and not self._last_lin_x:
    #         self.current_lin_x += 0.25
    #         if self.current_lin_x > 1.0:
    #             self.current_lin_x = -1.0
    #         print(f'[Speed] Linear velocity X: {self.current_lin_x}')
    #     # Button to cycle linear velocity Y (-1 ... 1)
    #     if hasattr(se, 'button_lin_y_pressed') and se.button_lin_y_pressed and not self._last_lin_y:
    #         self.current_lin_y += 0.25
    #         if self.current_lin_y > 1.0:
    #             self.current_lin_y = -1.0
    #         print(f'[Speed] Linear velocity Y: {self.current_lin_y}')
    #     # Button to cycle yaw velocity (-1 ... 1)
    #     if hasattr(se, 'button_yaw_pressed') and se.button_yaw_pressed and not self._last_yaw:
    #         self.current_yaw += 0.25
    #         if self.current_yaw > 1.0:
    #             self.current_yaw = -1.0
    #         print(f'[Speed] Yaw velocity: {self.current_yaw}')
    #     # Update edge detection for new buttons
    #     self._last_lin_x = getattr(se, 'button_lin_x_pressed', False)
    #     self._last_lin_y = getattr(se, 'button_lin_y_pressed', False)
    #     self._last_yaw = getattr(se, 'button_yaw_pressed', False)
    #     # --- Apply selected velocities to the command ---
    #     # command[0] = self.current_lin_x
    #     # command[1] = self.current_lin_y
    #     # command[2] = self.current_yaw
    #     command[0] = 0
    #     command[1] = 0
    #     command[2] = 0
    #     # --- Remaining logic (gait, duration, gait start) ---
    #     # Gait buttons (single press)
    #     if hasattr(se, 'button_A_pressed') and se.button_A_pressed and not self._last_A:
    #         print("*************** BUTTON A PRESSED *****************")
    #         self.gait_mode = 0  # trot
    #     if hasattr(se, 'button_B_pressed') and se.button_B_pressed and not self._last_B:
    #         print("*************** BUTTON B PRESSED *****************")
    #         self.gait_mode = 1  # pace
    #     if hasattr(se, 'button_X_pressed') and se.button_X_pressed and not self._last_X:
    #         print("*************** BUTTON X PRESSED *****************")
    #         self.gait_mode = 2  # bound
    #     if hasattr(se, 'button_Y_pressed') and se.button_Y_pressed and not self._last_Y:
    #         print("*************** BUTTON Y PRESSED *****************")
    #         self.gait_mode = 3  # gallop
    #     # Duration buttons
    #     if hasattr(se, 'dpad_up_pressed') and se.dpad_up_pressed and not self._last_dpad_up:
    #         print("*************** DPAD UP PRESSED *****************")
    #         self.gait_duration = min(self.gait_duration + 1, 10)
    #     if hasattr(se, 'dpad_down_pressed') and se.dpad_down_pressed and not self._last_dpad_down:
    #         print("*************** DPAD DOWN PRESSED *****************")
    #         self.gait_duration = max(self.gait_duration - 1, 3)
    #     # Gait start button
    #     if hasattr(se, 'R1_pressed') and se.R1_pressed and not self._last_R1 and not self.gait_active:
    #         print("*************** R1 PRESSED *****************")
    #         self.gait_active = True
    #         self.gait_start_time = time.time()
    #     # Update button states (edge detection)
    #     self._last_A = getattr(se, 'button_A_pressed', False)
    #     self._last_B = getattr(se, 'button_B_pressed', False)
    #     self._last_X = getattr(se, 'button_X_pressed', False)
    #     self._last_Y = getattr(se, 'button_Y_pressed', False)
    #     self._last_dpad_up = getattr(se, 'dpad_up_pressed', False)
    #     self._last_dpad_down = getattr(se, 'dpad_down_pressed', False)
    #     self._last_R1 = getattr(se, 'R1_pressed', False)
    #     # --- Gait control ---
    #     if not self.gait_active:
    #         command[5:8] = 0  # pronking
    #     else:
    #         # gait parameters by self.gait_mode
    #         if self.gait_mode == 0:  # trot
    #             print("*************** TROTTING *****************")
    #             command[5] = 0.5  # phase
    #             command[6] = 0.0  # offset
    #             command[7] = 0.0  # bound
    #         elif self.gait_mode == 1:  # pace
    #             print("*************** PACING *****************")
    #             command[5] = 0.0
    #             command[6] = 0.0
    #             command[7] = 0.5
    #         elif self.gait_mode == 2:  # bound
    #             print("*************** BOUND***************")
    #             command[5] = 0.0
    #             command[6] = 0.5
    #             command[7] = 0.0
    #         elif self.gait_mode == 3:  # gallop
    #             print("*************** GALLOPING *****************")
    #             command[5] = 0.25
    #             command[6] = 0.0
    #             command[7] = 0.0
    #         command[8] = 0.5 # duration
    #         # Check if gait duration is over
    #         if self.gait_start_time is not None and (time.time() - self.gait_start_time > self.gait_duration):
    #             self.gait_active = False
    #     return command, reset_timer

    def get_command(self, t, probe=False):

        command = self.state_estimator.get_command()

        self.path = VelocityProfile(T=self.max_steps/50., t=t)

        command[0] = command[0] * self.x_scale
        command[1] = command[1] * self.y_scale
        command[2] = command[2] * self.yaw_scale

        print("x scale "+str(self.x_scale))
        print("y scale "+str(self.y_scale))
        print("yaw scale "+str(self.yaw_scale))
        print("vel: "+str(command[1:4]))

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
        [x_vel_cmd, y_vel_cmd, yaw_vel_cmd] = self.path.generate_trapezoid(v_x=0.0, v_y=0.0, v_omega=1.0, acc_ratio=0.5)
        # [x_vel_cmd, y_vel_cmd, yaw_vel_cmd] = self.path.generate_symmetric_ramp(v_x=1.5)
        # [x_vel_cmd, y_vel_cmd, yaw_vel_cmd] = self.path.generate_circle_omni(radius=1.0, clockwise=False)
        # [x_vel_cmd, y_vel_cmd, yaw_vel_cmd] = self.path.generate_circle_forward(radius=1.2, linear_speed=0.8, clockwise=True)
    
        command[0] = x_vel_cmd
        command[1] = y_vel_cmd
        command[2] = yaw_vel_cmd
        command[3] = 0.25
        command[4] = 3.0
        # env.commands[5:8] = gait
        # env.commands[8] = 0.5
        # env.commands[9] = footswing_height_cmd
        command[10] = 0.1745
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
