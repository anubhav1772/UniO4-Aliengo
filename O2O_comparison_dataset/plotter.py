import os
import h5py
import numpy as np
import matplotlib.pyplot as plt

class Plotter:
    def __init__(self):
        # Folder containing all .hdf5 files
        self.folder_path = "/home/anubhav1772/Github/UniO4-Aliengo/O2O_comparison_dataset/vx_2_vy_0_w_0"

        # Define feature slices (based on your table)
        self.feature_slices = {
            "gravity_vector": slice(0, 3),
            "x_vel": slice(3, 4),
            "y_vel": slice(4, 5),
            "yaw_vel": slice(5, 6),
            "body_height": slice(6, 7),
            "step_freq": slice(7, 8),
            "gait": slice(8, 11),
            "durations": slice(11, 12),
            "footswing_height": slice(12, 13),
            "body_pitch": slice(13, 14),
            "body_roll": slice(14, 15),
            "stance_width": slice(15, 16),
            "stance_length": slice(16, 17),
            "aux_reward": slice(17, 18),
            "dof_pos": slice(18, 30),
            "dof_vel": slice(30, 42),
            "actions": slice(42, 54),
            "clock_inputs": slice(54, 58),
    
            # Camera-based features
            "cam_translation_z": slice(58, 59),
            "cam_translation_x": slice(59, 60),
            "cam_translation_y": slice(60, 61),
            "cam_velocity_z": slice(61, 62),        # V_x (-cam_velocity_z)
            "cam_velocity_x": slice(62, 63),        # V_y (-cam_velocity_x)
            "cam_velocity_y": slice(63, 64),        # V_z (cam_velocity_y)
            "cam_acceleration_z": slice(64, 65),
            "cam_acceleration_x": slice(65, 66),
            "cam_acceleration_y": slice(66, 67),
            "cam_roll": slice(67, 68),
            "cam_pitch": slice(68, 69),
            "cam_yaw": slice(69, 70),
            "cam_angular_velocity_z": slice(70, 71),
            "cam_angular_velocity_x": slice(71, 72),
            "cam_angular_velocity_y": slice(72, 73),
            "cam_angular_acceleration_z": slice(73, 74),
            "cam_angular_acceleration_x": slice(74, 75),
            "cam_angular_acceleration_y": slice(75, 76),
        }

    def process_data(self, observations):
        # print(f"Processing...")

        x0 = observations[0, 58]
        y0 = observations[0, 59]
        z0 = observations[0, 60]
        r0 = observations[0, 67]
        p0 = observations[0, 68]
        theta0 = observations[0, 69]

        N = len(observations)
        L = 0.16

        observations[:, 58] -= x0 * np.ones(N)
        observations[:, 59] -= y0 * np.ones(N)
        observations[:, 60] -= z0 * np.ones(N)
        observations[:, 67] -= r0 * np.ones(N)
        observations[:, 68] -= p0 * np.ones(N)
        observations[:, 69] -= theta0 * np.ones(N)

        for i in range(len(observations)):
            C = np.cos(observations[i, 69] + theta0)
            S = np.sin(observations[i, 69] + theta0)

            observations[i, 58] = observations[i, 58] - L * np.cos(theta0) + L
            observations[i, 59] = observations[i, 59] - L * np.sin(theta0)

            observations[i, 61] += L * S
            observations[i, 62] -= L * C

            R = np.array([[C, S, 0],
                          [-S, C, 0],
                          [0, 0, 1]])
            
            v_ref = np.array([[observations[i, 61]],
                              [observations[i, 62]],
                              [0]])
            v = np.dot(R, v_ref)
            observations[i, 61] = v[0]
            observations[i, 62] = v[1]

        return observations

    def get_dataset_states(self, mode="offline"):
        """Load and process states for a given mode (offline/online)."""
        all_states = []
        for filename in os.listdir(os.path.join(self.folder_path, mode)):
            if filename.endswith(".hdf5"):
                with h5py.File(os.path.join(self.folder_path, mode, filename), "r") as f:
                    if "states" in f:
                        states = np.array(f["states"])
                        states = self.process_data(states)
                        all_states.append(states)
        if len(all_states) == 0:
            raise ValueError(f"No valid HDF5 files with 'states' in mode '{mode}' found.")
        return np.array(all_states)


    def plot_features(self, save_folder=None, selected_feature=None):
        """
        Plot a selected feature and x_vel from multiple episodes.
        Only plot timesteps up to 5 seconds.
        """
        all_states = self.get_dataset_states()  # List of arrays: one per episode

        if selected_feature is None:
            raise ValueError("You must specify a selected_feature to plot.")

        if selected_feature not in self.feature_slices:
            raise KeyError(f"Feature '{selected_feature}' not found in feature_slices.")

        feat_slice = self.feature_slices[selected_feature]
        x_vel_slice = self.feature_slices["x_vel"]

        # Create full plot
        plt.figure(figsize=(12, 6))

        for ep_idx, ep_states in enumerate(all_states):
            values = ep_states[:, feat_slice]
            x_vel = ep_states[:, x_vel_slice]
            timesteps = np.arange(values.shape[0]) / 50.0  # in seconds

            # Only use values up to 5 seconds
            mask = timesteps <= 5.0
            timesteps = timesteps[mask]
            values = values[mask]
            x_vel = x_vel[mask]

            # Plot selected feature
            if values.shape[1] == 1:
                plt.plot(timesteps, values, label=f"{selected_feature} Ep {ep_idx + 1}")
            else:
                for j in range(values.shape[1]):
                    plt.plot(timesteps, values[:, j], label=f"Ep{ep_idx + 1}_{selected_feature}_{j+1}", alpha=0.7)

            label = "Commanded x_vel" if ep_idx == 0 else None
            plt.plot(timesteps, x_vel/2.0, linestyle='--', alpha=0.6, label=label)

        plt.title("Forward Linear Velocity")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

        if save_folder:
            os.makedirs(save_folder, exist_ok=True)
            save_path = os.path.join(save_folder, f"{selected_feature}_xvel_multi_episode.png")
            plt.savefig(save_path)
            print(f"Saved plot to {save_path}")
        else:
            plt.show()

    def plot(self, selected_feature=None):
        all_states = self.get_dataset_states()  # List of episodes

        if selected_feature is None:
            raise ValueError("You must specify a selected_feature to plot.")

        if selected_feature not in self.feature_slices:
            raise KeyError(f"Feature '{selected_feature}' not found in feature_slices.")

        feat_slice = self.feature_slices[selected_feature]
        x_vel_slice = self.feature_slices["x_vel"]  # This is the commanded velocity

        feature_all = []
        x_vel_ref = None  # To store commanded x_vel from the first episode

        for ep in all_states:
            timesteps = np.arange(ep.shape[0]) / 50.0
            mask = timesteps <= 5.0
            timesteps = timesteps[mask]

            feature = ep[mask, feat_slice]
            if feature.shape[1] == 1:
                feature = feature[:, 0]
            feature_all.append(feature)

            # Save commanded x_vel once (same across episodes)
            if x_vel_ref is None:
                x_vel = ep[mask, x_vel_slice]
                if x_vel.shape[1] == 1:
                    x_vel = x_vel[:, 0]
                x_vel_ref = x_vel

        # Match lengths
        min_len = min(len(f) for f in feature_all)
        feature_all = [f[:min_len] for f in feature_all]
        feature_array = np.stack(feature_all, axis=0)
        timesteps = np.arange(min_len) / 50.0
        x_vel_ref = x_vel_ref[:min_len]

        # Mean and std
        mean_feat = feature_array.mean(axis=0)
        std_feat = feature_array.std(axis=0)

        # Plot
        plt.figure(figsize=(10, 5))
        plt.plot(timesteps, mean_feat, label=f"Mean {selected_feature}", color="blue")
        plt.fill_between(timesteps, mean_feat - std_feat, mean_feat + std_feat,
                         color="blue", alpha=0.3, label="Std Dev")

        # Add commanded x_vel as dashed black line
        plt.plot(timesteps, x_vel_ref/2.0, linestyle="--", color="black", label="Commanded x_vel")

        plt.title("Forward Linear Velocity")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    def plot_O2O(self, selected_feature=None):
        if selected_feature is None:
            raise ValueError("You must specify a selected_feature to plot.")
        if selected_feature not in self.feature_slices:
            raise KeyError(f"Feature '{selected_feature}' not found in feature_slices.")

        feat_slice = self.feature_slices[selected_feature]
        x_vel_slice = self.feature_slices["x_vel"]

        modes = {
            "offline": {"color": "blue"},
            "online": {"color": "green"},
        }

        plt.figure(figsize=(10, 5))
        x_vel_ref = None  # To store commanded x_vel once

        for mode_name, style in modes.items():
            all_states = self.get_dataset_states(mode=mode_name)
            feature_all = []

            for ep in all_states:
                timesteps = np.arange(ep.shape[0]) / 50.0
                mask = timesteps <= 5.0
                timesteps = timesteps[mask]

                feature = ep[mask, feat_slice]
                if feature.shape[1] == 1:
                    feature = feature[:, 0]
                feature_all.append(feature)

                # Save commanded x_vel from first episode
                if x_vel_ref is None:
                    x_vel = ep[mask, x_vel_slice]
                    if x_vel.shape[1] == 1:
                        x_vel = x_vel[:, 0]
                    x_vel_ref = x_vel[:len(timesteps)]

            # Trim and stack
            min_len = min(len(f) for f in feature_all)
            feature_all = [f[:min_len] for f in feature_all]
            feature_array = np.stack(feature_all, axis=0)
            timesteps = np.arange(min_len) / 50.0

            mean_feat = feature_array.mean(axis=0)
            std_feat = feature_array.std(axis=0)

            # plt.plot(timesteps, mean_feat, label=f"{mode_name} mean {selected_feature}", color=style["color"])
            plt.plot(timesteps, mean_feat, label=f"{mode_name} mean x vel", color=style["color"])
            plt.fill_between(timesteps, mean_feat - std_feat, mean_feat + std_feat,
                             color=style["color"], alpha=0.3, label=f"{mode_name} std band")

        # Commanded x_vel (same for both)
        plt.plot(timesteps, x_vel_ref / 2.0, linestyle="--", color="black", label="Commanded x vel")

        plt.title(f"Forward Velocity Tracking: Offline vs Online ({np.mean(x_vel_ref / 2.0)} m/s)")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    plotter = Plotter()
    # plotter.plot_features(save_folder="mbrl_dynamics_net/plots")
    plotter.plot_O2O(selected_feature="cam_velocity_z") 
