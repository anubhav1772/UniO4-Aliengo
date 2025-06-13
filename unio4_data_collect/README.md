### List of Observations / Commands

| No. | Feature                | Dimension | Description                              |
|-----|------------------------|-----------|------------------------------------------|
| 1   | `gravity_vector`       | 3         | Gravity direction vector                 |
| 2   | `x_vel`                | 1         | Commanded x-axis velocity                |
| 3   | `y_vel`                | 1         | Commanded y-axis velocity                |
| 4   | `yaw_vel`              | 1         | Commanded yaw rate                       |
| 5   | `body_height`          | 1         | Commanded body height                    |
| 6   | `step_freq`            | 1         | Gait frequency (Hz)                      |
| 7   | `gait`                 | 3         | Gait timing: [phase, offset, bounds]     |
| 8   | `durations`            | 1         | Gait duration                            |
| 9   | `footswing_height`     | 1         | Desired foot swing height                |
| 10  | `body_pitch`           | 1         | Commanded pitch of the body              |
| 11  | `body_roll`            | 1         | Commanded roll of the body               |
| 12  | `stance_width`         | 1         | Distance between left and right legs     |
| 13  | `stance_length`        | 1         | Distance between front and back legs     |
| 14  | `aux_reward`           | 1         | Optional auxiliary reward command        |
| 15  | `dof_pos`              | 12        | Joint positions                          |
| 16  | `dof_vel`              | 12        | Joint velocities                         |
| 17  | `actions`              | 12        | Previous actions taken                   |
| 18  | `clock_inputs`         | 4         | Sinusoidal clock encoding per leg        |

---

**Total Observation Dimension**: **58**  
- Commands & States: 18 values  
- DOF-related: 36 values  
- Clock Inputs: 4 values  

