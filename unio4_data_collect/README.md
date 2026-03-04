### List of Observations / Commands

<table>
  <tr>
    <td><img src="https://drive.google.com/uc?export=view&id=1Tp0pox7x3wJ6NP0vMtJD5i3ac7CSEqH3" width=2000 height=350></td>
  </tr>
 </table>

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


**Total Observation Dimension**: **58**  
- Commands & States: 18 values  
- DOF-related: 36 values  
- Clock Inputs: 4 values

---

**Camera Observations:**

| **No.**   | **Pose Feature**                     | **Description (Robot Context)**                                             |
|-----------|--------------------------------------|-----------------------------------------------------------------------------|
| 1         | `-translation.z`                     | Robot position Z (converted from camera frame to world/robot frame)         |
| 2         | `-translation.x`                     | Robot position X                                                            |
| 3         | `translation.y`                      | Robot position Y                                                            |
| 4         | `-velocity.z`                        | Linear velocity Z                                                           |
| 5         | `-velocity.x`                        | Linear velocity X                                                           |
| 6         | `velocity.y`                         | Linear velocity Y                                                           |
| 7         | `-acceleration.z`                    | Linear acceleration Z                                                       |
| 8         | `-acceleration.x`                    | Linear acceleration X                                                       |
| 9         | `acceleration.y`                     | Linear acceleration Y                                                       |
| 10        | `roll`                               | Roll angle (from quaternion orientation)                                    |
| 11        | `pitch`                              | Pitch angle                                                                 |
| 12        | `yaw`                                | Yaw angle                                                                   |
| 13        | `-angular_velocity.z`                | Angular velocity around Z axis                                              |
| 14        | `-angular_velocity.x`                | Angular velocity around X axis                                              |
| 15        | `angular_velocity.y`                 | Angular velocity around Y axis                                              |
| 16        | `-angular_acceleration.z`            | Angular acceleration around Z axis                                          |
| 17        | `-angular_acceleration.x`            | Angular acceleration around X axis                                          |
| 18        | `angular_acceleration.y`             | Angular acceleration around Y axis                                          |

<table>
  <tr>
    <td><img src="https://drive.google.com/uc?export=view&id=1mW0_UBgTUpJ_7JWroGgx4d8NFvUUpVQH" width=500 height=650></td>
  </tr>
 </table>






