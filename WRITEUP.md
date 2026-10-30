# Control and Trajectory Tracking for an Autonomous Vehicle

## 1. Project Overview

This project implements PID-based lateral and longitudinal control for a vehicle in the CARLA simulator. The controller receives a planned trajectory, the vehicle pose, and the measured speed through a WebSocket connection. It then computes steering, throttle, and brake commands while the motion planner generates collision-free candidate spirals around three parked vehicles.

The final implementation was validated by driving past all three parked vehicles without collision. The final configuration uses:

| Controller | $K_p$ | $K_i$ | $K_d$ | Output limits |
|---|---:|---:|---:|---:|
| Steering | 0.45 | 0.00 | 0.00 | $[-0.20, 0.20]$ |
| Throttle/brake | 0.40 | 0.02 | 0.02 | $[-1.00, 1.00]$ |

Additional planning and tracking parameters are:

- Speed limit: $2.5\,\mathrm{m/s}$
- Maximum acceleration: $1.5\,\mathrm{m/s^2}$
- Steering trajectory target: six points after the nearest trajectory point
- Steering error limit: $[-0.45, 0.45]$ radians
- Trajectory update threshold: 16 remaining points
- Throttle target: mean velocity of the first three trajectory points

## 2. PID Controller Implementation

The PID controller is implemented with the standard control law

$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \frac{de(t)}{dt}.$$

At every update, the controller:

1. Stores the current proportional error.
2. Accumulates the integral error using the elapsed time.
3. Computes the derivative from the difference between the current and previous errors.
4. Combines all three terms.
5. Clamps the command to the configured output limits.

Output limiting is important because CARLA expects normalized actuator commands and because unlimited steering corrections produced unstable oscillation during tuning.

## 3. Steering Control

### 3.1 Target-point selection

Using a fixed trajectory index was not reliable because the physical vehicle and the virtual trajectory did not always advance at exactly the same rate. The final implementation first finds the trajectory point nearest to the physical CARLA vehicle:

$$i_{\text{nearest}} = \arg\min_i \left[(x_i-x)^2+(y_i-y)^2\right].$$

The steering target is then selected six points ahead:

$$i_{\text{target}} = \min(i_{\text{nearest}}+6,\;N-1).$$

This provides sufficient preview for smooth obstacle avoidance while still reacting to the local path geometry.

### 3.2 Vehicle-coordinate error

The vector from the vehicle to the target is

$$
\Delta x=x_{\text{target}}-x, \qquad
\Delta y=y_{\text{target}}-y.
$$

It is transformed into the vehicle coordinate frame using the vehicle yaw $\psi$:

$$
\begin{aligned}
e_{\text{forward}} &= \cos(\psi)\Delta x+\sin(\psi)\Delta y,\\
e_{\text{lateral}} &= -\sin(\psi)\Delta x+\cos(\psi)\Delta y.
\end{aligned}
$$

If the target is ahead of the vehicle, the steering-angle error is

$$e_{\text{steer}}=\operatorname{atan2}(e_{\text{lateral}},L),$$

where the effective look-ahead distance is bounded between 5 m and 12 m. The resulting error is limited to $\pm0.45$ rad. If the selected point is behind the vehicle, the steering error is set to zero to avoid commanding a turn toward a stale point.

### 3.3 Steering tuning

The final steering gains are

$$K_p=0.45,\qquad K_i=0,
\qquad K_d=0.$$

A proportional controller was sufficient after improving the geometric steering error. Integral action was intentionally omitted because persistent integration can cause windup when the path changes during obstacle avoidance. Derivative action was also omitted because trajectory updates can introduce discrete changes in the target angle, which makes the derivative term noisy.

The steering command is limited to $\pm0.20$. Lower proportional gains did not turn early enough around obstacles. Higher gains or larger command limits caused excessive left-right motion and roadside excursions.

## 4. Longitudinal Control

The desired velocity is calculated from the mean of the first three planned velocity samples:

$$v_{\text{target}}=\frac{1}{M}\sum_{i=0}^{M-1}v_i,\qquad M=\min(3,N).$$

The throttle error is

$$e_v=v_{\text{target}}-v_{\text{measured}}.$$

The final longitudinal gains are

$$K_p=0.40,\qquad K_i=0.02,
\qquad K_d=0.02.$$

A positive PID output is applied as throttle. A negative output is converted to a positive brake command:

$$
\begin{cases}
\text{throttle}=u,\;\text{brake}=0, & u>0,\\
\text{throttle}=0,\;\text{brake}=-u, & u\leq0.
\end{cases}
$$

Averaging the first three velocity points reduced sensitivity to an individual stale or discontinuous speed sample while preserving adequate response to planned speed changes.

## 5. Effect of the PID Terms

### Proportional term

The proportional term reacts directly to the current error. Increasing $K_p$ produces a faster and stronger response. If it is too small, the vehicle turns too late or accelerates too slowly. If it is too large, the vehicle overshoots and oscillates.

For steering, the final controller is proportional-only. Therefore, the steering-command curve follows the steering-error curve with a scale factor of 0.45 until the output reaches its $\pm0.20$ limit.

### Integral term

The integral term accumulates error over time and removes steady-state offset. It is useful for longitudinal control because drag and rolling resistance can leave a small persistent speed error. Too much integral gain causes overshoot and windup, particularly when the target speed changes or the command saturates.

### Derivative term

The derivative term reacts to the rate of change of the error. It adds damping and can reduce overshoot. However, it amplifies noise and discontinuities. A small derivative gain is used for longitudinal control, while steering uses no derivative term because the selected trajectory target can change discretely.

## 6. Plot Interpretation

The controller writes two log files during a run:

- `steer_pid_data.txt`: iteration, steering error, steering output, nearest-path distance, selected indices, yaw, and transformed target errors.
- `throttle_pid_data.txt`: iteration, speed error, brake output, and throttle output.

The steering plot should show the command changing sign as the vehicle enters and exits each avoidance maneuver. Because the controller is proportional-only, steering output should track steering error without integral lag. Flat sections at $\pm0.20$ indicate actuator saturation; brief saturation can be acceptable, but prolonged saturation usually indicates that the vehicle and trajectory have diverged.

The longitudinal plot should show a large positive speed error during initial acceleration, followed by a reduction as measured speed approaches the planned speed. Short brake pulses may appear when the actual speed exceeds the planned profile. Stable operation is indicated by bounded speed error and by the absence of rapid throttle/brake switching.

Plots can be generated from the recorded files with:

```bash
cd project/pid_controller
python3 ../plot_pid.py
```

## 7. Tuning Process

The parameters were tuned manually using repeated CARLA runs and the recorded PID logs:

1. Begin with proportional-only steering at a low gain.
2. Increase steering $K_p$ until the vehicle can follow the planned avoidance path.
3. Add a nearest-point search so steering remains referenced to the physical vehicle.
4. Add a six-point preview to turn early enough around obstacles.
5. Bound the angular error and actuator output to prevent unstable commands.
6. Tune throttle $K_p$ for adequate acceleration.
7. Add small $K_i$ and $K_d$ terms to reduce steady-state speed error and damp speed changes.
8. Reduce the planning speed limit to $2.5\,\mathrm{m/s}$ to improve tracking margin.
9. Validate the complete controller against all three parked vehicles.

A major lesson was to separate controller tuning from planner and state-estimation problems. A steering command that remains saturated is not necessarily a PID tuning problem; it can also mean that the trajectory, position, and yaw are inconsistent.

## 8. Automatic PID Tuning

An automatic tuner could minimize a weighted objective over repeated deterministic simulations:

$$J=w_1\sum_t e_{\text{lateral}}^2+w_2\sum_t e_v^2+w_3\sum_t \Delta u_t^2+w_4N_{\text{collision}}+w_5N_{\text{lane invasion}}.$$

A practical procedure would be:

1. Fix the CARLA map, spawn point, obstacles, random seed, and simulation duration.
2. Define safe ranges for $K_p$, $K_i$, and $K_d$.
3. Run each candidate parameter set from an identical initial state.
4. Reject candidates that collide, leave the road, or remain saturated for too long.
5. Optimize the remaining candidates using Bayesian optimization, coordinate descent, or the Twiddle algorithm.
6. Validate the best gains over different frame rates, initial offsets, and vehicle speeds.

Bayesian optimization would be appropriate because simulator evaluations are expensive and the objective is not analytically differentiable.

## 9. Model-Free Controller: Advantages and Disadvantages

### Advantages

- It does not require an accurate vehicle dynamics model.
- It is simple to implement and computationally inexpensive.
- Gains can be tuned directly from measured behavior.
- The same controller structure can be reused with different vehicles.
- It is robust to moderate modeling uncertainty.

### Disadvantages

- Gains may not generalize across speeds, frame rates, road friction, or vehicles.
- The controller does not explicitly account for steering dynamics, tire forces, actuator delay, or vehicle constraints.
- Saturation and integral windup require additional handling.
- PID reacts to error rather than predicting future path curvature.
- Discontinuous trajectories can cause abrupt commands.
- Separate lateral and longitudinal loops do not capture their dynamic coupling.

## 10. Possible Improvements

The controller could be improved by:

- Using a monotonic nearest-point search instead of searching the full trajectory on every cycle.
- Selecting the look-ahead distance as a continuous function of vehicle speed.
- Adding steering-rate limiting for smoother actuator commands.
- Adding integral anti-windup for throttle and brake.
- Using a high-resolution monotonic clock or simulator timestamps for PID timing.
- Validating and truncating stale trajectory entries when a new path is received.
- Handling motion-planner failure with a safe fallback trajectory.
- Replacing the lateral PID with a Stanley, Pure Pursuit, LQR, or Model Predictive Controller.
- Running CARLA in synchronous mode with a fixed simulation step for reproducible tests.

## 11. Final Result

The final controller successfully followed the generated trajectory and passed all three parked vehicles without collision in the validated test run. The most important changes were robust nearest-point selection, a six-point steering preview, bounded steering commands, smoothed velocity targeting, and conservative planning speed.
