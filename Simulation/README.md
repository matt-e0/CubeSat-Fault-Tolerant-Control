Simulation

MATLAB and Simulink implementations of the Adaptive Sliding Mode Controller. All simulation results presented in the project poster and report were generated from these files.

## Files

| File | Description |
|------|-------------|
| `ASMC_waypoints.m` | Main ASMC simulation — attitude regulation with waypoints, fault injection, and performance plots |
| `ASMC_tracking.m` | MATLAB Function block implementation for Simulink integration |
| `asmc_simulink.m` | Drop-in MATLAB Function block (3 inputs: `q`, `q_d`, `omega` → `tau_body`) |
| `load_and_plot_log.m` | Loads SD card CSV logs from hardware and plots against simulation |

## Running the simulation

1. Open MATLAB (R2021a or later)
2. Run `ASMC_waypoints.m` directly — no toolboxes required beyond base MATLAB
3. Adjust parameters at the top of the file:

```matlab
fault_time  = 20;    % time of wheel failure (seconds)
fault_wheel = 3;     % which wheel fails (1–4)
fault_level = 0.0;   % effectiveness after fault (0 = dead, 1 = healthy)
```

## Simulation results

The simulation runs for 80 seconds, injecting wheel failures mid-run and logging:

- Attitude error (degrees)
- Angular velocity (rad/s)
- Body frame torque (N·m)
- Reaction wheel torques
- Sliding surface norm
- Adaptive gain `K_adapt`
- Quaternion components

## Simulink integration

`asmc_simulink.m` is designed as a drop-in MATLAB Function block:

| Port | Name | Size | Description |
|------|------|------|-------------|
| In1 | `q` | 4×1 | Current quaternion (scalar-first) |
| In2 | `q_d` | 4×1 | Desired quaternion |
| In3 | `omega` | 3×1 | Angular velocity (rad/s) |
| Out1 | `tau_body` | 3×1 | Body torque demand (N·m) |

Set `q_d` using a Constant block. Common values:

```matlab
[1; 0; 0; 0]              % Identity — no rotation
[cos(pi/4); sin(pi/4); 0; 0]  % 90° about X
[cos(pi/4); 0; sin(pi/4); 0]  % 90° about Y
[cos(pi/4); 0; 0; sin(pi/4)]  % 90° about Z
```

## Key parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `lambda` | 1.5 | Sliding surface slope |
| `Kp` | 0.04 | Attitude error gain |
| `Kd` | 0.08 | Angular velocity damping |
| `phi` | 0.05 | Boundary layer thickness |
| `Gamma` | 2.0 | Adaptive gain rate |
| `K_min` | 0.01 | Minimum switching gain |
| `K_max` | 2.0 | Maximum switching gain |

## Comparing hardware logs to simulation

After running the hardware, copy the SD card CSV (`log###.csv`) to this folder and run:

```matlab
load_and_plot_log('log001.csv')
```

Plots use the same axes and colour scheme as `ASMC_waypoints.m` for direct comparison.
