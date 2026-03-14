Simulation

MATLAB and Simulink implementations of the Adaptive Sliding Mode Controller. All simulation results presented in the project poster and report were generated from these files.

## Files

| File | Description |
|------|-------------|
| `ASMC_waypoints.m` | Main ASMC simulation — attitude regulation with waypoints, fault injection, and performance plots |
| `ASMC_tracking.m` | MATLAB Function block implementation for Simulink integration |
| `cubesatVisualisation.m` | Render and plots of ASMC simulations, requiring the controller simulation and mesh files |
| `meshes` | Mesh files used for 3D simulation renders |

## Running the simulation

<p align="center">
  <img src="Resources/SimulationPlot.gif"/>
  <br/>
  <em>ASMC simulation — attitude tracking (180 degree)</em>
</p>

1. Open MATLAB (R2021a or later)
2. Run `ASMC_waypoints.m` 
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

## Simulation Viewer

`cubesatVisualisation.m` is designed to render a view of the ASMC simulation:

<p align="center">
  <img src="Resources/SimulationRender.gif"/>
  <br/>
  <em>ASMC simulation in viewer</em>
</p>

1. Run `ASMC_waypoints.m` 
2. Parameters are stored locally from ASMC simulation
3. Ensure 'meshes' folder is present to load stl files
4. Run `cubesatVisualisation.m` from the same file

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


```matlab
load_and_plot_log('log001.csv')
```

Plots use the same axes and colour scheme as `ASMC_waypoints.m` for direct comparison.
