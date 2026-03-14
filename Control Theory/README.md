# Adaptive Sliding Mode Controller Design

The at heart of the system is the Adaptive Sliding Mode Controller (ASMC). ASMC was chosen over traditional control methods due the requirements of fault-tolerance for the attitude control system.

| Parameter | Description |
|------------|-------------|
| `s` | Sliding surface |
| `lambda` | Sliding surface slope |
| `q` | Quaternion angle |
| `omega` | Rotational velocity |
| `J` | Inertia matrix |
| `tau` | Torque |
| `sigma` | Leakage coefficient |
| `Gamma` | Adaptive gain rate |
| `K/ K_adaptive` | Switching gain |
| `K_min` | Minimum switching gain |
| `K_max` | Maximum switching gain |

## Passive VS Active Fault Isolation

| Passive | Active |
|-----------|------|
| Inherently robust, doesn't require detection layer | More efficient distribution of torque, improved wheel health |
| Reduces processing load of microcontroller (no reaction wheel speed sensing or heavy matrix updates) | Can achieve a higher pointing accuracy during failures as feedback prevents overcompensation |
| Independent of fault-detection architecture, no risk for false-positives | System doesn't try to power faulty motors |

A passive fault-isolating control method was chosen as the project is aimed to integrate easily into existing satellite designs. The addition of a fault-detection and isolation program and the requirement for actuator feedback increases the complexity of the design and the load on the control electronics. The most promising passive controller is the ASMC, due to its high robustness and it's ability to linearise non-linear kinematics by defining a sliding surface.

<p align="center">
  <img src="/Resources/ControlTheory1.png" width="500"/>
  <br/>
  <em>ASMC Design - Page 1</em>
</p>

<p align="center">
  <img src="/Resources/ControlTheory2.png" width="500"/>
  <br/>
  <em>ASMC Design - Page 2</em>
</p>

## Lyapunov Proof

In order to verify that the sliding mode controller is stable, the energy of the system is analysed using a Lyapunov Proof, in this case the Quadratic Function is chosen as the system dynamics aligns with the equation for rotational kinetic energy. By proving that the system's energy will decay, the controller is shown to be stable.

<p align="center">
  <img src="/Resources/ControlTheory3.png" width="500"/>
  <br/>
  <em>ASMC Design - Page 3</em>
</p>

## Adaptive Gain Term

To overcome the limitations of a manually set gain, the gain of the sliding mode controller can be defined using an adaptive term, that increases at a rate tied to the norm 's' of the sliding surface (how far the system is from its point of stability) and decays over time to prevent the system from experiencing wind-up. The following equation ensures that the gain of the controller is bounded between two values, getting the best performance regardless of the current disturbance to the system.

<p align="center">
  <img src="/Resources/ControlTheory4.png" width="500"/>
  <br/>
  <em>ASMC Design - Page 4</em>
</p>

<p align="center">
  <img src="/Resources/ControlTheory5.png" width="500"/>
  <br/>
  <em>ASMC Design - Page 5</em>
</p>
