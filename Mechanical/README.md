# Mechanical

CAD files and design documentation for the reaction wheel assembly. All structural components are 3D printed in PLA. Flywheels are machined from aluminium for higher density and rotational inertia.

<p align="center">
  <img src="/Resources/Screenshot 2026-03-14 223014.png" width="500"/>
  <br/>
  <em>CAD Model of full assembly</em>
</p>

## Design overview

The reaction wheel module is designed to fit within a 1U (100×100×100mm) CubeSat compartment. Four wheels are mounted in a pyramid (tetrahedron) configuration - each wheel's spin axis is tilted at approximately 54.7° from the body Z axis, with the four axes evenly distributed around Z.

<p align="center">
  <img src="/Resources/Screenshot 2026-03-14 223140.png" width="500"/>
  <br/>
  <em>CAD Render of CubeSat 1U compartment</em>
</p>

This arrangement ensures:
- Full three-axis torque authority from four wheels
- Redundancy - any single wheel failure leaves the remaining three capable of full 3-axis control
- Equal torque distribution across all axes (condition number of allocation matrix = 2.41)

<p align="center">
  <img src="/Resources/Screenshot 2026-03-14 223248.png" width="500"/>
  <br/>
  <em>CAD Render of CubeSat 1U compartment</em>
</p>

## Wheel inertia

The flywheel moment of inertia about its spin axis is estimated as:

```
I_wheel = 0.5 * m * r²
```

With mass ~14g and radius ~27mm this gives approximately `5.37×10⁻⁶ kg·m²`. Since there is limited mass and therefore, volume that the flywheel can occupy, the best method to increase performance is to increase its density or rotational velocity. This design uses aluminium flywheels despite aluminiums relatively low desnity, as it allows the motors to spin them up to incredibly high speeds. It also reduces the overall weight of system, reducing launch cost.

Motor testing with the aluminium flywheel demonstrated a capability to comfortably reach 6000rpm, enabling it to provide enough angular momentum to compete with commercial reaction wheels in this domain. Further testing demonstrated the capability to reach over 14000rpm. However, the power requirements to drive the motors at such high speed nullify the advantage of the torque increase. At roughly 11.1V * 2.9A each motor would consume roughly 32W each, which would not be .feasible. Instead the upper rpm limit is set to 6000rpm, as the angular momentum is more than adequate at around 0.0035Nms, or 0.014Nms across all wheels.

## Motor and encoder

Each wheel uses a BLDC motor paired with an AS5047P magnetic encoder. The encoder provides quadrature A/B signals to the STM32 B-G431B ESC for closed-loop speed control via Field Oriented Control (FOC).

## Torque allocation matrix

The geometry of the pyramid gives the following allocation matrix `A` (maps wheel torques to body torques):

```
A = [-0.5,  -0.5,   0.5,  0.5]   ← body X
     [ 0.5,  -0.5,  -0.5,  0.5]   ← body Y
     [0.707, 0.707, 0.707, 0.707]  ← body Z
```

The pseudoinverse `A_pinv = pinv(A)` is precomputed and hardcoded in the Teensy controller. If the physical wheel arrangement differs from this geometry, recalculate `A` from the actual spin axis unit vectors and update `A_pinv` in `controller/main3.cpp`.

## 3-Axis Gimbal Platform

<p align="center">
  <img src="/Resources/Screenshot 2026-02-17 081849.png" width="500"/>
  <br/>
  <em>CAD bearings in frame</em>
</p>

To test the performance of the hardware prototype a 3-axis gimbal platform was constructed using aluminium extrusion and bearings, connected via 3D printed mounts and aluminium rod to form each frame (one frame for each axis). This particular setup was chosen as it's relatively simple to design and construct but still offers full range of motion. The short comings of this design are that the overlapping rings are subject to gimbal lock, which can impede the motion of the satellite.

In testing the platform was found to be inadequate, with too much friction in the bearings and weight distributed off balance with the centre point of the CubeSat unit, the reaction wheels could not generate enough torque to move the prototype freely. This could be remedied by using an air-bearing sphere, as it would offer 3-axis orientation with near zero friction.

