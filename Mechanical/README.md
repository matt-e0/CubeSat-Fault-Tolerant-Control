# Mechanical

CAD files and design documentation for the reaction wheel assembly. All structural components are 3D printed in PLA. Flywheels are machined from aluminium for higher density and rotational inertia.

<p align="center">
  <img src="/Resources/Screenshot 2026-03-14 223014.png" width="500"/>
  <br/>
  <em>CAD Model of full assembly</em>
</p>

## Design overview

The reaction wheel module is designed to fit within a 1U (100×100×100mm) CubeSat compartment. Four wheels are mounted in a pyramid (tetrahedron) configuration - each wheel's spin axis is tilted at approximately 54.7° from the body Z axis, with the four axes evenly distributed around Z.

This arrangement ensures:
- Full three-axis torque authority from four wheels
- Redundancy - any single wheel failure leaves the remaining three capable of full 3-axis control
- Equal torque distribution across all axes (condition number of allocation matrix = 2.41)

## Wheel inertia

The flywheel moment of inertia about its spin axis is estimated as:

```
I_wheel = 0.5 * m * r²
```

With mass ~14g and radius ~27mm this gives approximately `5.37×10⁻⁶ kg·m²`. 

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
