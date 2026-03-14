# Adaptive Sliding Mode Controller Design

The at heart of the system is the Adaptive Sliding Mode Controller (ASMC). ASMC was chosen over traditional control methods due the requirement of fault-tolerance for the attitude control system.

## Passive VS Active Fault Isolation

| Passive | Active |
|-----------|------|
| Inherently robust, doesn't require detection layer | More efficient distribution of torque, improved wheel health |
| Reduces processing load of microcontroller (no reaction wheel speed sensing or heavy matrix updates) | Can achieve a higher pointing accuracy during failures as feedback prevents overcompensation |
| Independent of fault-detection architecture, no risk for false-positives | System doesn't try to power faulty motors |


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

<p align="center">
  <img src="/Resources/ControlTheory3.png" width="500"/>
  <br/>
  <em>ASMC Design - Page 3</em>
</p>

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
