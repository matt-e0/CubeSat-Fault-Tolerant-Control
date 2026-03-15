# Project References & Design Basis

This document chronicles the theoretical and technical foundations of the **CubeSat Fault-Tolerant Control (FTC)** system.

---

## Sliding Mode Control Theory + Kinematics

[1] MATLAB, “What Is Sliding Mode Control?,” YouTube, Sep. 26, 2024. [Online]. Available: https://www.youtube.com/watch?v=RD-2oiwEbDo

[2] MAFarooqi, “NCS - 34a - Sliding Mode Control - Basic Concept,” YouTube, Dec. 01, 2023. [Online]. Available: https://www.youtube.com/watch?v=AQ9BJEJ8nKw

[3] “Sliding Mode Control,” Mathworks.com, 2017. [Online]. Available: https://uk.mathworks.com/help/slcontrol/ug/design-sliding-mode-control-reaching-law.html

[4] Y. S. Lee, N. P. Nguyen, and S. K. Hong, “Active fault‐tolerant control scheme for satellite with four reaction wheels: Multi actuator faults case,” *IET Control Theory & Applications*, vol. 19, no. 1, Jan. 2025, doi: https://doi.org/10.1049/cth2.70000.

## Quaternions, Frames and Reaction Wheel Matrix

[5] F. Franke, “Master Thesis Modeling and Control of Spacecraft Attitude - Control Allocation for a Variable Pointing and Maneuvering Satellite,” Norwegian University of Science and Technology. Accessed: Mar. 15, 2026. [Online]. Available: https://tomgra.folk.ntnu.no/Diplomer/Franke.pdf

[6] R. Valenzuela and J. Merino, “Spacecraft Dynamics, 2023/2024,” 2023. Accessed: Mar. 15, 2026. [Online]. Available: https://aero.us.es/dve/Apuntes/progDVE2324eng.pdf

## LQR/LPV Controller

[7] F. Cipro, “CubeSat attitude and position control systems based on LPV and Sliding Mode methods,” 2019. Accessed: Mar. 15, 2026. [Online]. Available: https://webthesis.biblio.polito.it/14486/1/tesi.pdf

## CubeSat and Reaction Wheels

[8] “NASA CubeSat/SmallSat Reference Model,” NASA Technical Reports Server. Accessed: Mar. 15, 2026. [Online]. Available: https://ntrs.nasa.gov/api/citations/20220009496/downloads/NASA%20CubeSat%20SmallSat%20Reference%20Model(PSAM).pdf

[9] N. Bonafede, “LOW-COST REACTION WHEEL DESIGN FOR CUBESAT APPLICATIONS,” M.S. Thesis, California Polytechnic State University, 2020. [Online]. Available: https://digitalcommons.calpoly.edu/cgi/viewcontent.cgi?article=3705&context=theses

---

## Traceability Log

| Design Component | Reference Source |
| :--- | :--- |
| **Fault Recovery Logic** | Lee et al. [4] |
| **Actuator Mapping ($B$ Matrix)** | Franke [5] |
| **Kinematics (Quaternions)** | Valenzuela & Merino [6] |
| **Control Law (SMC)** | Mathworks [3], MATLAB [1] |
| **Hardware Constraints** | NASA [8], Bonafede [9] |
