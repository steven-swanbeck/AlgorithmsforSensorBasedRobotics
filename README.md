# Screw Theory-Based Manipulator Analysis Code Featuring Forward Kinematics, Manipulability Analysis, and Inverse Kinematics
Associated with ME 397 Algorithms for Sensor-Based Robotics at UT Austin
---

The examples shown here are for a Franka-Emika Panda robot, though the code is easily generalizable to any robot by adding the relevant parameters to instaniate_robot.m.

<p align="middle">
  <img src="/gifs/animation2.gif" width="500" /> 
</p>

---

## Screw-Based Forward Kinematics
<!-- Zero configuration space frame FK:
![](https://github.com/steven-swanbeck/AlgorithmsforSensorBasedRobotics/blob/main/figures/fk_space_Test1.png)

Ready configuration space frame FK:
![](https://github.com/steven-swanbeck/AlgorithmsforSensorBasedRobotics/blob/main/figures/fk_space_Test2.png)
 -->
Zero and ready configurations of the robot using space frame FK:
<p align="middle">
  <img src="/figures/fk_space_Test1.png" width="300" />
  <img src="/figures/fk_space_Test2.png" width="300" /> 
</p>

---

## Manipulability Measures
Manipulability ellispoids at zero and ready configurations:
<p align="middle">
  <img src="/figures/ellipsoids_Test1.png" width="300" />
  <img src="/figures/ellipsoids_Test2.png" width="300" /> 
</p>

---

## Jacobian Pseudo-Inverse Inverse Kinematics
Algorithm performance:
<p align="middle">
  <img src="/figures/j_ik_Test1.png" width="300" />
  <img src="/gifs/j_ik_Test1.gif" width="300" /> 
</p>

---

## Jacobian Transpose Inverse Kinematics
Algorithm performance:
<p align="middle">
  <img src="/figures/jt_ik_Test1.png" width="300" />
  <img src="/gifs/jt_ik_Test1.gif" width="300" /> 
</p>

---

## Redundancy Resolution Inverse Kinematics
Algorithm performance:
<p align="middle">
  <img src="/figures/rr_ik_Test1.png" width="300" />
  <img src="/gifs/rr_ik_Test1.gif" width="300" /> 
</p>

---

## Damped Least_Squares Inverse Kinematics
Algorithm performance:
<p align="middle">
  <img src="/figures/dls_ik_Test1.png" width="300" />
  <img src="/gifs/dls_ik_Test1.gif" width="300" /> 
</p>

---

## Performance Comparison
Performance of Jacobian, Tranpose, Redundancy Resolution, and Damped Least-Squares algorithms compared:
<p align="middle">
  <img src="/figures/comp_Test3.png" width="600" /> 
</p>
<p align="middle">
  <img src="/gifs/j_ik_Test3.gif" width="145" />
  <img src="/gifs/jt_ik_Test3.gif" width="145" /> 
  <img src="/gifs/rr_ik_Test3.gif" width="145" /> 
  <img src="/gifs/dls_ik_Test3.gif" width="145" /> 
</p>

---
