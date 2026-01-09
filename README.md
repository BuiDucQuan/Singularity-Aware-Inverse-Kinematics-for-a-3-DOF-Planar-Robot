<<<<<<< HEAD
# 3-DOF Planar Robot – Kinematics and Singularity Analysis

## Summary
This project demonstrates my understanding of robotic kinematics and singularity analysis
through the implementation of a 3-DOF planar robotic arm.

The focus is on forward kinematics, Jacobian analysis, and geometric interpretation of
singular configurations, which are fundamental topics in robot motion planning and control.

---

## Skills Demonstrated
- Robot kinematics (FK)
- Jacobian matrix computation
- Singular configuration analysis
- Geometric interpretation of motion constraints
- Python for robotics simulation
- Clear project structuring and documentation

---

## Robot Model
- Type: 3-DOF planar RRR robot
- Joints: Revolute
- Workspace: 2D (x, y)
- Configuration vector: q = [q1, q2, q3]

---

## Key Experiments
- Visualization of robot configurations
- Jacobian computation at different joint angles
- Rank and singular value analysis
- Identification of singular configurations

---

## Results
When the robot links become collinear, the Jacobian matrix loses rank.
At this singular configuration, the end-effector loses instantaneous motion
capability in the direction perpendicular to the arm.

This behavior is verified both mathematically (Jacobian rank deficiency)
and geometrically (robot configuration).

---

## Why This Project Matters
Understanding singularities is critical for:
- Motion planning
- Inverse kinematics
- Safe robot control near workspace boundaries

This project reflects practical knowledge required for robotics and
motion planning engineering roles.

---

## How to Run
```bash
python src/main.py






=======
# Singularity-Aware-Inverse-Kinematics-for-a-3-DOF-Planar-Robot
>>>>>>> 5db6f761b74827467432d2fa729d74f5ce13fc19
