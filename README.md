# 6-Axis Robotic Arm

## 1. Introduction
The main goal of this project was to create a 6-DoF robotic arm that is mostly* 3D printed.  
\* "mostly" means that all custom parts are 3D printed, relying only on commonly available components like screws, timing belts, and ball bearings.

This approach provides several key benefits:
- **Modular construction**: 3D printing allows for easy design improvements and modular upgrades.
- **Easy maintenance**: Repairs are fast and cheap, eliminating the need to order specific parts.
- **Safety**: The lightweight design allows operators to work safely near the robot without the risk of serious injuries.
- **Complete ownership**: The author has full control over the design and modifications.

The arm's architecture is based on standard 6-DoF industrial manipulators to avoid reinventing the wheel. This makes it an ideal educational platform. It mimics the robots future engineers will encounter, but remains forgiving of operator mistakes.

---

## 2. Design
All 3D printed components were custom designed by the author in FreeCAD. The entire robot was manufactured using PLA on a Bambu Lab A1 printer equipped with a standard 0.4 mm nozzle.

---

## 3. Electronics & Actuation
- **Stepper Motors**: NEMA 17 and NEMA 23 stepper motors drive the primary axes, chosen for their torque and wide availability among hobbyists (see Chapter 5.1 in Masters_Thesis.pdf).
- **Servomotors**: The wrist and end effector utilize 180° servomotors. These are recommended for their low weight. Since most grippers are symmetrical, a 180° range is fully sufficient for standard operations.
- **Controller**: An STM32H755ZI-Q microcontroller serves as the main board, selected for its reliability, high processing speed, and hardware FPU necessary for real-time kinematics.

---

## 4. Control & Software
- **Communication**: A Serial (UART) connection is used to transmit data between the operator's PC and the STM32 board.
- **Interface**: The operator's GUI was developed in Python. It handles user inputs and sends movement commands directly to the microcontroller.
