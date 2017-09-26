# FuzzyLogic_Quad
Quadcopter flight controller algorithm using fuzzy logic for attitude control

Quadcopter code is compatable with teensy 3.2 board and MPU-6050 imu. If u wish to use other micro-controller (arduino)
the user defined macros for digital read needs to be changed accoding to which micrcontroller is used.

The Fuzzy logic controller behaves like a PID-Algorithm and uses the same inputs. However, it has the posibility
of increasing the non-linarity by adding more membership functions.


