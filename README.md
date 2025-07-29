# BabyDiffDrive
A Differential Drive Controller with PID tunings, all written in MicroPython

## Motor Info:
  - Uses two pins: normal and interrupt, normal start, then interrupt is in the
  middle, use the interrupt pin to figure out clockwise or ccw movement (0/1)
  - Uses a PWM signal (max: 65536 - 1) to drive the speed of the motor down on the
  two encoder pins

## Hardware Used:
  - ESP32-Wroom-Dev Microcontroller Board
  - LN298 Motor Driver
  - Two JGA25 Motors

## Resources Used:
  - [Curio Res](https://www.youtube.com/watch?v=dTGITLnYAY0)
  - [CMU Kinematics Diff Drive](https://www.cs.cmu.edu/~rasc/Download/AMRobots3.pdf)
  - [Brett Blog *Awesome*](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-sample-time/)

