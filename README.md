# Differential Drive Robot
A simulation of two wheel differential drive robot. The robot could track a given path with PID controller or model predictive controller. And some tracks have been designed for comparing two different controller.

## Usage
In the basic mode, the robot using PID controller in default. User can use their mouse to add or remove waypoint on the window, and observe the effects of different parameters.
``` shell
python main.py
```
<div style="text-align: center;">
  <img src="./assets/keyboard.gif" alt="keyboard" width="300" />
</div>

## Comparison
In comparison mode, we can simply observe the behavior of PID control in orange robot and MPC in blue robot. 

<div style="text-align: center;">
  <img src="./assets/map00.gif" alt="map00" width="300" />
  <img src="./assets/map01.gif" alt="map01" width="293" />
  <img src="./assets/map02.gif" alt="map02" width="297" />
</div>
<div style="text-align: center;">
  <img src="./assets/map03.gif" alt="map03" width="300" />
  <img src="./assets/map04.gif" alt="map00" width="298" />
</div>

