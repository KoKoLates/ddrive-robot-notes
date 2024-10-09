# Differential Drive Robot
A simulation of two wheel differential drive robot. The robot could track a given path with PID controller or model predictive controller. And some tracks have been designed for comparing two different controller.

* simple description
* how to use

User could create a virtual environment `.venv` for the project. And make sure all dependencies are installed properly. Apply the command below to setup properly.

## Usage
In the basic mode, the robot using PID controller in default. User can use their mouse to add or remove waypoint on the window, and observe the effects of different parameters. The default is user mouse event, user can use left button to create waypoint and right button to remove waypoint from the list. 
``` shell
python main.py --type [square|circle]
```

<div align="center">
  <img src="./assets/cursor.gif" alt="cursor" width="235" />
  <img src="./assets/circle.gif" alt="circle" width="234" />
  <img src="./assets/square.gif" alt="square" width="236" />
</div>

Tuning the controller parameter with the simulation application, here we test on different parameter of PID controller and observe the difference of behavior between different condition.  

<div align="center">
  <img src="./assets/param1.gif" alt="param1" width="235" />
  <img src="./assets/param2.gif" alt="param2" width="232" />
</div>

## Model Predictive Controller
The objective of the model predictive control is to minimize a cost function over a finite prediction horizon (window) $n$. The cost function is mainly compose of three terms:
1. **state cost**: to penalize deviation of robot's state from target
2. **input cost**: to penalize large control input
3. **input difference cost**: encourage smooth control inputs by penalizing large difference between consecutive control inputs

Thus, the cost function to minimize is:

$$
\mathcal J(u_k)=\sum_{k=0}^{n-1}\Big((x_k-t)^TQ(x_k-t)+u_k^TRu_k+(u_{k+1}-u_k)^TR_d(u_{k+1}-u_k) \Big)
$$

where:
* $t$ is taget
* $x_k$ and $u_k$ is the robot state and control input at step $k$
* $Q, R$ and $R_d$ are positive definite matrices representing the weight matrices for state error, control input and control input difference cost respectively 

<br/>

The optimization is subject to input constraints, which are bounds on the control inputs

$$
v_{\  min}\leq v_k\leq v_{\max}, w_{\min}\leq w_k\leq w_{\max}
$$

where $v_k$ is the linear velocity and $w_k$ is the angular velocity at time step $k$.


## Examples
There are several maps could be used. And in the example, you can compare two different controller with PID in **orange** robot and model predictive controller in **blue** robot.
```bash
python example.py --map map/maps/map00.txt
```


<div align="center">
  <img src="./assets/map00.gif" alt="map00" width="235" />
  <img src="./assets/map02.gif" alt="map02" width="232" />
  <img src="./assets/map06.gif" alt="map06" width="233" />
</div>
<div style="text-align: center;">
  <img src="./assets/map10.gif" alt="map10" width="235" />
  <img src="./assets/map03.gif" alt="map03" width="235" />
  <img src="./assets/map09.gif" alt="map09" width="235" />
</div>

The are more map, please see [here](./map/) for more information.

```python
x, y, error, interval = pipeline.extract_history()
```

<div align="center">
  <img src="./assets/map02.png" alt="map02-history" width="650">
</div>

## References
* [differential drive robot](https://en.wikipedia.org/wiki/Differential_wheeled_robot)
* [model predictive control](https://en.wikipedia.org/wiki/Model_predictive_control)

