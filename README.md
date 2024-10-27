# Differential Drive Robots
This project simulates a two-wheeled differential drive robot that follows given waypoints using either a PID controller or a Model Predictive Controller (MPC). The simulation allows for real-time parameter adjustments, multiple path types, and controller comparisons.

## Setup
It is recommended to use a virtual environment `venv` for this project. After set up the environment, make sure cooresponding dependencies are installed properly.

```text
pip install -r requirements.txt
```


## Simulation

The robot simulation defaults to using a PID controller. You can add waypoints interactively with the left mouse button and remove them with the right mouse button. To specify the path type (`circle` or `square`), use the following command to start the simulation:

``` shell
python main.py --type [square|circle]
```

<div align="center">
  <img src="./assets/cursor.gif" alt="cursor" width="235" />
  <img src="./assets/circle.gif" alt="circle" width="234" />
  <img src="./assets/square.gif" alt="square" width="236" />
</div>

Parameters for the controllers can be modified directly in the main file. This allows you to observe how different parameters affect robot behavior in real time, which helps in fine-tuning the control settings. Here, two parameters of PID controller have been applied, and the difference of behavior could be direcly observe through the simulation

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

This project includes multiple maps for comparing controller performance, with **`orange`** representing the PID-controlled robot and **`blue`** representing the MPC-controlled robot. To run an example with a specific map, use:

```bash
python example.py --map map/maps/map00.txt
```

<div align="center">
  <img src="./assets/map00.gif" alt="map00" width="235" />
  <img src="./assets/map02.gif" alt="map02" width="232" />
  <img src="./assets/map06.gif" alt="map06" width="233" />
</div>
<div align="center">
  <img src="./assets/map10.gif" alt="map10" width="235" />
  <img src="./assets/map03.gif" alt="map03" width="235" />
  <img src="./assets/map09.gif" alt="map09" width="235" />
</div>

Each controller pipeline provides methods `pipeline.extract_history()` to extract historical data for analysis:

```python
x, y, error, interval = pipeline.extract_history()
```

This history can be used to plot and analyze trajectory performance, control error, and time-series behavior, helping to optimize parameters or compare controllers.

<div align="center">
  <img src="./assets/map02.png" alt="map02-history" width="650">
</div>

Additional map files are available in the [`map`](./map/) folder. These maps can be used for developing or testing alternative control strategies or configurations. You can also create custom maps to suit your requirements.

## References
* [differential wheeled robot](https://en.wikipedia.org/wiki/Differential_wheeled_robot)
* [model predictive controller](https://en.wikipedia.org/wiki/Model_predictive_control)

