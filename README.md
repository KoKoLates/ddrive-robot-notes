# Differential Wheeled Robots
This interactive simulation allows users to explore and compare the performance of a two-wheeled differential drive robot as it follows waypoints using a **PID controller** or a **Model Predictive Controller (MPC)**. Experience real-time parameter adjustments, different path types, and detailed controller performance analysis.


It is recommended to use a virtual environment `venv` for this project. After set up the environment, make sure cooresponding dependencies are installed properly.

```text
pip install -r requirements.txt
```


## Quick Start

The robot simulation defaults to using a PID controller. To add waypoints interactively, use the left mouse button, and to remove them, use the right mouse button. Start the simulation with your desired path type (`circle` or `square`) using the following command:

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

This project includes multiple maps designed for comparing controller performance. The maps offer various environments that highlight the strengths and weaknesses of the different control strategies. In the visual outputs, **`orange`** represents the PID-controlled robot, while **`blue`** indicates the MPC-controlled robot. To run an example using a specific map, execute the following command in your terminal, replacing `map00.txt` with the desired map file name:

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

After running the simulation, you can utilize the `pipeline.extract_history()` method to analyze the trajectory performance and control errors. This function allows you to capture the robot's historical data, including position, velocity, and the errors encountered along the way.

```python
x, y, error, period = pipeline.extract_history()
```

The historical data extracted from the simulation can be invaluable for analyzing various aspects of the robot's performance like trajectory performances, control error analysis and comparative performance.

<div align="center">
  <img src="./assets/map02.png" alt="map02-history" width="650">
</div>

Additional map files are available in the [`map`](./map/) folder. These maps can be used for developing or testing alternative control strategies or configurations. You can also create custom maps to suit your requirements.

## References
* [Differential Wheeled Robot](https://en.wikipedia.org/wiki/Differential_wheeled_robot)
* [Model Predictive Control](https://en.wikipedia.org/wiki/Model_predictive_control)
