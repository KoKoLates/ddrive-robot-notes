# Model Predictive Control

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

## Optimization
After the designa of cost function, we can apply the optimization solver. Typically, the optimization problem will be formed into linear programming or quadratic programming format, and then we can use corresponding solver to deal with the problem, and obtain the corresponding optimal parameters $v, w$

$$
\min_{v,w}\mathcal J
$$

The optimization is subject to input constraints, which are bounds on the control inputs

$$
v_{\  min}\leq v_k\leq v_{\max}, w_{\min}\leq w_k\leq w_{\max}
$$

where $v_k$ is the linear velocity and $w_k$ is the angular velocity at time step $k$.
