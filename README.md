# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

# The Model

I've decided to stick with the kinematic model presented in the class, as I didn't had any specification or time to measure the pyhsics of the car in the simulator.
Therfore the state equation are the following:

```
// Kinematic model
AD<double> f = ( coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3) );
AD<double> psi = CppAD::atan( coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2) );

fg[2 + t + x_start]     = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[2 + t + y_start]     = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[2 + t + psi_start]   = psi1 - (psi0 + (v0/Lf) * delta0 * dt);
fg[2 + t + v_start]     = v1 - (v0 + acc0 * dt);
fg[2 + t + cte_start]   = cte1 - ((f - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[2 + t + epsi_start]  = epsi1 - ((psi0 - psi) + (v0/Lf) * delta0 * dt);
```

The same equations have been applied to calculate the offset due to the latency ( see below )

# Hyperparameters

The hyperparameter have been choosen in the attempt to maximize the speed, in order to do so, the minimum suitable value for DT that wasn't triggering dangeroud oscilation was 0.1, higher values were resulting not suitable for high speed due to the lack of resolution.
One of the limits of driving at high speed is the computation time required the by optimizer, even if a limit of 0.5 sounds like a negligebale time gap, as high speed, let's say 100 mps, is equivalent to a more than 20m without the car updating the controls. Just one 0.5s delay is enoush to crash the car.
In order to speedup the optimizer N have been kept to 10, below which, it would fail to interpolate correctly.
Also, in order to simplify the work of the optimizer at high speed, I decided keep at the bare minimum the penalization of the use of the actuators.
I've tried to reduce the 'max_cpu_time' in order to minimize such delayies, which worked, however the car started to behave erratically, but at the same time it greatly increade the reactivity. 
It would be an interesting experiment to implement a "real-time" system that keep the processing time at an acceptable minimum, and detect failed "optimization", skip the use of actuators and repeat the calculations using the new state from the sensors, perhaps the whole system  might need to take advantage of multithreading and hardware acceleration in order yield higher performance and remain reliable.

# Car Coordinates

The reference points given in map coordinates have been converted into car coordinates using the following equations:

```
const unsigned point_num = ptsx.size();
VectorXd xvals = VectorXd::Zero(point_num);
VectorXd yvals = VectorXd::Zero(point_num);

const double cos_psi = cos(-psi);
const double sin_psi = sin(-psi);
for (int i=0; i < point_num; i++ ){
  xvals(i)=  cos_psi * (ptsx[i] - px) - sin_psi * (ptsy[i] - py);
  yvals(i)=  sin_psi * (ptsx[i] - px) + cos_psi * (ptsy[i] - py);
}
```

# Polinomial fitting

A 3th order polinomial have been chosen to fit the waypoints, as a lower order faild to represent close subsequent turns into opposite directions and a higher order polinomial had an unnescessary level of complexity.


# Latency correction

The same model equation have been applied to the variables in car coordinates (px, py, psi = 0), using the equations coming from the kinematic model, cte and epsi have been computed taking into account the latency.
```
const double v_ms = v*mph2mps;
const double latency = latency_s; // push it more in the future
const double lat_m = v_ms * latency; // meters offset for given latency

double cte0 =  coeff[0]; // fit first derivate for x = 0
double epsi0 = -std::atan( coeff_d1[0] ); // fit first derivate for x = 0

px  = lat_m* cos(psi);
py  = lat_m * sin(psi);
psi = (lat_m/Lf) * -steering_angle;
double epsi = epsi0 + psi;
double cte = cte0 + lat_m*sin(epsi);
v = v + throttle*latency; //throttle != acceleration but probably better than constant velocity

```
## State
Using variables accounting the latency I've then assembled the state vector and passed it to the solver.
```
VectorXd state = VectorXd(6);
state << px,py,psi,v,cte,epsi;
auto values = mpc.Solve(state, coeff);
```

## Actuators
The result of the solver have been averaged in order to partially compensate for the latency, is a rather crude method, perhaps more accurate results could be obtained by taking into account the exact amount of time required by the computation (optimizer mostly, but not only)
```
const double delta = (sx[delta_start]+sx[delta_start+1]+sx[delta_start+2])/3;
const double acc = (sx[acc_start]+sx[acc_start]+sx[acc_start+1]+sx[acc_start+2])/3;
```
