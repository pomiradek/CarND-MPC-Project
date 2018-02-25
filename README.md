# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## The Model
In this project I used the Kinematic simplified bicycle model. Kinematic models are simplifications of dynamic models
that ignore tire forces, gravity, air resistance and mass. This simplification reduces the accuracy of the model, but it also
makes the model more tractable.
The model next state equations are:
```
      // x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      
      where:
      x - x position of the car
      y - y position of the car
      psi - heading direction
      v - speed of teh car
      cte -  cross-track error
      epsi - orientation error
      delta - steering angle
      a - acceleration (throttle)
      Lf - distance between trhe center of mass of teh vehicle
```
The vehicle model is implemented in the FG_eval class.

## Timestep Length and Elapsed Duration (N & dt)
The prediction horizon is the duration over which future predictions are made. We’ll refer to this as `T`.
`T` is the product of two other variables, `N` and `dt`.
`N` is the number of timesteps in the horizon. `dt` is how much time elapses between actuations. For example, if `N` were 20 and `dt` were 0.5, then `T` would be 10 seconds.
`N`, `dt`, and `T` are hyperparameters which needs to tune for each model predictive controller.
However, there are some general guidelines:
- `T` should be as large as possible, while dt should be as small as possible
- In the case of driving a car, `T` should be a few seconds, at most. Beyond that horizon, the environment will change enough that it won't make sense to predict any further into the future
- Larger values of `dt` result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. This is sometimes called "discretization error".

A good approach to setting `N`, `dt`, and `T` is to first determine a reasonable range for `T` and then tune `dt` and `N` appropriately, keeping the effect of each in mind.
These guidelines create tradeoffs.

As sugegsted above I started with time horizon of 1 sec. I have tried combination of N = 20 and dt = 0.05 in contrast to N = 10 and dt = 0.1s.
First combination was not able to pass the track at target speed of 100mph. The second was resulting better and it was my final choice.
I also tried combination dt = 0.1 with N = 20 or N = 30. Both resulted in worse vehicle behavior, mainly in the sharp turns. maybe these values were too high to fit good polynom in 0.1s limit.
 
## Polynomial Fitting and MPC Preprocessing
The waypoints provided by the simulator are transformed to the car coordinate system:
```
     car_x = (ptsx[i] - px) * cos(0-psi) - (ptsy[i] - py) * sin(0-psi);
     car_y = (ptsx[i] - py) * sin(0-psi) + (ptsy[i] - px) * cos(0-psi);
```
Reference trajectory is typically passed to the control block as a polynomial. This polynomial is usually 3rd order, since third order polynomials will fit trajectories for most roads.

## Model Predictive Control with Latency
A contributing factor to latency is actuator dynamics. For example the time elapsed between when you command a steering angle to when that angle is actually achieved.
This could easily be modeled by a simple dynamic system and incorporated into the vehicle model. One approach would be running a simulation using the vehicle model starting from the current state for the duration of the latency. The resulting state from the simulation is the new initial state for MPC.
Thus, MPC can deal with latency much more effectively, by explicitly taking it into account, than a PID controller.

In my solution I computed next state in t0+delay time using these equations:
```
        const double x0 = 0;
        const double y0 = 0;
        const double psi0 = 0;
    
        // compute next postion in t+delay
        double x_delay = x0 + ( v * cos(psi0) * delay );
        double y_delay = y0 + ( v * sin(psi0) * delay );
        double psi_delay = psi0 - ( v * steer_value * delay / mpc.Lf );
        double v_delay = v + throttle_value * delay;
        double cte_delay = cte0 + ( v * sin(epsi0) * delay );
        double epsi_delay = epsi0 + ( v * epsi0 * delay / mpc.Lf );
    
        Eigen::VectorXd state(6);
        state << x_delay, y_delay, psi_delay, v_delay, cte_delay, epsi_delay;
          
```
Until this step I was not able to reach 100mph, at this speed it was oscillating. When I incorporate the latency to my model, the vehicle was able to run much faster.

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.