# Model Predictive Control

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

---

The goals / steps of this project are the following:

* Implement a Model Predictive Control (MPC) controller in C++, for steering angle and throttle.
* Test implementation by driving a car simulator around a track.

---

## 1. Files

My project includes the following files:

- [<b>C++</b> - The source code](https://github.com/ArjaanBuijk/CarND-MPC-Project/tree/master/src)
- [<b>README.md</b> - A summary of the project](https://github.com/ArjaanBuijk/CarND-MPC-Project/blob/master/README.md)
- [<b>video_MPC.mp4</b> - Video showing the car driving on the track using the MPC controller](https://github.com/ArjaanBuijk/CarND-MPC-Project/blob/master/Videos/video_MPC.mp4)

    ![track1](https://github.com/ArjaanBuijk/CarND-MPC-Project/blob/master/Videos/video_MPC.gif?raw=true)


---
## 2. Implementation

### The Model
In our Model Predictive Control implementation, we predict the future state of the car by using a kinematic model. Dynamic effects, like tire forces & inertia are ignored.

Each time-step, the MPC will be provided with a target trajectory to follow, and will be asked for the best control inputs to apply (steering & acceleration). In order to allow the MPC to calculate these control inputs, we include in our model two error measures that reflect the difference between the target trajectory and the predicted trajectory by our kinematic model. The two error measures are added as state Variables, resulting in:

<b>State Variables with error measures</b>
 - x	= x location of car center
 - y	= y location of car center
 - psi	= orientation angle of car
 - v	= velocity of car
 - cte = cross track error
 - epsi = orientation error

<b>Control Inputs (actuators)</b>
 - delta= steering angle
 - a	= acceleration (positive represents throttle, negative represents brake)

The equations that update the state variables from time t to time t+1 are given by:

    x(t+1) = x(t) + v(t)*cos(psi(t)) * dt
    y(t+1) = y(t) + v(t)*sin(psi(t)) * dt
    psi(t+1) = psi(t) + v(t)*delta(t) * dt / Lf
    v(t+1) = v(t) + a(t) * dt
    cte(t+1) = f(x(t+1)) - y(t+1)
    epsi(t+1) = psi(t+1) - atan(f'(x(t+1)))

Where:
 - All calculations are performed in the car coordinate system.
 - Lf is a characteristic of the car we are controlling. It represents the distance between the front wheels and its center of gravity. The simulator models a car with Lf=2.67 m.
 - f(x(t+1)) is the provided trajectories' y-location at time t+1
 - atan(f'(x(t+1))) is the angle of the provided trajectory at time t+1

These equations are implemented in the class FG_eval, as model constraints on an optimization problem. 

<b>The optimization problem</b>

To find the best actuation values (delta, a), we ask the optimizer to minimize this 'cost' function:

    cost = FACT_CTE             * SUM(cte*cte) +
           FACT_EPSI            * SUM(epsi*epsi) +
           FACT_V               * SUM(v*v) +
           FACT_DELTA           * SUM(delta*delta) +
           FACT_A               * SUM(a*a) +
           FACT_DELTA_CHANGE    * SUM(ddelta*ddelta) +
           FACT_A_CHANGE        * SUM(da*da)

The values for the hyperparameters were optimized by trial & error:

|Parameter|Value|Comment|
|-|-|-|
|FACT_CTE         |2.0| |
|FACT_EPSI        |1000.0| Penalize angle error more than cross track error. This improved Driving at higher speeds. |
|FACT_V           |0.1| |
|FACT_DELTA       |1.0| |
|FACT_A           |0.0| Do not penalize accelerating or braking. This improved driving at higher speeds by allowing car to apply the brakes.|
|FACT_DELTA_CHANGE|500.0| Penalize steering angle changes more. This enforces smoother curves.|
|FACT_A_CHANGE    |0.0| Do not penalize accelerating or braking. This improved driving at higher speeds by allowing car to apply the brakes|


### Timestepping (N & dt)
The optimizer will minimize the cost function for a certain predicted time horizon into the future. This is done by iterating over the kinematic model for a number of steps (N), with a fixed time-step (dt). The values for N and dt must be chosen carefully, taking the following into consideration:
- N*dt should be only a few seconds. The environment will change enough that it does not make sense to predict too far into the future.
- dt should be small, to avoid discretization errors.
- N should be small, to minimize computational cost.


The values for N & dt were optimized by trial & error:

|Parameter| Value|
|-|-|
|N|5|
|dt|0.3|

Besides these chosen values, many combinations were tried, for example (10,0.3), (10,0.1), (25,0.2), etc. The outcome was that the best choice depends on the target speed. It is best to use values for N and dt that give a time horizon that extends 2-3 waypoints forward.

### Polynomial Fitting, MPC Preprocessing and MPC with latency
To deal with the latency of 100 msec, the vehicle location after the latency period is first predicted using the kinematic model equations. That location is then used as the basis for the vehicle coordinate system in which all the rest of the calculations are performed. In detail, these are the steps used:

- The waypoints coordinates are transformed into the vehicle coordinate system.
- The waypoints in the local system are curve fitted with a 3rd order polynomial.
- The waypoints are offset into the direction of radius of curvature:
    ![track1](https://github.com/ArjaanBuijk/CarND-MPC-Project/blob/master/Images/radius_of_curvature.jpg?raw=true)
    - This makes the car take inside curves.
    - The offset is calculated as:
        - y   = f(x) : function fitted to waypoints
        - f'  = df/dx   : first derivative, giving tangential direction.
        - f'' = df'/dx  : second derivative is the change in angle along trajectory.
        - nx,ny         : normal direction. There are two normals. Pick the 'inside' normal.
        - R   = (( 1+(f'*f')^(3/2 ) / abs(f'') : Radius of curvature
        - offset = min(1.75, RMIN/R) 
        - RMIN = 100m
        - RMIN/R is the amount of offset:
        - = 0 for a straight line
        - = Higher for tighter curves
        - = Capped at 1.75m

- The offsetted waypoints are again curve fitted with a 3rd order poynomial and provided to the MPC as the target trajectory to follow.

This image shows the offsetted waypoints in green, with the orignal waypoints in yellow:

![track1](https://github.com/ArjaanBuijk/CarND-MPC-Project/blob/master/Images/Offset_Waypoints.jpg?raw=true)

### Reference speed & Top speed reached
The highest reference speed with which the car safely drives around the track is <b>110 mph</b>.

The top speed reached is <b>105.18 mph</b>, reached early during the 2nd time around the track:
![track1](https://github.com/ArjaanBuijk/CarND-MPC-Project/blob/master/Images/Top_Speed.jpg?raw=true)





# 3. Summary

The result can be summarized as follows:

- The MPC controller is very good in driving the car smoothly around the track.
- To avoid weaving, it is important to  keep the orientation of the car very closely aligned with the waypoint trajectory. This is achieved by applying a very high penalty in the cost function for delta and change in delta. This was one of the key findings allowing the car to drive at higher speeds.
- Furthermore, to allow driving at very high speeds, over 100 mph, even with latency in the system, it helps to offset the trajectory to the inside of curves, and to take acceleration completely out of the cost function.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
