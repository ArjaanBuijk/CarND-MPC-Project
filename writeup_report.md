# Model Predictive Control Project
---

The goals / steps of this project are the following:

* Implement a Model Predictive Control (MPC) controller in C++, for steering angle and throttle.
* Test implementation by driving a car simulator around a track.

---

## 1. Files

My project includes the following files:

- [<b>C++</b> - The source code](https://github.com/ArjaanBuijk/CarND-MPC-Project/tree/master/src)
- [<b>writeup_report.md</b> - A summary of the project](https://github.com/ArjaanBuijk/CarND-MPC-Project/blob/master/writeup_report.md)
- [<b>video_MPC.mp4</b> - Video showing the car driving on the track using the MPC controller](https://github.com/ArjaanBuijk/CarND-MPC-Project/blob/master/Videos/video_MPC.mp4)

    ![track1](https://github.com/ArjaanBuijk/CarND-MPC-Project/blob/master/Videos/video_MPC.gif?raw=true)


---
## 2. Implementation

### The Model
In our Model Predictive Control implementation, we predict the future state of the car by using a kinematic model. Dynamic effects, like tire forces & inertia are ignored.

Each time-step, the MPC will be provided with a target trajectory to follow, and will be asked for the best control inputs to apply (steering & acceleration). In order to allow the MPC to calculate these control inputs, we include in our model two error measures that reflect the difference between the target trajectory and the predicted trajectory by our kinematic model. The two error measures are added as state Variables, resulting in:

<u>State Variables with error measures</u>
 - x	= x location of car center
 - y	= y location of car center
 - psi	= orientation angle of car
 - v	= velocity of car
 - cte = cross track error
 - epsi = orientation error

<u>Control Inputs (actuators)</u>
 - delta= steering angle
 - a	= acceleration (positive represents throttle, negative represents brake)

The equations that update the state variables from time t to time t+1 are given by:

    x(t+1) = x(t) + v(t)*cos(psi(t)) * dt
    y(t+1) = y(t) + v(t)*sin(psi(t)) * dt
    psi(t+1) = psi(t) + v(t)*delta(t) * dt / Lf
    v(t+1) = v(t) + a(t) * dt
    cte(t+1) = f(x(t+1)) - y(t+1)
    epsi(t+1) = psi(t+1) - f'(x(t+1))

Where:
 - All calculations are performed in the car coordinate system.
 - Lf is a characteristic of the car we are controlling. It represents the distance between the front wheels and its center of gravity. The simulator models a car with Lf=2.67 m.
 - f(x(t+1)) is the provided trajectories' y-location at time t+1
 - f'(x(t+1)) is the angle of the provided trajectory at time t+1

These equations are implemented in the class FG_eval, as model constraints on an optimization problem. 

<u>The optimization problem</u>
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
|FACT_A           |0.0| Do not penalize accelerating or breaking. This improved driving at higher speeds by allowing car to apply the breaks.|
|FACT_DELTA_CHANGE|500.0| Penalize steering angle changes more. This enforces smoother curves.|
|FACT_A_CHANGE    |0.0| Do not penalize accelerating or breaking. This improved driving at higher speeds by allowing car to apply the breaks.penalize acceleration less. Allow breaking.|


### Timestep Length and Elapsed Duration (N & dt)
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
To deal with the latency of 100 msec, the vehicle location after the latency period is first predicted using the kinematic model equations. That location is then used as the basis for the vehicle coordinate system in which all the rest of the calculations are performed:

- The waypoints coordinates are transformed into the vehicle coordinate system.
- The waypoints in the local system are curve fitted with a 3rd order polynomial.
- The waypoints are offset into the direction of radius of curvature:
    ![track1](https://github.com/ArjaanBuijk/CarND-MPC-Project/blob/master/Images/radius_of_curvature.jpg?raw=true)
    - This makes the car drive take inside curves.
    - The offset is calculated as:
            y   = f(x) : function fitted to waypoints
            f'  = df/dx   : first derivative, giving tangential direction.
            f'' = df'/dx  : second derivative is the change in angle along trajectory.
            nx,ny         : normal direction. There are two normals. Pick the 'inside' normal.
            R   = (( 1+(f'*f')^(3/2 ) / abs(f'') : Radius of curvature
            offset = min(1.75, RMIN/R):  
            where: 
                RMIN = 100m
                RMIN/R is the amount of offset:
                -> 0 for a straight line
                -> Higher for tighter curves
                -> Capped at 1.75m

- The offsetted waypoints are again curve fitted with a 3rd order poynomial.

This image shows the offsetted waypoints in green, with the orignal waypoints in yellow:

![track1](https://github.com/ArjaanBuijk/CarND-MPC-Project/blob/master/Images/Offset_Waypoints.jpg?raw=true)

### Reference speed & Top speed reached
The highest reference speed with which the car safely drives around the track is <b>110 mph</b>.

The top speed reached is <b>105.18 mph</b>, reached early during the 2nd time around the track:
![track1](https://github.com/ArjaanBuijk/CarND-MPC-Project/blob/master/Images/Top_Speed.jpg?raw=true)





# 3. Summary

The result can be summarized as follows:

- The MPC controller is very good in driving the car smoothly around the track.
- To avoid weaving, it is important to  keep the orientation of the car very closely aligned with the waypoint trajectory, by applying a very high penalty in the cost function for delta and change in delta. This was one of the key findings allowing the car to drive at higher speeds.
- To allow driving at very high speeds, over 100 mph, even with latency in the system, it helps to offset the trajectory to the inside of curves, and to take acceleration out of the cost function.