# MPC Project Reflection

## Overview

I used the solution to the MPC Quiz as the base for my work. The difficulty in completing this project came from dealing with global coordinates and a realistic map, tuning the cost function, and determining a good size and quantity for the time steps.

## The Model

### State Equations

I followed the kinematic motion model described in the lessons. The state consisted of six parts:

* x, the x position of the car.
* y, the y position of the car.
* psi, the car's rotation relative to the global coordinate system.
* v, the speed of the car.
* cte, the cross-track error.
* epsi, the rotation of the car relative to the road.

### Update Equations

Using the kinematic model, the global x and y position of the car are calculated in the usual way:

```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
```

where `t` is the time step number and `dt` is the duration of a time step.

The velocity at a given time step also updates intuitively:

```
v[t+1] = v[t] + a[t] * dt
```

where `a` is the throttle.

The rotation relative to the global coordinate system is updates as shown in the lessons:

```
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
```

where `Lf` is the length from the front of the car to its center of gravity, and `delta` is the steering angle. `Lf` is a tuned variable that was provided by Udacity.

The rotation relative to the road updates similarly to `psi`, although it also factors in `psides`, the initial angle relative to the road:

```
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

`psides` is calculated by taking the arctangent of the first order polynomial coefficient, which represents the line tangent to the polynomial-fitted road at the origin.

Finally, the cross-track error is calculated based on the initial cross-track error plus the predicted cross-track motion based on the car's track-relative rotation and speed:

```
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
```

`f(x)` is the evaluation of the polynomial at the car's x coordinate at time t.

### Actuators and Cost Function

Ipopt with CppAD was used to minimize a cost function, and the throttle and steering were contained in the solution.

The cost function minimizes 7 parameters over a given timeline:

* Cross-track error.
* Rotation relative to the road.
* Difference between target speed and actual speed.
* Steering magnitude.
* Throttle magnitude.
* Rate of change of steering.
* Rate of change of throttle.

I uses the cost function from Udacity's solution to the MPC Quiz as a base. To get the car to drive around the track successfully, I found that I had to heavily weigh steering magnitude and the rate of change of steering. The idea is that huge changes in steering should not be necessary, and sudden change in steering should also not be necessary. Otherwise, the car tended to either oscillate around the road's center line or steer off the track entirely.

I also had to increase the weight of the cross-track error. I found that weighing this parameter heavily would steer the car off the track, but too small of an error would cause it to slowly drift away from the center line.

## Timestep length N and elapsed duration dt

I chose the following values for N and dt:

* N = 30
* dt = 0.1

These values were determined via trial and error.

I began with the values from the quiz, N = 20 and dt = 0.05. I found that these values were too small. They only predicted 1 second ahead, meaning the car did not anticipate road curvature far enough into the future. This ultimately caused problems around turns, since the car would wait until it was too late to begin turning and end up turning too hard.

I had also tried higher timestep length values like 40 and above, along with longer elapsed durations, going as high as 0.25. There were two issues I noticed at higher values:

1. Numerical stability - The solver would extrapolate too far into the future at times and fit to a curve that caused sharp changes.
2. Computational load - The increase in the number of values increased the time it took just to perform the calculation, to the point where it was the limiting factor in updating the actuators.

The values I ultimately settled on provided a good balance of computational speed, future projection, timestep resolution, and numerical stability.

## Polynomial Fitting and MPC Preprocessing

I fit the road waypoints to a 3rd order polynomial, since this level of resolution is generally able to fit most road surfaces. A 3rd order polynomial allows for inflection points in the road surface, which do occur fairly often.

I had to preprocess the waypoints by mapping them from the global coordinate system into the car's coordinate system. This was necessary for two reasons:

1. In general, the polynomial fit cannot be performed in the global coordinate system. This is because the points may sit very close together on the x axis, depending on the yaw of the car, and therefore produce a curve that cannot be accurately fit. Despite this, the model will take the coefficients produces and try to fit them anyway, resulting in wildly inaccurate projections. By mapping the global coordinates into the car's coordinate system, it will generally be possible to produce a valid polynomial fit.
2. To visualize the points on-screen, the simulator requires waypoint coordinates in the car's coordinate system rather than the global coordinate system.

Note that as a result of working in the car's coordinate system, it is the case that the x, y, and psi parameters fed to the initial optimizer state are always 0. This makes sense, since the car is always at the origin with zero yaw in its own coordinate system.

The mapping itself occurs in the function `getForwardWaypointsInCarCoordinateSystem`. This mapping is a standard translation by the car's offset followed by a rotation based on the car's yaw.

## Model Predictive Control with Latency

Before introducing the required 100ms latency, I developed a solution for the case where there is no latency. I saw that once latency was added, the car tended to oscillate about the road centerline and switch between acceleration and deceleration often.

Although my solution was still successful, I found that I was able to improve it by choosing the throttle and steering value that are anticipated by the model at the 100ms mark, rather than the instantaneous throttle and steering value. This makes sense; if it takes 100ms for the signal to reach the actuators, then the car should be actuated with the throttle and steering value that is predicted to be necessary at that time rather than the current time.

It was noted by a previous project reviewer that it's necessary to constrain the bounds of the first timestep's acceleration and steering values when using this method. I had not done so originally, but I have since updated the solver to do so.
