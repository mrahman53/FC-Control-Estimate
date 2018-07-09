# 3D Quadrotor Estimation Project #

This project is about the estimation portion of the controller used in the CPP simulator. The simulated quad will be flying with the estimator and the custom controller (from the previous project)!
By using EKF(Extended Kalman Filter), we can fusion noisy GPS, IMU and magnetometer to estimate current drone position, velocity and yaw.

### Project Structure ###

The files that are needed to modify are below:

 - The EKF was already partially implemented in src/QuadEstimatorEKF.cpp

 - Parameters for tuning the EKF are in the parameter file config/QuadEstimatorEKF.txt

 - Cascaded PID control implemented from the last project are in src/QuadControl.cpp

 - Parameters for the control code are in config/QuadControlParams.txt


### Project outline: ###

 - [Step 1: Sensor Noise](#step-1-sensor-noise)
 - [Step 2: Attitude Estimation](#step-2-attitude-estimation)
 - [Step 3: Prediction Step](#step-3-prediction-step)
 - [Step 4: Magnetometer Update](#step-4-magnetometer-update)
 - [Step 5: Closed Loop + GPS Update](#step-5-closed-loop--gps-update)
 - [Step 6: Adding Your Controller](#step-6-adding-your-controller)



### Step 1: Sensor Noise ###

For the controls project, we assumed the simulator was working with a perfect set of sensors, meaning none of the sensors had any noise. But that was not the case, so the first step to adding additional realism to the problem, and developing an estimator, is adding noise to the quad's sensors.  For the first step, we collect some simulated noisy sensor data and estimate the standard deviation of the quad's sensor.
it is scenario 06_SensorNoise. The simulator generates two files with GPS and IMU measurements.


### Step 2: Attitude Estimation ###

Now let's look at the first step to our state estimation: including information from our IMU.  In this step, we are improving the complementary filter-type attitude filter with a better rate gyro attitude integration scheme.

1. Run scenario `07_AttitudeEstimation`.  For this simulation, the only sensor used is the IMU and noise levels are set to 0 (see `config/07_AttitudeEstimation.txt` for all the settings for this simulation).  There are two plots visible in this simulation.
   - The top graph is showing errors in each of the estimated Euler angles.
   - The bottom shows the true Euler angles and the estimates.
Observe that there’s quite a bit of error in attitude estimation.

2. In `QuadEstimatorEKF.cpp`, the function `UpdateFromIMU()` contains a complementary filter-type attitude filter.  To reduce the errors in the estimated attitude (Euler Angles), we need to do is to integrate pqr from the gyroscope into the estimated pitch and roll. It should be able to reduce the attitude errors to get within 0.1 rad for each of the Euler angles, as shown in the screenshot below.

![attitude example](images/attitude-screenshot.png)

In the screenshot above the attitude estimation using linear scheme (left) and using the improved nonlinear scheme (right). Note that Y axis on error is much greater on left.


### Step 3: Prediction Step ###

In this next step you will be implementing the prediction step of your filter.


1. Run scenario `08_PredictState`.  This scenario is configured to use a perfect IMU (only an IMU). Due to the sensitivity of double-integration to attitude errors, we've made the accelerometer update very insignificant (`QuadEstimatorEKF.attitudeTau = 100`).  The plots on this simulation show element of your estimated state and that of the true state.  At the moment we see that your estimated state does not follow the true state.

2. In `QuadEstimatorEKF.cpp`, implemented the state prediction step in the `PredictState()` functon. Then run the scenario `08_PredictState` we see the estimator state track the actual state, with only reasonably slow drift, as shown in the figure below:

![predict drift](images/predict-slow-drift.png)

3. Now let's introduce a realistic IMU, one with noise.  Run scenario `09_PredictionCov`. We see a small fleet of quadcopter all using your prediction code to integrate forward. Below two plots:
   - The top graph shows 10 (prediction-only) position X estimates
   - The bottom graph shows 10 (prediction-only) velocity estimates
notice however that the estimated covariance (white bounds) currently do not capture the growing errors.

4. In `QuadEstimatorEKF.cpp`, calculate the partial derivative of the body-to-global rotation matrix in the function `GetRbgPrime()`.  Once you have that function implement, implement the rest of the prediction step (predict the state covariance forward) in `Predict()`.

5. Run your covariance prediction and tune the `QPosXYStd` and the `QVelXYStd` process parameters in `QuadEstimatorEKF.txt` to try to capture the magnitude of the error you see. Note that as error grows our simplified model will not capture the real error dynamics (for example, specifically, coming from attitude errors), therefore  try to make it look reasonable only for a relatively short prediction period (the scenario is set for one second).  A good solution looks as follows:

![good covariance](images/predict-good-cov.png)

Looking at this result, you can see that in the first part of the plot, our covariance (the white line) grows very much like the data.

If we look at an example with a `QPosXYStd` that is much too high (shown below), we can see that the covariance no longer grows in the same way as the data.

![bad x covariance](images/bad-x-sigma.PNG)

Another set of bad examples is shown below for having a `QVelXYStd` too large (first) and too small (second).  As you can see, once again, our covariances in these cases no longer model the data well.

![bad vx cov large](images/bad-vx-sigma.PNG)

![bad vx cov small](images/bad-vx-sigma-low.PNG)


### Step 4: Magnetometer Update ###

Up until now we've only used the accelerometer and gyro for our state estimation.  In this step, We will be adding the information from the magnetometer to improve the filter's performance in estimating the vehicle's heading.

1. Run scenario `10_MagUpdate`.  This scenario uses a realistic IMU, but the magnetometer update hasn’t been implemented yet. As a result, you will notice that the estimate yaw is drifting away from the real value (and the estimated standard deviation is also increasing).  Note that in this case the plot is showing you the estimated yaw error (`quad.est.e.yaw`), which is drifting away from zero as the simulation runs.  You should also see the estimated standard deviation of that state (white boundary) is also increasing.

2. Tune the parameter `QYawStd` (`QuadEstimatorEKF.txt`) for the QuadEstimatorEKF so that it approximately captures the magnitude of the drift, as demonstrated here:

![mag drift](images/mag-drift.png)

3. Lets Implement the magnetometer update in the function `UpdateFromMag()`.  now take a loot at resulting plot similar to this one:

![mag good](images/mag-good-solution.png)


### Step 5: Closed Loop + GPS Update ###

1. Run scenario `11_GPSUpdate`.  At the moment this scenario is using both an ideal estimator and and ideal IMU.  Even with these ideal elements, watch the position and velocity errors (bottom right). As we see they are drifting away, since GPS update is not yet implemented.

2. Let's change to using your estimator by setting `Quad.UseIdealEstimator` to 0 in `config/11_GPSUpdate.txt`.  Rerun the scenario to get an idea of how well your estimator work with an ideal IMU.

3. Now repeat with realistic IMU by commenting out these lines in `config/11_GPSUpdate.txt`:
```
#SimIMU.AccelStd = 0,0,0
#SimIMU.GyroStd = 0,0,0
```

4. Tune the process noise model in `QuadEstimatorEKF.txt` to try to approximately capture the error you see with the estimated uncertainty (standard deviation) of the filter.

5. To implement the EKF GPS Update in the function `UpdateFromGPS()`, we need to use the equations from section 7.3.1 GPS from the Estimation for Quadrotors paper.

6. The objective is to complete the entire simulation cycle with estimated position error of < 1m.


### Step 6: Adding Your Controller ###

Up to this point, we have been working with a controller that has been relaxed to work with an estimated state instead of a real state.  So now, you will see how well your controller performs and de-tune your controller accordingly.

1. Replaced `QuadController.cpp` with the controller we implemented from the last project.

2. Replaced `QuadControlParams.txt` with the control parameters from the last project.

3. Then Run scenario `11_GPSUpdate`. Decrease the position and velocity gains (we’ve seen about 30% detuning being effective) to stabilize it. goal is to once again complete the entire simulation cycle with an estimated position error of < 1m.



### Writeup: ###


#### Implement Estimator ####

Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.

The standard deviation calculation was done with this notebook Step 1 Sensor Noise using numpy np.std function.

#### Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function ####

The implementation of this integration is at src/QuadEstimatorEKF.cpp from line 74 to line 129.

#### Implement all of the elements of the prediction step for the estimator ####

The different elements of the predict step are implemented at different function:

PredictState function: src/QuadEstimatorEKF.cpp line 154 to line 191.
GetRbgPrime function: src/QuadEstimatorEKF.cpp line 193 to line 231.
Predict function: src/QuadEstimatorEKF.cpp line 233 to line 285.

#### Implement the GPS update ####

The GPS update was implemented at src/QuadEstimatorEKF.cpp line 287 to line 319.
