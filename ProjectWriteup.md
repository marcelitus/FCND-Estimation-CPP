# The C++ Project Writeup #

This is the writeup for the C++ Estimation project.



Below are the notes on each of the required sections for this project.

## UpdateFrom IMU ##

For this part I decided to use the Quaternion method, whcih seemed simpler to me.
I used the FromEulerRPY function to assign the quaternion from rollEst, pitchEst and ekfState(6)).
Then I integrated it with IntegrateBodyRate(gyro, dtIMU) to get the body rates.
From then I converted back to Euler and assigned to a vector, form wich each component was assigned to the corresponding values.

```c++

 Quaternion<float> estAttitude;

  estAttitude = estAttitude.FromEuler123_RPY(rollEst, pitchEst, ekfState(6));

  estAttitude.IntegrateBodyRate(gyro, dtIMU);
  V3D IMUValues = estAttitude.ToEulerRPY();
  float predictedPitch = IMUValues.y;
  float predictedRoll = IMUValues.x;
  ekfState(6) = IMUValues.z;

```

### PredictState ###

This is a fairly simple part, I used the hint to calculate to rotate the body rate to the inertial frame. And then I calulated each predicted state using the position and accelerations accordingly. I made sure to substract the gravity constant from the Z acceleration.


```c++

V3F inertialAcceleration = attitude.Rotate_BtoI(accel);
 //  V3F bodyPossition = attitude.Rotate_BtoI(accel);
   predictedState(0) = predictedState(0) + predictedState(3) * dt;
   predictedState(1) = predictedState(1) + predictedState(4) * dt;
   predictedState(2) = predictedState(2) + predictedState(5) * dt;
   predictedState(3) = predictedState(3) + inertialAcceleration.x * dt;
   predictedState(4) = predictedState(4) + inertialAcceleration.y * dt;
   predictedState(5) = predictedState(5) + (inertialAcceleration.z - CONST_GRAVITY) * dt;

```

### GetRbgPrime ###

This was a fairly easy function following the information in this document and making sure that the correct angles and sin cos combinations where filled out.

https://www.overleaf.com/read/vymfngphcccj

### QuadEstimatorEKF::Predict ###

This was a somwewhat easy function following the information in this document and making sure that in the the component for Z I substracted CONST_GRAVITY as well.


```c++

  gPrime(0,0) = 1;
  gPrime(0,3) = dt;
  gPrime(1,1) = 1;
  gPrime(1,4) = dt;
  gPrime(2,2) = 1;
  gPrime(2,5) = dt;
  gPrime(3,3) = 1;
  gPrime(4, 4) = 1;
  gPrime(3, 6) = ((RbgPrime(0,0) * accel.x) + (RbgPrime(0,1) * accel.y) + (RbgPrime(0,2) * (accel.z - CONST_GRAVITY))) * dt;
//  gPrime(3, 6) = ((RbgPrime(0,0) * accel.x) + (RbgPrime(0,1) * accel.y) + (RbgPrime(0,2) * accel.z)) * dt;
  gPrime(5, 5) = 1;
  gPrime(4, 6) = ((RbgPrime(1,0) * accel.x) + (RbgPrime(1,1) * accel.y) + (RbgPrime(1,2) * (accel.z - CONST_GRAVITY))) * dt;
  //gPrime(4, 6) = ((RbgPrime(1,0) * accel.x) + (RbgPrime(1,1) * accel.y) + (RbgPrime(1,2) * accel.z)) * dt;
  gPrime(6, 6) = 1;
  gPrime(5, 6) = ((RbgPrime(2,0) * accel.x) + (RbgPrime(2,1) * accel.y) + (RbgPrime(2,2) * (accel.z - CONST_GRAVITY))) * dt;
//  gPrime(5, 6) = ((RbgPrime(2,0) * accel.x) + (RbgPrime(2,1) * accel.y) + (RbgPrime(2,2) * accel.z)) * dt;

  ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;

```

 ### UpdateFromGPS and UpdateFromMag ### 

These 2 functions are very straightforward using the information in the document below.

https://www.overleaf.com/read/vymfngphcccj
All scenarios pass in my program.