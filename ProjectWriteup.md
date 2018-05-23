# The C++ Project Writeup #

This is the writeup for the C++ Estimation project.



Below are the notes on each of the required sections for this project.

## Sensor Noise ##

For this part of the project I ran the Sensor Noise Scenario, until just before the reset. 
Then I took the data in config/log/Graph1.txt (GPS X data) and config/log/Graph2.txt (Accelerometer X data) and pasted in Excel.
Then I looked up instrucions on how to calculate the Standar Deviation. Below is a screenshoot of part of my Excel spreasheet.

MeasuredStdDev_GPSPosX calulation:

```
	9.605251	0.090992	0.114107253	0.013020465
	9.705292	-0.034533	-0.011417747	0.000130365
	9.805332	-0.507209	-0.484093747	0.234346756
	9.905373	0.853465	0.876580253	0.768392939
Sum		-2.28841	0	50.44829621
Count		99	99	99
AverageMean)		-0.023115253		
Variance				0.514778533
Stnd Dev				0.717480685
```

MeasuredStdDev_AccelXY calulation:

```
	6.694697	-0.222196	-0.199983	0.0399932
	6.699697	0.708848	0.731061	0.534450186
	6.704697	0.916377	0.93859	0.880951188
Sum		-2.932376	26.855257	351.9767634
Count		1341	1341	1341
AverageMean)		-0.002186708		
Variance				0.262669226
Stnd Dev				0.51251266
```
å
Then I updated the values in 06_SensorNoise.txt

MeasuredStdDev_GPSPosXY = .71
MeasuredStdDev_AccelXY = .51

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