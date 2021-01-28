## Estimation Project Writeup

This file provides detailed descriptions of the solutions to all of the steps needed to complete the project, outlined in the readMe.

### Step 1: 
- After running scenario six a couple of times, I closed the simulation and opened the logs for graph 1 for GPS, and graph 2 for the accelerometer. 
- I then copyed the data from both graph.txt files and loaded them into a jupyter notebook. There I seperated out the GPS.X, and IMU.AX data from their respective times. 
- With the data seperated, I was able to use the following equation to solve for the standard deviations:


<a href="https://www.codecogs.com/eqnedit.php?latex=\sigma&space;=&space;\sqrt{\frac{\sum_{i=1}^{n}\left&space;(&space;x_{i}&space;-&space;\bar{x}&space;\right&space;)^{2}}{n-1}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\sigma&space;=&space;\sqrt{\frac{\sum_{i=1}^{n}\left&space;(&space;x_{i}&space;-&space;\bar{x}&space;\right&space;)^{2}}{n-1}}" title="\sigma = \sqrt{\frac{\sum_{i=1}^{n}\left ( x_{i} - \bar{x} \right )^{2}}{n-1}}" /></a>
- With a mean of zero, the code was made simpler to: 
```python 
np.sqrt(np.sum(x**2)/(len(x)-1)) 
```
- This resulted in standard deviations of .65 for the GPS, and .48 for the accelerometer.

### Step 2:

- To implement this step I used the Quaternion class from the quaternion.h file. I first created a quaternion named qt. 
```cpp
Quaternion<float> qt;
```
- Then I used the Quaternion class built in function FromEuler123_RPY and passed in the roll, pitch and yaw estimates to convert them into a quaternion.
```cpp
qt = qt.FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
```
- FromEuler123_RPY uses the following function numbered 84 from James Diebel's, Representing Attitude paper [1] for converting euler angles into quaternions. With c and s 
representing cosine and sine.


  <a href="https://www.codecogs.com/eqnedit.php?latex=q_{123}\left&space;(&space;\phi&space;\theta&space;\Psi&space;\right&space;)&space;=&space;\begin{bmatrix}&space;c_\phi/2&space;c_\theta/2&space;c_\psi/2&space;&plus;&space;s_\phi/2&space;s_\theta/2&space;s_\psi/2\\&space;-c_\phi/2&space;s_\theta/2&space;s_\psi/2&space;&plus;&space;c_\theta/2&space;c_\psi/2&space;s_\phi/2\\&space;c_\phi/2&space;c_\psi/2&space;s_\theta/2&space;&plus;&space;s_\phi/2&space;c_\theta/2&space;s_\psi/2\\&space;c_\phi/2&space;c_\theta/2&space;s_\psi/2&space;-&space;s_\phi/2&space;c_\psi/2&space;s_\theta/2&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?q_{123}\left&space;(&space;\phi&space;\theta&space;\Psi&space;\right&space;)&space;=&space;\begin{bmatrix}&space;c_\phi/2&space;c_\theta/2&space;c_\psi/2&space;&plus;&space;s_\phi/2&space;s_\theta/2&space;s_\psi/2\\&space;-c_\phi/2&space;s_\theta/2&space;s_\psi/2&space;&plus;&space;c_\theta/2&space;c_\psi/2&space;s_\phi/2\\&space;c_\phi/2&space;c_\psi/2&space;s_\theta/2&space;&plus;&space;s_\phi/2&space;c_\theta/2&space;s_\psi/2\\&space;c_\phi/2&space;c_\theta/2&space;s_\psi/2&space;-&space;s_\phi/2&space;c_\psi/2&space;s_\theta/2&space;\end{bmatrix}" title="q_{123}\left ( \phi \theta \Psi \right ) = \begin{bmatrix} c_\phi/2 c_\theta/2 c_\psi/2 + s_\phi/2 s_\theta/2 s_\psi/2\\ -c_\phi/2 s_\theta/2 s_\psi/2 + c_\theta/2 c_\psi/2 s_\phi/2\\ c_\phi/2 c_\psi/2 s_\theta/2 + s_\phi/2 c_\theta/2 s_\psi/2\\ c_\phi/2 c_\theta/2 s_\psi/2 - s_\phi/2 c_\psi/2 s_\theta/2 \end{bmatrix}" /></a>

- Computed in code as:
```cpp
    roll /= 2.f;
    pitch /= 2.f;
    yaw /= 2.f;
  
    T q0 = cos(roll) * cos(pitch) * cos(yaw)   + sin(roll)  * sin(pitch) * sin(yaw);
    T q1 =-cos(roll) * sin(pitch) * sin(yaw)   + cos(pitch) * cos(yaw)   * sin(roll);
    T q2 = cos(roll) * cos(yaw)   * sin(pitch) + sin(roll)  * cos(pitch) * sin(yaw);
    T q3 = cos(roll) * cos(pitch) * sin(yaw)   - sin(roll)  * cos(yaw)   * sin(pitch);
```
- With the given current estimate converted into a quaternion, I was able to use the Quaternion class's IntegrateBodyRate funciton to integrate the body rate from the gyro sensor.
```cpp
inline Quaternion IntegrateBodyRate(const V3D pqr, const double dt) //body rates must be expressed in the body coordinate frame!
  {
    *this = Quaternion::FromAxisAngle(pqr*dt)*(*this);
    return *this;
  };
```
- As you can see this function uses another class function, FromAxisAngle while incorporating the time step in the input. It also uses the overloaded * operator. Diebel [1] defines this conversion in equation 175 as: 


<a href="https://www.codecogs.com/eqnedit.php?latex=q_{\alpha}\left&space;(&space;\alpha,&space;n&space;\right&space;)&space;:=&space;\begin{bmatrix}&space;cos\left&space;(&space;\frac{1}{2}\alpha&space;\right&space;)\\&space;n&space;sin\left&space;(&space;\frac{1}{2}\alpha&space;\right&space;)&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?q_{\alpha}\left&space;(&space;\alpha,&space;n&space;\right&space;)&space;:=&space;\begin{bmatrix}&space;cos\left&space;(&space;\frac{1}{2}\alpha&space;\right&space;)\\&space;n&space;sin\left&space;(&space;\frac{1}{2}\alpha&space;\right&space;)&space;\end{bmatrix}" title="q_{\alpha}\left ( \alpha, n \right ) := \begin{bmatrix} cos\left ( \frac{1}{2}\alpha \right )\\ n sin\left ( \frac{1}{2}\alpha \right ) \end{bmatrix}" /></a>
- Where alpha is the magnitude of the angle and n is the unit vector. In the code below, the magnitude, here named theta, is calculated. Then the formula above is 
followed with the input, pqr*dt, being converted into a unit vector by dividing each component by the magnitude, theta (alpha above), using the following formula:


  <a href="https://www.codecogs.com/eqnedit.php?latex=u&space;=&space;\frac{v}{\left&space;\|&space;v&space;\right&space;\|}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?u&space;=&space;\frac{v}{\left&space;\|&space;v&space;\right&space;\|}" title="u = \frac{v}{\left \| v \right \|}" /></a>
  
- The code for FromAxisAngle is as follows:
```cpp
static Quaternion FromAxisAngle(const V3D axisAngle)
  {
    const T theta = (T)axisAngle.mag();
    if(!theta) return Identity();

    Quaternion qout;
    qout[0] = cos(theta/2.f);
    qout[1] = sin(theta/2.f)*(T)axisAngle.x/theta;
    qout[2] = sin(theta/2.f)*(T)axisAngle.y/theta;
    qout[3] = sin(theta/2.f)*(T)axisAngle.z/theta;
    return qout;
  };  
``` 

- Diebel [1] defines quaternion multiplication to be not communative in this formula:


<a href="https://www.codecogs.com/eqnedit.php?latex=q&space;\cdot&space;p&space;=&space;\begin{bmatrix}&space;q_0p_0&space;-&space;q_{1:3}^{T}p_{1:3}\\&space;q_0p_{1:3}&space;&plus;&space;p_0q_{1:3}&space;-&space;q_{1:3}p_{1:3}&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?q&space;\cdot&space;p&space;=&space;\begin{bmatrix}&space;q_0p_0&space;-&space;q_{1:3}^{T}p_{1:3}\\&space;q_0p_{1:3}&space;&plus;&space;p_0q_{1:3}&space;-&space;q_{1:3}p_{1:3}&space;\end{bmatrix}" title="q \cdot p = \begin{bmatrix} q_0p_0 - q_{1:3}^{T}p_{1:3}\\ q_0p_{1:3} + p_0q_{1:3} - q_{1:3}p_{1:3} \end{bmatrix}" /></a>

- The overloaded * operator in code is:
```cpp
// quaternion multiplication: q*p <- this corresponds to a rotation p followed by rotation q?
  Quaternion operator*(const Quaternion& p) const
  {
    const T* q = _q; // for convenience
    T c0 = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3]; // 0 -1 -2 -3
    T c1 = q[1]*p[0] + q[0]*p[1] + q[3]*p[2] - q[2]*p[3]; // 1  0  3 -2
    T c2 = q[2]*p[0] - q[3]*p[1] + q[0]*p[2] + q[1]*p[3]; // 2 -3  0  1
    T c3 = q[3]*p[0] + q[2]*p[1] - q[1]*p[2] + q[0]*p[3]; // 3  2 -1  0
    return Quaternion(c0,c1,c2,c3);
  }
```
- I then converted the integrated quaternion back into euler angle representations using the Quaternion class functions Roll, Pitch, and Yaw, and placing them in the variables 
predictedRoll, predictedPitch, and the QuadEstimatorEKF class variable ekfState(6).
- The formula for converting the quaternion into a euler representation from Diebel's equation 290 [1] is: 


<a href="https://www.codecogs.com/eqnedit.php?latex=u_{123}\left&space;(&space;R_q\left&space;(&space;q&space;\right&space;)&space;\right&space;)&space;=&space;\begin{bmatrix}&space;atan2\left&space;(&space;2q_2q_3&space;&plus;&space;2q_0q_1,&space;q_3^2&space;-&space;q_2^2&space;-&space;q_1^2&space;&plus;&space;q_0^2&space;\right&space;)\\&space;-asin(2q_1q_3&space;-&space;2q_0q_2)&space;\\&space;atan2(2q_1q_2&space;&plus;&space;2q_0q_3,&space;q_1^2&space;&plus;&space;q_0^2&space;-&space;q_3^2&space;-&space;q_2^2)&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?u_{123}\left&space;(&space;R_q\left&space;(&space;q&space;\right&space;)&space;\right&space;)&space;=&space;\begin{bmatrix}&space;atan2\left&space;(&space;2q_2q_3&space;&plus;&space;2q_0q_1,&space;q_3^2&space;-&space;q_2^2&space;-&space;q_1^2&space;&plus;&space;q_0^2&space;\right&space;)\\&space;-asin(2q_1q_3&space;-&space;2q_0q_2)&space;\\&space;atan2(2q_1q_2&space;&plus;&space;2q_0q_3,&space;q_1^2&space;&plus;&space;q_0^2&space;-&space;q_3^2&space;-&space;q_2^2)&space;\end{bmatrix}" title="u_{123}\left ( R_q\left ( q \right ) \right ) = \begin{bmatrix} atan2\left ( 2q_2q_3 + 2q_0q_1, q_3^2 - q_2^2 - q_1^2 + q_0^2 \right )\\ -asin(2q_1q_3 - 2q_0q_2) \\ atan2(2q_1q_2 + 2q_0q_3, q_1^2 + q_0^2 - q_3^2 - q_2^2) \end{bmatrix}" /></a>

- The code for the Quaternion class functions Roll, Pitch, and Yaw are implented using the above formula as:

```cpp
float Yaw() const
  {
    const T div1 = _q[0] * _q[0] + _q[1] * _q[1] - _q[2] * _q[2] - _q[3] * _q[3];
    return atan2(2 * (_q[1] * _q[2] + _q[0] * _q[3]), div1);
  }

  float Pitch() const
  {
    return asin(-2 * (_q[1] * _q[3] - _q[0] * _q[2]));
  }

  float Roll() const
  {
    const T div2 = _q[0] * _q[0] - _q[1] * _q[1] - _q[2] * _q[2] + _q[3] * _q[3];
    return atan2(2 * (_q[2] * _q[3] + _q[0] * _q[1]), div2);
  }
  ```
  
  ### Step 3
  
- For the PredictState function, I started by creating a V3F named aI and using the given quaternion of the estimated attitude, I used its class function, Rotate_BtoI, to rotate the input accelerometer reading from the body frame into the world (or inertial) frame. 
  ```cpp
  Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, curState(6));
  V3F aI = attitude.Rotate_BtoI(accel);
  ```
  ```cpp
  V3F Rotate_BtoI(const V3F& in) const
	{
		float R[9];
		RotationMatrix_IwrtB(R);

		V3F ret(R[0]*in[0] + R[1]*in[1] + R[2]*in[2],
			      R[3]*in[0] + R[4]*in[1] + R[5]*in[2],
						R[6]*in[0] + R[7]*in[1] + R[8]*in[2]);

		return ret;
	}
  ```
- As you can see above the Rotate_BtoI function first creates a rotation matrix with another class function, RotationMatrix_IwrtB. It then multiplys the V3F input, here the accelerometer readings, by the newly created rotation matrix. Equation 127 from Diebel [1], represents the code above, with Rq(q) being the rotation matrix (equation 125 in Diebel [1]), z' being the vector in the body frame, and of course z being the vector in the world frame.
  
  <a href="https://www.codecogs.com/eqnedit.php?latex=z&space;=&space;R_q(r)^T\acute{z}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?z&space;=&space;R_q(r)^T\acute{z}" title="z = R_q(r)^T\acute{z}" /></a>
  
- Due to the very short time step, I implemented a very simple integration method, by multiplying the current state's velocities by the time step and adding it to the positions. Then multiplying the accelerations, already rotated to the world frame, by the time step and adding them to the velocities. For the velocity in the z axis, I also subtracted the CONST_GRAVITY multplied by the time step.

```cpp
  predictedState(0) += predictedState(3)*dt;
  predictedState(1) += predictedState(4)*dt;
  predictedState(2) += predictedState(5)*dt;
  predictedState(3) += aI.x*dt;
  predictedState(4) += aI.y*dt;
  predictedState(5) += - CONST_GRAVITY*dt + aI.z*dt;
```

- The above method is best described from equation 49, from Tellex's estimation paper [2]. With <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;u_t&space;\Delta&space;t" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;u_t&space;\Delta&space;t" title="u_t \Delta t" /></a>
as the input accelerations, having already been rotated to the world frame above. With psi remaining the same, there was no need to include it in these calculations.

<a href="https://www.codecogs.com/eqnedit.php?latex=g(x_t,&space;u_t,&space;\Delta&space;t)&space;=&space;\begin{bmatrix}&space;x_{t,x}&space;&plus;&space;x_{t,\dot{x}}\Delta&space;t\\&space;x_{t,y}&space;&plus;&space;x_{t,\dot{y}}\Delta&space;t&space;\\&space;x_{t,z}&space;&plus;&space;x_{t,\dot{z}}\Delta&space;t&space;\\&space;x_{t,\dot{x}}\\&space;x_{t,\dot{y}}&space;\\&space;x_{t,\dot{z}}&space;\\&space;x_{t,&space;\Psi&space;}&space;\end{bmatrix}&space;&plus;&space;\begin{bmatrix}&space;0&space;&&space;0&space;&&space;0&space;&&space;0\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0\\&space;R_{bg}[0:]&space;&&space;&&space;&&space;0\\&space;R_{bg}[1:]&space;&&space;&&space;&&space;0\\&space;R_{bg}[2:]&space;&&space;&&space;&&space;0\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}&space;u_t&space;\Delta&space;t" target="_blank"><img src="https://latex.codecogs.com/gif.latex?g(x_t,&space;u_t,&space;\Delta&space;t)&space;=&space;\begin{bmatrix}&space;x_{t,x}&space;&plus;&space;x_{t,\dot{x}}\Delta&space;t\\&space;x_{t,y}&space;&plus;&space;x_{t,\dot{y}}\Delta&space;t&space;\\&space;x_{t,z}&space;&plus;&space;x_{t,\dot{z}}\Delta&space;t&space;\\&space;x_{t,\dot{x}}\\&space;x_{t,\dot{y}}&space;\\&space;x_{t,\dot{z}}&space;\\&space;x_{t,&space;\Psi&space;}&space;\end{bmatrix}&space;&plus;&space;\begin{bmatrix}&space;0&space;&&space;0&space;&&space;0&space;&&space;0\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0\\&space;R_{bg}[0:]&space;&&space;&&space;&&space;0\\&space;R_{bg}[1:]&space;&&space;&&space;&&space;0\\&space;R_{bg}[2:]&space;&&space;&&space;&&space;0\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}&space;u_t&space;\Delta&space;t" title="g(x_t, u_t, \Delta t) = \begin{bmatrix} x_{t,x} + x_{t,\dot{x}}\Delta t\\ x_{t,y} + x_{t,\dot{y}}\Delta t \\ x_{t,z} + x_{t,\dot{z}}\Delta t \\ x_{t,\dot{x}}\\ x_{t,\dot{y}} \\ x_{t,\dot{z}} \\ x_{t, \Psi } \end{bmatrix} + \begin{bmatrix} 0 & 0 & 0 & 0\\ 0 & 0 & 0 & 0\\ 0 & 0 & 0 & 0\\ R_{bg}[0:] & & & 0\\ R_{bg}[1:] & & & 0\\ R_{bg}[2:] & & & 0\\ 0 & 0 & 0 & 1 \end{bmatrix} u_t \Delta t" /></a>




- Moving onto the function GetRbgPrime, I first created phi, theta, and psi variables, with roll, pitch, and yaw passed into them to make it easier to copy equation 71 from Diebel [1]. I then used the comma initializer from the Eigen library and passed in all the coefficients from Diebels equation.

```cpp
float phi = roll;
float theta = pitch;
float psi = yaw;

RbgPrime << -cos(theta)*sin(psi), -sin(phi)*sin(theta)*sin(psi) - cos(phi)*cos(psi), -cos(phi)*sin(theta)*sin(psi) + sin(phi)*cos(psi),
            cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi),
            0.0, 0.0, 0.0;
```

For the Predict function I first initialized ut as a MatrixXf from the Eigen library, and used the comma initializer to pass in the accelerometer's readings. I again used the comma initializer to pass in the jacobian of the above <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;g(x_t,&space;u_t,&space;\Delta_t)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;g(x_t,&space;u_t,&space;\Delta_t)" title="g(x_t, u_t, \Delta_t)" /></a> as defined in equation 51 from Tellex's estimation paper [2]. From there I followed the extended kalman filter algorithm on page 3 of Tellex's estimation paper [2], written below, to compute the predicted state variance.



<a href="https://www.codecogs.com/eqnedit.php?latex=\bar{\Sigma}&space;=&space;G_t&space;\Sigma_{t-1}G_{t}^{T}&space;&plus;&space;Q_t" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\bar{\Sigma}&space;=&space;G_t&space;\Sigma_{t-1}G_{t}^{T}&space;&plus;&space;Q_t" title="\bar{\Sigma} = G_t \Sigma_{t-1}G_{t}^{T} + Q_t" /></a>



In code as follows:

```cpp
MatrixXf ut(3, 1);

  ut << accel.x,
        accel.y,
        accel.z;



  gPrime << 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, (RbgPrime.row(0) * ut) * dt,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, (RbgPrime.row(1) * ut) * dt,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, (RbgPrime.row(2) * ut) * dt,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;



  ekfCov << gPrime * ekfCov * gPrime.transpose() + Q;
  ```

- I then tuned QPosXYStd and QVelXYStd in the QuadEstimatorEKF.txt file as .05 and .1.

### Step 4

- In the file QuadEstimatorEKF.txt I tuned the QYawStd to .1 to approximately capture the magnitude of the drift I observed when running scenario 10.

- For the UpdateFromMag function I first used the comma initializer from the Eigen library to pass in the values for hPrime as it is in equation 58 from Tellex's estimation paper [2]. I then passed in the current estimated yaw to the zFromX vector. Then I created the float y, to pass in the difference of the measured and estimated yaw. From there I normalised the difference to keep the movements as short as possible when this residual is computed in the call to Update with the following equation from the EKF algorithm on page 3 of Tellex's estimation paper [2]. <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\mu_t&space;=&space;\bar{\mu}_t&space;&plus;&space;K_t(z_t&space;-&space;h(\bar{\mu_t}))" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\mu_t&space;=&space;\bar{\mu}_t&space;&plus;&space;K_t(z_t&space;-&space;h(\bar{\mu_t}))" title="\mu_t = \bar{\mu}_t + K_t(z_t - h(\bar{\mu_t}))" /></a> This was normalised by preventing the residual from being greater than pi radians (180 degrees). If it were found to be greater than pi radians (180 degrees), 2 pi (360 degrees) was added to the estimated yaw, sending it in the opposite (counter-clockwise) direction. Conversely if the difference was found to be less than -pi (-180 degrees), then 2 pi (360 degrees) was subtracted from the estimated yaw, again sending it in the opposite (clockwise) direction.   

```cpp
hPrime << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
zFromX << ekfState(6);

float y = magYaw - ekfState(6);
if (y > F_PI){
    zFromX(0) += 2.0 * F_PI;
}
else if (y < -F_PI){
    zFromX(0) -= 2.0 * F_PI;
}
```
  
### Step 5
  
- In the file QuadEstimatorEKF.txt I tuned the QPosXYStd and QPosZStd both to .05 to approximately capture the magnitude of the drift I observed when running scenario 11. I also tuned the velocities for x, y, and z to .1, for the drift I observed there.

- I again used the Eigen library's comma initializer to pass in the values needed for hPrime as defined in equation 55 in Tellex's estimation paper [2]. I also used the Eigen library's comma initializer to pass in the values needed for zFromX as defined in equation 54 in Tellex's estimation paper [2]. 

```cpp
hPrime <<  1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;

zFromX << ekfState(0), ekfState(1), ekfState(2), ekfState(3), ekfState(4), ekfState(5);
```

### Step 6

- In this step I simply replaced the given controller with the controller I created in the controls project earlier. I did the same for the QuadControlParams.txt file. Even though it still passed all of the scenarios, the path flown was pretty wonky so I updated the paramaters as follows. 
	- kpPosXY = 3
	- kpPosZ = 2
  	- kpVelXY = 9
	- kpBank = 12
	- kpYaw = 9
  
## References

[1] James Diebel.  Representing attitude:  Euler angles, unit quaternions, and rotation vectors.Matrix, 58(15-16):1–35, 2006.

[2] Stefanie Tellex, Andy Brown, and Sergei Lupashin. Estimation for Quadrotors, January 27, 2021.
