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
- FromEuler123_RPY uses the following function numbered 84 from James Diebel's, Representing Attitude paper [1] for converting euler angles into a quaternions. With c and s 
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
- With the given current estimate converted into a quaternion, I was able to use the inline IntegrateBodyRate funciton to integrate the body rate from the gyro sensor.
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
  
## References

[1] James Diebel.  Representing attitude:  Euler angles, unit quaternions, and rotation vectors.Matrix, 58(15-16):1–35, 2006.
