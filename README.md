# STATE ESTIMATION
* The .pickle file possesses the LiDAR readings of an environment and the information about the controls of the bot at corresponding time-stamps.
* Aim is to get trajectory and orientation of bot using the data.pickle file.
* An Extended Kalman Filter is used to execute the same.
## Working Of EKF
![image](https://user-images.githubusercontent.com/64797216/124354872-eba38a00-dc2b-11eb-9411-8aa871f09fa5.png)
Given the initial pose of the bot, there are two major steps in EKF algorithm.
