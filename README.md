# STATE ESTIMATION
* The .pickle file possesses the LiDAR readings of an environment and the information about the controls of the bot at corresponding time-stamps.
* Aim is to get trajectory and orientation of bot using the data.pickle file.
* An Extended Kalman Filter is used to execute the same.
## Working Of EKF
<img src="https://user-images.githubusercontent.com/64797216/124354872-eba38a00-dc2b-11eb-9411-8aa871f09fa5.png" width="600">
Given the initial pose of the bot, there are two major steps in EKF algorithm.  

* **PREDICTION STEP**   
>The mean and covariance is calculated in the prediction step where mean indicates that the position of the robot is most likely at that point and the covariance is the uncertainity or the error in the estimation.  
>Mean is calculated on the basis of the previous position and controls feeded and covariance by the error in the controls input and previous covariance.

* **CORRECTION STEP**  
>Calculated mean and covariance are updated in this step through the sensor measurements.    
>Kalman Gain can be considered as the weighted mean which signifies how certain the calculated sensor measurements are compared to the actual sensor measurements.  
>Finally the updated mean and covariance are then feeded back into prediction step for calculation of the next pose of bot to complete the loop.

