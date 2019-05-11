# Team Rocket code

--------------------

## Control System (Arduino)
*Juan R. Robles Gómez*

### Launch system
Arduino code is ready to control the launch. We have developed a countdown function that will indicate the countdown status using two leds. When the countdown finishes, the solenoid valve will open and the rasing stage begins.

The launch system have been simulated on Proteus.

The electronic valve will not be implemented as we explain on the report. The pressure drop and the weight increment are too important for the final results. How ever, the countdown function is still available, so we have to launch the rocket manually at the same time the valve would open.

### Vertical flight
During the vertical flight, flaperons will stay at zero degrees to avoid any lift.
Our first idea was to use a IMU to detect the end of the raising stage and the roll and pitch angle. Finally we rejected the idea because it was difficult to obtain velocities using the IMU. Integrations will accumulate many errors and it will be impossible to detect the zero velocity instant. All the code to use the IMU had been developed, even a calibration and a debug function, but it will not be used in Patras.
The raising time have been calculated using the Maximim altitude code that we have developed. It will be adjusted during the competition in Patras.

### Horizontal flight
During the horizontal flight, the flaperon angle will be set to 5 degrees to acheieve the maximum lift as possible.

--------------------

## Maximum altitude
*Miriam García Medina*

To choose the measures of the propellant tank, we developed a python code to calculate the maximum altitude for every water bottle we found. 
