PATROLLER
===

__Robot patroller for robot navigation between landmarks__

![image](img/patroller.gif)

Set some landmarks so the robot will automatic navigate between those landmarks.

## Usage
``` bash
rosrun patroller patroller
# now you can using `Publish Point` to set some landmarks under RVIZ.

# to start patrol
rosservice call /StartPatrol

# to stop patrol
rosservice call /StopPatrol

# to reset patrol
rosservice call /ResetPatrol
```
