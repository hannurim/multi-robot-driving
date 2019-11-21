# multi-robot-driving
Operates turtlebot3 in ROS

Consists of leader-follower form.
Two followers follow one leader.

    Robot1 = Leader
    Robot2,3 = Follower
    
The leader indexes two robots, tracks their location, and publishes them coninuously.
The follower robot receives the data published by the leader and calculates the target point using the rotation matrix.
Tracking uses association.
Update to the nearest value within 0.3m radius of the existing value.
Reduced noise effects by using smoothing filter.

The target point is ±30cm from the position of the leader robot on the X axis and -10cm the Y axis.
The follower robots will follow the target 20cm away.
