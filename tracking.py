#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class TrackBB8():
    def __init__(self, position, resolution):
        # initiliaze
        # rospy.init_node('TrackBB8', anonymous=False)

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);
        
        # Class attributes
        self.position = position
        self.resolution = resolution
        self.track()

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
        # publish the velocity
            self.cmd_vel.publish(move_cmd)
        # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
        
    def track(self):
        # Twist is a datatype for velocity
        move_cmd = Twist()

        """
        STATE MACHINE
        1. Check if column/X-axis is ~at midpoint. If yes - no movement.
        2. If column > midpoint of frame, turn right. Angular velocity +/-?
        3. If column < midpoint of frame, turn left. Angular velocity +/-?
        4. For now, don't worry about linear velocity (row value)
        5. Repeat
        Look into rate of checking - will check @ 10 Hz.
        Turn a little bit then check again - repeat
        Same with moving forward/backward """

        mid_row = self.resolution[0]//2 - 1
        mid_col = self.resolution[1]//2 - 1

        # State machine
        if self.position[1] == mid_col:
            # turn at 0 radians/s
            move_cmd.angular.z = 0.2
        else:
            move_cmd.angular.z = 0

        # let's go forward at 0.2 m/s
        move_cmd.linear.x = 0
        pass
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stopping TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        TrackBB8()
    except:
        rospy.loginfo("TrackBB8 node terminated.")