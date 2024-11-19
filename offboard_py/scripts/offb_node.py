"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from offboard_py.msg import DevKit

class drone_ros():
    def __init__(self):
        self.current_state = State()
        self.target_heading = PoseStamped() # Drone Target Heading from DevKit
        self.current_pos = PoseStamped()

        rospy.init_node("offb_node_py")

        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_callback)

        # Where target_heading data is published as PoseStamped (x, y, z)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = self.record_position)
        
        # Fake DevKit publisher, will be removed eventually
        self.heading_pub = rospy.Publisher("devkit/heading", DevKit, queue_size=10)

        # Where DevKit heading data is received from as DevKit type
        self.heading_sub = rospy.Subscriber("devkit/heading", DevKit, callback = self.convert_heading)
        
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.Rate(20)

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.current_state.connected):
            self.rate.sleep()

        self.set_initial_heading()
        
    def set_initial_heading(self):
        """
        This is an iniziation function for the sole purpose of making the 
        drone hover before getting readings and updating flight path. 
        """
    
        # Initially hover
        self.set_hover()
        
        # Send a few setpoints before starting
        for i in range(100):
            if(rospy.is_shutdown()):
                break

            self.local_pos_pub.publish(self.target_heading)
            self.rate.sleep()
        
        # Sets drone to OFFBOARD mode for autonomous control
        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'

        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

        self.last_req = rospy.Time.now()
        self.change_heading_interval = rospy.Time.now()
        self.change_heading = True

    def fake_heading_publish(self):
        """
        THIS IS A FAKE FUNCTION, built for the purpose of mimicking DevKit readings.
        This function will eventually be replaced.
        """

        # Send fake DevKit reading
        heading_dev = DevKit()
        heading_dev.range = 5
        heading_dev.direction = 2

        self.heading_pub.publish(heading_dev)
        rospy.loginfo("Published DevKit message")

    def state_callback(self, msg):
        """
        This figures out the MODE of the drone.
        """

        self.current_state = msg
    
    def new_heading(self, pose, x, y, z):
        """
        Shorcut to create a new PoseStamped heading.
        """

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        return pose
        
    def convert_heading(self, msg):
        """
        Function that takes in DevKit message and converts it to PoseStamped.
        """
        
        relative_pos = self.current_pos
        rospy.loginfo(relative_pos)
        
        (_, _, yaw) = euler_from_quaternion([relative_pos.pose.orientation.x, relative_pos.pose.orientation.y, relative_pos.pose.orientation.z, relative_pos.pose.orientation.w])

        converted_heading = PoseStamped()

        pieps_range = msg.range * 0.1
        pieps_direction_t = msg.direction

        # PIEPS uses -5 -> 5 to indicate range of rotation, this generalizes the numbers to a degree and either positive and negative
        relative_range = 0
        if abs(pieps_direction_t) == 1:
            relative_range = 0
        elif abs(pieps_direction_t) == 2:
            relative_range = (30 + 20) / 2
        elif abs(pieps_direction_t) == 3:
            relative_range = (40 + 30) / 2
        elif abs(pieps_direction_t) == 4:
            relative_range = (50 + 40) / 2
        elif abs(pieps_direction_t) == 5:
            relative_range = 50

        if pieps_direction_t < 0:
            relative_range = relative_range + -1
            
        # add initial rotation for calcuation
        relative_range += yaw
        rospy.loginfo(relative_range)

        converted_heading.pose.position.x = relative_pos.pose.position.x + pieps_range * math.cos(math.radians(relative_range))
        converted_heading.pose.position.y = relative_pos.pose.position.y + pieps_range * math.sin(math.radians(relative_range))
        converted_heading.pose.position.z = relative_pos.pose.position.z

        rospy.loginfo(converted_heading)

        self.target_heading = converted_heading

    def set_hover(self):
        self.target_heading = self.new_heading(self.target_heading, 0, 0, 2)

    def record_position(self, msg):
        self.current_pos = msg
        

if __name__ == "__main__":
    
    dr = drone_ros()

    while(not rospy.is_shutdown()):
        # Waits until OFFBOARD is enabled to continue
        if(dr.current_state.mode != "OFFBOARD" and (rospy.Time.now() - dr.last_req) > rospy.Duration(5.0)):
            if(dr.set_mode_client.call(dr.offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            dr.last_req = rospy.Time.now()
        else:
            # Waiting for the drone to finish arming to continue
            if(not dr.current_state.armed and (rospy.Time.now() - dr.last_req) > rospy.Duration(5.0)):
                if(dr.arming_client.call(dr.arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                dr.last_req = rospy.Time.now()

        # rospy.loginfo(dr.target_heading)
        dr.local_pos_pub.publish(dr.target_heading) # Sends current target heading to the drone
        
        # After a certain delay, we change the heading (not looped yet)
        if(rospy.Time.now() - dr.change_heading_interval > rospy.Duration(40.0) and dr.change_heading):
            dr.change_heading = False

            dr.fake_heading_publish()

        dr.rate.sleep()
