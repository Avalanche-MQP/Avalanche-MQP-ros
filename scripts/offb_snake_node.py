"""
 * File: offb_snake_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion
from mavros_msgs.msg import State, AvalancheBeacon
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

def unsigned_to_signed(d):
    bit_version = f'{d:08b}'

    v = int(bit_version, 2)

    if bit_version[0] =='1':
        v -= 2 ** len(bit_version)

    return v

class drone_ros():
    def __init__(self):
        self.current_state = State()
        self.target_heading = PoseStamped()
        self.current_pos = PoseStamped()

        rospy.init_node("offb_node_py")

        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_callback)

        # Where target_heading data is published as PoseStamped (x, y, z)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = self.record_position, queue_size=10)

        self.local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
  
        # Where DevKit heading data is received from as AvalancheBeacon type
        self.velocity_sub = rospy.Subscriber("mavros/avalanche_beacon", AvalancheBeacon, callback = self.check_signal, queue_size=10)
        
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.Rate(20)

        # Inizialize everything
        self.target_twist = TwistStamped()
        self.offb_set_mode = SetModeRequest()
        self.arm_cmd = CommandBoolRequest()
        self.last_req = rospy.Time.now()
        self.change_heading_interval = rospy.Time.now()
        self.change_heading = True
        self.new_message = False

        self.search_size = (10, 24) # (x,y) in meters
        self.search_range = 1 # in meter

        self.no_signal = True
        self.fly_to_right = True
        self.flying_up = False

        self.signal = False

        # Timestamp of last beacon msg
        self.beacon_ts = rospy.get_time()

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.current_state.connected):
            self.rate.sleep()
        
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

    def set_hover(self):
        self.target_heading = self.new_heading(self.target_heading, 0, 0, self.current_pos.pose.position.z + 2)
        rospy.loginfo(self.target_heading)

        # Send a few setpoints before starting
        for _ in range(100):
            if(rospy.is_shutdown()):
                break

            self.local_pos_pub.publish(self.target_heading)
            self.rate.sleep()

    def record_position(self, msg):
        self.current_pos = msg

    def fly_x(self, right):
        target_twist = TwistStamped()
        if right:
            target_twist.twist.linear.x = 1
        else:
            target_twist.twist.linear.x = -1

        z_diff = self.current_pos.pose.position.z - self.target_heading.pose.position.z
        target_twist.twist.linear.z = -1*z_diff
        rospy.loginfo(z_diff)
        target_twist.twist.linear.y = 0
      
        self.local_vel_pub.publish(target_twist)
        
    def fly_y(self, right):
        target_twist = TwistStamped()
        if right:
            target_twist.twist.linear.y = 1
        else:
            target_twist.twist.linear.y = -1
        z_diff = self.current_pos.pose.position.z - self.target_heading.pose.position.z
        target_twist.twist.linear.z = -1*z_diff
        rospy.loginfo(z_diff)
        target_twist.twist.linear.x = 0

        self.local_vel_pub.publish(target_twist)
        
    def check_signal(self, msg):
        if msg.range >= 0:
            self.signal = True
    
        
    def run(self):
        # Sets drone to OFFBOARD mode for autonomous control
        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'

        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

        self.last_req = rospy.Time.now()
        self.change_heading_interval = rospy.Time.now()
        self.change_heading = True
        self.offboard_disabled = True
        self.hovered = False

        # Send initial setpoint
        self.set_hover()

        initial_y = 0

        while(not rospy.is_shutdown()):
            # Waits until OFFBOARD is enabled to continue
            if(self.current_state.mode != "OFFBOARD" and self.offboard_disabled):
                print('waiting for offboard')
                if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")
                    self.offboard_disabled = False
                    self.last_req = rospy.Time.now()
            else:
                # Waiting for the drone to finish arming to continue
                if(not self.current_state.armed):
                    if(self.arming_client.call(self.arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")
                    else:
                        self.last_req = rospy.Time.now()

            
            if(rospy.Time.now() - self.last_req) > rospy.Duration(10.0):
                self.hovered = True

            if self.hovered and not self.signal:

                current_distance_x = self.current_pos.pose.position.x
                current_distance_y = self.current_pos.pose.position.y
                
                if self.flying_up:
                    self.fly_y(True)
                    if (current_distance_y - initial_y) > 2 * self.search_range:
                        rospy.loginfo("Snake turn")
                        self.fly_x(self.fly_to_right)
                        self.fly_to_right = not (self.fly_to_right)
                        self.flying_up = False
                else:
                    self.fly_x(self.fly_to_right)
                    if (self.fly_to_right and (self.search_size[0] - current_distance_x) < self.search_range) or (not self.fly_to_right and (current_distance_x < self.search_range)):
                        rospy.loginfo("Snake up")
                        self.fly_y(True)
                        initial_y = current_distance_y
                        self.flying_up = True
            else:
                self.local_pos_pub.publish(self.target_heading)
            self.rate.sleep()

if __name__ == '__main__':
    drone = drone_ros()
    drone.run()

