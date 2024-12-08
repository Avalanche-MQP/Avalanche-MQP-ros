"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion
from mavros_msgs.msg import State, AvalancheBeacon
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from offboard_py.msg import DevKit

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
        #self.velocity_sub = rospy.Subscriber("mavros/avalanche_beacon", AvalancheBeacon, callback = self.convert_velocity, queue_size=10)
        
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

        # Timestamp of last beacon msg
        self.beacon_ts = rospy.get_time()

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.current_state.connected):
            print('waaaaiiiting is the hardest part')
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

    def convert_velocity(self, msg):
        self.beacon_ts = rospy.get_time()
        pieps_range = (float)(msg.range) / 10.0
        pieps_direction_t = unsigned_to_signed(msg.direction)


        # PIEPS uses -5 -> 5 to indicate range of rotation, this generalizes the numbers to a degree and either positive and negative
        relative_range = 0
        if abs(pieps_direction_t) == 1:
            relative_range = 0
        elif abs(pieps_direction_t) == 2:
            relative_range = 0.0349065850399
        elif abs(pieps_direction_t) == 3:
            relative_range = 0.0523598775598
        elif abs(pieps_direction_t) == 4:
            relative_range = 0.0698131700798
        elif abs(pieps_direction_t) == 5:
            relative_range = 0.0872664625997

        if pieps_direction_t < 0:
            relative_range = relative_range * -1

        target_twist = TwistStamped()
        target_twist.twist.linear.x = 1
        target_twist.twist.linear.z = 0
        target_twist.twist.angular.z = relative_range

        if pieps_range < 0.2:
            target_twist.twist.linear.x = 0
            target_twist.twist.linear.z = 0
            target_twist.twist.angular.z = 0

        self.target_twist = target_twist

    def set_hover(self):
        self.target_heading = self.new_heading(self.target_heading, 0, 0, 2)

        # Send a few setpoints before starting
        for _ in range(100):
            if(rospy.is_shutdown()):
                break

            self.local_pos_pub.publish(self.target_heading)
            self.rate.sleep()

    def record_position(self, msg):
        self.current_pos = msg
        
    def run(self):
        rospy.loginfo('run starting')
        # Sets drone to OFFBOARD mode for autonomous control
        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'

        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

        self.last_req = rospy.Time.now()
        self.change_heading_interval = rospy.Time.now()
        self.change_heading = True
        self.offboard_disabled = True

        rospy.loginfo('Entering the while loop')
        while(not rospy.is_shutdown()):
            # Waits until OFFBOARD is enabled to continue
            self.set_hover()
            if(self.current_state.mode != "OFFBOARD" and self.offboard_disabled):
                print('waiting for offboard')
                if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")
                    self.offboard_disabled = False
                    rospy.loginfo(self.offboard_disabled)

                self.last_req = rospy.Time.now()
            else:
                # Waiting for the drone to finish arming to continue
                print('waiting for arm')
                if(not self.current_state.armed):
                    print('arming')
                    # self.set_hover()
                    if(self.arming_client.call(self.arm_cmd).success == True):
                        
                        rospy.loginfo("Vehicle armed")
                        # self.set_hover()
                    else:
                        print('arm failed :sadge:')
                    self.last_req = rospy.Time.now()

            # if rospy.get_time() < self.beacon_ts + 5: #self.new_message
            #     self.local_vel_pub.publish(self.target_twist)
            # else:
            self.local_pos_pub.publish(self.target_heading) # Sends current target heading to the drone

            self.rate.sleep()


if __name__ == '__main__':
    drone = drone_ros()
    rospy.loginfo('in between')
    drone.run()
