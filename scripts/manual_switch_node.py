#! /usr/bin/env python3
"""
 * File: offb_snake_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""



import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion
from mavros_msgs.msg import State, AvalancheBeacon, ParamValue
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, ParamSet

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
        self.init_z = 0

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
        self.target_heading = self.new_heading(self.target_heading, self.current_pos.pose.position.x, self.current_pos.pose.position.y, self.init_z)
        rospy.loginfo(self.target_heading)

        # Send a few setpoints before starting
        for _ in range(100):
            if(rospy.is_shutdown()):
                break

            self.local_pos_pub.publish(self.target_heading)
            self.rate.sleep()

    def record_position(self, msg):
        self.current_pos = msg

    def fly_x(self, posx):
        target_pose = PoseStamped()
        target_pose.pose.position.x = posx
        target_pose.pose.position.y = self.current_pos.pose.position.y
        target_pose.pose.position.z = self.init_z
        self.local_pos_pub.publish(target_pose)
        
    def fly_y(self, posy):
        target_pose = PoseStamped()
        target_pose.pose.position.y = posy
        target_pose.pose.position.x = self.current_pos.pose.position.x
        target_pose.pose.position.z = self.init_z
        self.local_pos_pub.publish(target_pose)
        
    def check_signal(self, msg):
        if msg.range > 0:
            self.signal = True
   
    def set_horizontal_velocity(self, max_velocity):
        rospy.wait_for_service('/mavros/param/set')
        try:
            max_hor_vel = rospy.ServiceProxy('/mavros/param/set', ParamSet)
            max_hor_vel(param_id="MPC_XY_VEL_ALL", value=ParamValue(real=max_velocity))
        except rospy.ServiceException as e:
            print("Service max_horizontal_velocity (MPC_XY_VEL_MAX) call failed: %s" % e)
        
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
        self.init_z = self.current_pos.pose.position.z + 2
        rospy.loginfo(self.init_z)

        # Send initial setpoint
        # self.set_hover()

        initial_y = 0

        # self.set_horizontal_velocity(1)

        # while(not rospy.is_shutdown()):
        #     # Waits until OFFBOARD is enabled to continue
        #     if(self.current_state.mode != "OFFBOARD" and self.offboard_disabled):
        #         print('waiting for offboard')
        #         if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
        #             rospy.loginfo("OFFBOARD enabled")
        #             self.offboard_disabled = False
        #             self.last_req = rospy.Time.now()
        #     else:
        #         # Waiting for the drone to finish arming to continue
        #         if(not self.current_state.armed):
        #             if(self.arming_client.call(self.arm_cmd).success == True):
        #                 rospy.loginfo("Vehicle armed")
        #             else:
        #                 self.last_req = rospy.Time.now()

        # spinlock until in pos mode
        while ((not self.current_state == ))

if __name__ == '__main__':
    drone = drone_ros()
    drone.run()
