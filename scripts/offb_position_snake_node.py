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

        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_callback, queue_size=1)

        # Where target_heading data is published as PoseStamped (x, y, z)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = self.record_position, queue_size=10)

        self.local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)
  
        # Where DevKit heading data is received from as AvalancheBeacon type
        self.velocity_sub = rospy.Subscriber("mavros/avalanche_beacon", AvalancheBeacon, callback = self.check_signal, queue_size=1)
        
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

        self.search_size = (12, 24) # (x,y) in meters
        self.search_range = 2.5 # in meter

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
        self.set_hover()

        initial_y = 0

        # self.set_horizontal_velocity(1)

        while(not rospy.is_shutdown()):
            spinlock_rate = rospy.Rate(2)
            # Spin until drone is armed by something else:
            while not self.current_state.armed and not rospy.is_shutdown():
                spinlock_rate.sleep()
                rospy.loginfo('Waiting for arm...')
            rospy.loginfo('@@@@@@ ARMED @@@@@@')

            # # Spin until drone is put into position mode
            # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
            # UNCOMMENT THIS ON REAL DRONE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            # YOU ABSOLUTE DOOFUS
            # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
            # while not self.current_state.mode == self.current_state.MODE_PX4_POSITION:
            #     spinlock_rate.sleep()
            #     rospy.loginfo(f'Waiting for position mode...    current mode: {self.current_state.mode}')
            # rospy.loginfo('@@@@@@ In position mode @@@@@@')

            # Spin until drone put into offboard mode
            while not self.current_state.mode == self.current_state.MODE_PX4_OFFBOARD and not rospy.is_shutdown():
                spinlock_rate.sleep()
                rospy.loginfo(f'Waiting for offboard mode... current mode: {self.current_state.mode}')
                self.local_pos_pub.publish(self.current_pos)
                self.last_req = rospy.Time.now()
            rospy.loginfo('@@@@@@ WARNING: OFFBOARD MODE ENGAGED @@@@@@')

            # while rospy.get_time() < self.last_req + 10:
            #     rospy.loginfo('hovering')

            if(rospy.Time.now() - self.last_req) > rospy.Duration(10.0):
                self.hovered = True

            if self.hovered and not self.signal:
                # rospy.loginfo('----------------------snaking-------------------------')
                current_distance_x = self.current_pos.pose.position.x
                current_distance_y = self.current_pos.pose.position.y
                
                if self.flying_up:
                    self.fly_y(current_distance_y + self.search_range)
                    if (current_distance_y - initial_y) > 2 * self.search_range:
                        rospy.loginfo("Snake turn")
                        if(self.fly_to_right):
                            self.fly_x(current_distance_x + self.search_range/2)
                        else:
                            self.fly_x(current_distance_x - self.search_range/2)
                        self.fly_to_right = not (self.fly_to_right)
                        self.flying_up = False
                else:
                    if(self.fly_to_right):
                            self.fly_x(current_distance_x + self.search_range/2)
                    else:
                        self.fly_x(current_distance_x - self.search_range/2)
                    if (self.fly_to_right and (self.search_size[0] - current_distance_x) < self.search_range) or (not self.fly_to_right and (current_distance_x < self.search_range)):
                        rospy.loginfo("Snake up")
                        self.fly_y(current_distance_y + self.search_range)
                        initial_y = current_distance_y
                        self.flying_up = True
            else:
                if self.signal:
                    self.target_heading.pose.position.x = self.current_pos.pose.position.x
                    self.target_heading.pose.position.y = self.current_pos.pose.position.y
                self.local_pos_pub.publish(self.target_heading)
                rospy.loginfo('hoverin')
                if self.last_req:
                    rospy.loginfo((rospy.Time.now() - self.last_req).secs)
            self.rate.sleep()

if __name__ == '__main__':
    drone = drone_ros()
    drone.run()
