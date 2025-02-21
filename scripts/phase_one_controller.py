#! /usr/bin/env python3
import rospy
from mavros_msgs.msg import State, AvalancheBeacon
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Point, Quaternion, Twist, Vector3
import numpy as np

STALE_BEACON_TIME = 5  # Most elapsed time to allow to follow a beacon message
LINEAR_SPEED = 1 # Max allowable linear speed
ANGULAR_SPEED = np.deg2rad(15)  # Max allowable angular speed

class PhaseOneController:
    def __init__(self):
        rospy.init_node('phase_one_controller')
        self.current_state: State = None
        self.current_pos: PoseStamped = None
        self.hover_level: float = None
        self.last_beacon_msg: AvalancheBeacon = None
        self.last_beacon_ts: float = None
        self.found_beacon = False

        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_callback)

        # Where target_heading data is published as PoseStamped (x, y, z)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = self.record_position, queue_size=10)

        self.local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
  
        # Where DevKit heading data is received from as AvalancheBeacon type
        self.velocity_sub = rospy.Subscriber("mavros/avalanche_beacon", AvalancheBeacon, callback = self.handle_beacon, queue_size=10)

        # Wait for drone to come up
        rospy.wait_for_message('mavros/state', State, rospy.Duration(100))

    def state_callback(self, msg):
        """
        This figures out the MODE of the drone.
        """

        self.current_state = msg
    
    def record_position(self, msg):
        self.current_pos = msg

    def handle_beacon(self, msg: AvalancheBeacon):
        if msg.range > 0:
            self.last_beacon_ts = rospy.get_time()
            self.last_beacon_msg = msg
            
    def follow_arrow(self):
        '''Follows beacon with velocity control'''
        # If very close, stop
        if self.last_beacon_msg.range < 30:
            self.local_vel_pub.publish(rospy.Header(frame_id='map'), Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
            print("FOUND!!!!")
        # Check if time since last beacon reading is reasonable
        if rospy.get_time() < self.last_beacon_ts + STALE_BEACON_TIME:
            # Translate last beacon arrow into a direction
            print(f'corrected for sign: {np.int8(self.last_beacon_msg.direction)}')
            sn = np.sign(np.int8(self.last_beacon_msg.direction))
            if abs(np.int8(self.last_beacon_msg.direction)) == 1:
                self.local_vel_pub.publish(rospy.Header(frame_id='map'), Twist(Vector3(LINEAR_SPEED, 0, 0), Vector3(0, 0, 0)))
            elif abs(np.int8(self.last_beacon_msg.direction)) == 2:
                angle = sn * np.deg2rad(25)
                self.local_vel_pub.publish(rospy.Header(frame_id='map'), Twist(Vector3(LINEAR_SPEED*np.cos(angle), LINEAR_SPEED*np.sin(angle), 0), 
                                                                               Vector3(sn * ANGULAR_SPEED, 0, 0)))
            elif abs(np.int8(self.last_beacon_msg.direction)) == 3:
                angle = sn * np.deg2rad(35)
                self.local_vel_pub.publish(rospy.Header(frame_id='map'), Twist(Vector3(LINEAR_SPEED*np.cos(angle), LINEAR_SPEED*np.sin(angle), 0), 
                                                                               Vector3(sn * ANGULAR_SPEED, 0, 0)))
            elif abs(np.int8(self.last_beacon_msg.direction)) == 4:
                angle = sn * np.deg2rad(45)
                self.local_vel_pub.publish(rospy.Header(frame_id='map'), Twist(Vector3(LINEAR_SPEED*np.cos(angle), LINEAR_SPEED*np.sin(angle), 0), 
                                                                               Vector3(sn * ANGULAR_SPEED, 0, 0)))
            else:
                angle = sn * np.deg2rad(50)
                self.local_vel_pub.publish(rospy.Header(frame_id='map'), Twist(Vector3(LINEAR_SPEED*np.cos(angle), LINEAR_SPEED*np.sin(angle), 0), 
                                                                               Vector3(sn * ANGULAR_SPEED, 0, 0)))


        else:  # If reading is stale, stay still (TODO: Recovery behavior)
            self.local_vel_pub.publish(rospy.Header(frame_id='map'), Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
    
    def run(self):
        spinlock_rate = rospy.Rate(2)
        # Spin until drone is armed by something else:
        while not self.current_state.armed:
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
        while not self.current_state.mode == self.current_state.MODE_PX4_OFFBOARD:
            spinlock_rate.sleep()
            rospy.loginfo(f'Waiting for offboard mode... current mode: {self.current_state.mode}')
            self.local_pos_pub.publish(self.current_pos)
        rospy.loginfo('@@@@@@ WARNING: OFFBOARD MODE ENGAGED @@@@@@')

        self.hover_level = self.current_pos.pose.position.z + 2  # @@@@@@@@@@@@@@@ COMMENT ON REAL DRONE!!!
        t = rospy.get_time()
        while t > rospy.get_time() - 5:  # COMMENT OUT
            self.local_pos_pub.publish(self.current_pos.header, Pose(Point(self.current_pos.pose.position.x, 
                                                                           self.current_pos.pose.position.y,
                                                                           self.hover_level), self.current_pos.pose.orientation))
            spinlock_rate.sleep()

        while not rospy.is_shutdown() and not self.found_beacon:
            if self.last_beacon_msg:
                self.follow_arrow()
            rospy.Rate(5).sleep()
        while not rospy.is_shutdown():
            self.local_pos_pub.publish(self.current_pos.header, Pose(Point(self.current_pos.pose.position.x, 
                                                                           self.current_pos.pose.position.y,
                                                                           self.hover_level), self.current_pos.pose.orientation))
            spinlock_rate.sleep()
            print('!!!!!FOUND!!!!!')
        rospy.spin()

if __name__ == '__main__':
    PhaseOneController().run()