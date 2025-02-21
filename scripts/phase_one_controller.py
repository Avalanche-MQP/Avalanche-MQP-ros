#! /usr/bin/env python3
import rospy
from mavros_msgs.msg import State, AvalancheBeacon
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Point, Quaternion
class PhaseOneController:
    def __init__(self):
        rospy.init_node('phase_one_controller')
        self.current_state: State = None
        self.current_pos: PoseStamped = None
        self.hover_level: float = None

        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_callback)

        # Where target_heading data is published as PoseStamped (x, y, z)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = self.record_position, queue_size=10)

        self.local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
  
        # Where DevKit heading data is received from as AvalancheBeacon type
        self.velocity_sub = rospy.Subscriber("mavros/avalanche_beacon", AvalancheBeacon, callback = self.check_signal, queue_size=10)

        # Wait for drone to come up
        rospy.wait_for_message('mavros/state', State, rospy.Duration(100))

    def state_callback(self, msg):
        """
        This figures out the MODE of the drone.
        """

        self.current_state = msg
    
    def record_position(self, msg):
        self.current_pos = msg

    def check_signal(self, msg):
        if msg.range > 0:
            self.signal = True
    
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
        while not rospy.is_shutdown():
            self.local_pos_pub.publish(self.current_pos.header, Pose(Point(self.current_pos.pose.position.x, 
                                                                           self.current_pos.pose.position.y,
                                                                           self.hover_level), self.current_pos.pose.orientation))
            spinlock_rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    PhaseOneController().run()