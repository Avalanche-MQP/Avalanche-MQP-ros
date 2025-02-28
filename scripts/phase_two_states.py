#! /usr/bin/env python3
import rospy
import smach
import smach_ros
from mavros_msgs.msg import State, AvalancheBeacon
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Point, Quaternion, Twist, Vector3, Vector3Stamped
import numpy as np
from tf.listener import TransformListener

SAFE_ALTITUDE = 2.5  # m; minimum altitude to start offboard control

class WaitForTakeoff(smach.State):
    '''Waits for the user to takeoff and enter OFFBOARD mode'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['offboard_engaged'])
        self.spin_rate = rospy.Rate(2)
        self.state: State = None
        rospy.Subscriber('/mavros/state', State, callback=self.state_cb)
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
    def state_cb(self, msg: State):
        self.state = msg

    def execute(self, userdata):
        # Wait for first state message to come through
        rospy.wait_for_message('/mavros/state', State)

        # Wait for arming
        while not self.state.armed and not rospy.is_shutdown():
            rospy.loginfo('Waiting for arm...')
            self.spin_rate.sleep()
        rospy.loginfo('@@@@@@ ARMED @@@@@@')

        # Wait for OFFBOARD
        while not self.state.mode == self.state.MODE_PX4_OFFBOARD and not rospy.is_shutdown():
            rospy.loginfo('Waiting for offboard...')
            # Seend target position data to ensure transfer to offboard
            # is allowed
            curr_pos = rospy.wait_for_message('mavros/local_position/pose', PoseStamped)
            self.local_pos_pub.publish(curr_pos)
            self.spin_rate.sleep()
        rospy.loginfo('@@@@@@ WARNING: OFFBOARD MODE ENGAGED @@@@@@')
        return 'offboard_engaged'

class SimTakeoff(smach.State):
    '''Takes off; SIMULATION ONLY'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['sim_takeoff_complete'],
                             output_keys=['hover_pos'])
        self.position_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.spin_rate = rospy.Rate(20)

    def execute(self, ud):
        start_time = rospy.get_time()
        target = PoseStamped(rospy.Header(frame_id='map'),
                             Pose(position=Point(0, 0, 3)))
        while rospy.get_time() < start_time + 10 and not rospy.is_shutdown():
            self.position_pub.publish(target)
            self.spin_rate.sleep()
        ud.hover_pos = rospy.wait_for_message('mavros/local_position/pose', PoseStamped)
        return 'sim_takeoff_complete'

class CheckAltitude(smach.State):
    '''Checks altitude to see if it's safe'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['safe_altitude', 'unsafe_altitude'],
                             output_keys=['hover_pos'])
    
    def execute(self, ud):
        curr_pos: PoseStamped = rospy.wait_for_message('mavros/local_position/pose', PoseStamped)
        ud.hover_pos = curr_pos
        if curr_pos.pose.position.z >= SAFE_ALTITUDE:
            return 'safe_altitude'
        else:
            return 'unsafe_altitude'
        
class WaitForBeacon(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_beacon', 'timeout'], 
                             input_keys=['hover_pos', 'max_time'],
                             output_keys=['beacon_data'])
        rospy.Subscriber('mavros/avalanche_beacon', 
                                           AvalancheBeacon, callback = self.handle_beacon, 
                                           queue_size=10)
        self.position_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.beacon_data: AvalancheBeacon = None
        self.spin_rate = rospy.Rate(3)
    
    def handle_beacon(self, msg: AvalancheBeacon):
        self.beacon_data = msg
    
    def execute(self, ud):
        start_time = rospy.get_time()
        while rospy.get_time() < start_time + ud.timeout and not self.beacon_data:
            self.position_pub.publish(ud.hover_pos)
            self.spin_rate.sleep()
        if self.beacon_data:
            ud.beacon_data = self.beacon_data
            return 'found_beacon'
        else:
            ud.beacon_data = None
            return 'timeout'
