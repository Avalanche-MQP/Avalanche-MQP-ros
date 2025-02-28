#! /usr/bin/env python3
import rospy
import smach
import smach_ros
from mavros_msgs.msg import State, AvalancheBeacon
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Point, Quaternion, Twist, Vector3, Vector3Stamped
import numpy as np
from tf.listener import TransformListener

SAFE_ALTITUDE = 2.5  # m; minimum altitude to start offboard control
STALE_BEACON_TIME = 5  # s; maximum time to use beacon data as a heading
LINEAR_SPEED = 1  # m/s; linear velocity for beacon following
ANGULAR_SPEED = np.deg2rad(10)  # s^-1; Max allowable angular speed
FOUND_THRESH = 20  # decimeters; max distance to consider beacon found
SETPOINT_ALT = 3  # m; how far above ground to follow

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
        while rospy.get_time() < start_time + ud.max_time and not self.beacon_data:
            self.position_pub.publish(ud.hover_pos)
            self.spin_rate.sleep()
        if self.beacon_data:
            ud.beacon_data = self.beacon_data
            return 'found_beacon'
        else:
            ud.beacon_data = None
            return 'timeout'

class FollowArrow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['located_beacon', 'beacon_loss', 'distance_increase'], 
                             input_keys=['hover_pos', 'beacon_data'],
                             output_keys=['last_beacon_data', 'last_beacon_pos', 'about_face_pos'])
        # Member variables
        self.last_beacon_time = rospy.get_time()
        self.beacon_data: AvalancheBeacon = None
        self.last_beacon_pos: PoseStamped = None
        self.pose = None
        self.velocity_vector = None

        self.spin_rate = rospy.Rate(20)

        # Publishers and Subscribers
        self.tl = TransformListener()
        rospy.Subscriber('mavros/avalanche_beacon', 
                                           AvalancheBeacon, callback = self.beacon_cb, 
                                           queue_size=10)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, 
                         callback=self.pose_cb, queue_size=10)
        self.local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', 
                                             TwistStamped, queue_size=10)
    def beacon_cb(self, msg: AvalancheBeacon):
        if not (msg.direction == msg.range and msg.range == 0):
            self.beacon_data = msg
            self.last_beacon_time = rospy.get_time()
            self.last_beacon_pos = self.pose  # Record last seen position
            self.velocity_vector = self.calculate_velocity_vector(msg)  # Calculate what direction to go in to find beacon
    
    def pose_cb(self, msg: PoseStamped):
        self.pose = msg
    
    def is_beacon_timeout(self):
        return rospy.get_time() > self.last_beacon_time + STALE_BEACON_TIME
    
    def is_beacon_found(self):
        return self.beacon_data.range <= FOUND_THRESH
    
    def is_distance_increasing(self):
        return False  # TODO: this
    
    def calculate_velocity_vector(self, msg: AvalancheBeacon) -> TwistStamped:
        '''Calculates a velocity vector for a given beacon message'''
        # Currently just angle based, but could do some sort of 
        # porporitional decay at low distances later

        # Correct uint8 to int8 and get sign
        sn = np.sign(np.int8(msg.direction)) 

        # Get vertical speed (to avoid ground)
        vspeed = (SETPOINT_ALT - self.pose.pose.position.z) * 0.5

        # Convert direction from message to radians
        if abs(np.int8(msg.direction)) == 1:
            angle = 0
        elif abs(np.int8(msg.direction)) == 2:
            angle = sn * np.deg2rad(25)
        elif abs(np.int8(msg.direction)) == 3:
            angle = sn * np.deg2rad(35)
        elif abs(np.int8(msg.direction)) == 4:
            angle = sn * np.deg2rad(45)
        else:
            angle = sn * np.deg2rad(50)
        vec = Vector3Stamped(rospy.Header(frame_id='base_link'), Vector3(LINEAR_SPEED * np.cos(angle), LINEAR_SPEED * np.sin(angle), vspeed))
        vec = self.tl.transformVector3('map', vec)
        return TwistStamped(rospy.Header(frame_id='base_link'), Twist(vec.vector, 
                                                                        Vector3(0, 0, sn * ANGULAR_SPEED)))
    
    def execute(self, ud):
        # Update member variables to state inputs
        self.beacon_cb(ud.beacon_data)

        # Follow the arrow until either:
        #   1. Beacon is found
        #       TODO: Add support for testing minimum
        #   2. Beacon is lost (> 5s since last message)
        #   3. Distance is increasing (based off moving avg)
        while not rospy.is_shutdown():
            # Check for exit conditions
            if self.is_beacon_timeout():
                ud.last_beacon_data = self.beacon_data
                ud.last_beacon_pos = self.last_beacon_pos
                return 'beacon_loss'
            elif self.is_beacon_found():
                ud.last_beacon_data = self.beacon_data
                ud.last_beacon_pos = self.last_beacon_pos
                return 'located_beacon'
            elif self.is_distance_increasing():
                ud.about_face_pos = self.pose
                return 'distance_increase'
            
            # Publish velocity aligning with arrow of beacon
            self.local_vel_pub.publish(self.velocity_vector)