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
FOUND_THRESH = 40  # decimeters; max distance to consider beacon found
SETPOINT_ALT = 3  # m; how far above ground to follow
MONOTONY_FILTER_MAX = 5  # ul; how many consecutive increases before deciding to turn around
TURN_AROUD_TIME = np.pi / ANGULAR_SPEED  # s; How long to wait for about face turn
TRAJ_TS = 0.05  # s; how large each time step is on trapezoidal trajectory
TRAJ_ACC = 0.25  # m/s^2; how fast to accelerate in trapezoidal trajectory
TRAJ_TOTAL_TIME = 7  # s; how long to take on the trapezoidal trajectory

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
                             input_keys=['hover_pos', 'max_time', 'last_beacon_pos'],
                             output_keys=['beacon_data', 'last_beacon_pos'])
        rospy.Subscriber('mavros/avalanche_beacon', 
                                           AvalancheBeacon, callback = self.handle_beacon, 
                                           queue_size=10)
        self.position_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.beacon_data: AvalancheBeacon = None
        self.spin_rate = rospy.Rate(3)
    
    def handle_beacon(self, msg: AvalancheBeacon):
        if not (msg.direction == msg.range == 0):
            self.beacon_data = msg
    
    def execute(self, ud):
        start_time = rospy.get_time()
        self.beacon_data = None
        while rospy.get_time() < start_time + ud.max_time and not self.beacon_data:
            self.position_pub.publish(ud.hover_pos)
            rospy.loginfo(f'waiting on beacon...  {rospy.get_time() - start_time}')
            self.spin_rate.sleep()
        if self.beacon_data:
            ud.beacon_data = self.beacon_data
            return 'found_beacon'
        else:
            ud.beacon_data = None
            if 'last_beacon_pos' in ud.keys():
                ud.last_beacon_pos = ud.last_beacon_pos
            return 'timeout'

class FollowArrow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['located_beacon', 'beacon_loss', 'distance_increase'], 
                             input_keys=['hover_pos', 'beacon_data'],
                             output_keys=['last_beacon_data', 'last_beacon_pos', 'hover_pos'])
        # Member variables
        self.last_beacon_time = rospy.get_time()
        self.beacon_data: AvalancheBeacon = None
        self.last_beacon_pos: PoseStamped = None
        self.pose = None
        self.velocity_vector = None
        self.monotony_filter = 0  # How many conscutive beacon range increases

        self.spin_rate = rospy.Rate(20)

        # Publishers and Subscribers
        self.tl = TransformListener()
        rospy.Subscriber('mavros/avalanche_beacon', 
                                           AvalancheBeacon, callback = self.beacon_cb, 
                                           queue_size=1)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, 
                         callback=self.pose_cb, queue_size=10)
        self.local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', 
                                             TwistStamped, queue_size=10)
    def beacon_cb(self, msg: AvalancheBeacon):
        if not (msg.direction == msg.range and msg.range == 0):
            if self.beacon_data and msg.range > self.beacon_data.range:
                self.monotony_filter += 1
                rospy.loginfo(f'monotony filter: {self.monotony_filter}')
            else:
                self.monotony_filter = 0
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
        if self.monotony_filter > MONOTONY_FILTER_MAX:
            self.monotony_filter = 0  # Reset so doesn't retrigger
            return True
        else:
            return False
    
    def calculate_velocity_vector(self, msg: AvalancheBeacon) -> TwistStamped:
        '''Calculates a velocity vector for a given beacon message'''
        # Currently just angle based, but could do some sort of 
        # porporitional decay at low distances later

        # Correct uint8 to int8 and get sign
        sn = np.sign(np.int8(msg.direction)) 

        # Get vertical speed (to avoid ground)
        vspeed = (SETPOINT_ALT - self.pose.pose.position.z) * 1

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
        self.monotony_filter = 0

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
                ud.hover_pos = self.pose
                rospy.loginfo(f'LOST BEACON; last position: {self.last_beacon_pos} ')
                return 'beacon_loss'
            elif self.is_beacon_found():
                ud.last_beacon_data = self.beacon_data
                ud.last_beacon_pos = self.last_beacon_pos
                ud.hover_pos = self.pose
                return 'located_beacon'
            elif self.is_distance_increasing():
                ud.hover_pos = self.pose
                ud.last_beacon_data = self.beacon_data
                return 'distance_increase'
            
            # Publish velocity aligning with arrow of beacon
            self.local_vel_pub.publish(self.velocity_vector)
            self.spin_rate.sleep()

class AboutFace(smach.State):
    '''Turn the drone about 180 degrees'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['lost_beacon', 'finished_turning'], 
                                input_keys=['hover_pos', 'beacon_data'],
                                output_keys=['last_beacon_data', 'last_beacon_pos'])
        self.pose: PoseStamped = None

        # Publishers and Subscribers
        self.tl = TransformListener()
        self.local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', 
                                             TwistStamped, queue_size=10)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, 
                         callback=self.pose_cb, queue_size=10)
        self.spin_rate = rospy.Rate(20)
    
    def pose_cb(self, msg: PoseStamped):
        self.pose = msg

    def execute(self, ud):
        # Wait for pose data
        rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        start_time = rospy.get_time()
        # Rotate for as long as necessary at set speed
        while rospy.get_time() < start_time + TURN_AROUD_TIME and not rospy.is_shutdown():
            vspeed = (SETPOINT_ALT - self.pose.pose.position.z) * 1  # Correct altitude
            tw = TwistStamped(rospy.Header(frame_id='map'), Twist(Vector3(0, 0, vspeed), 
                                                                  Vector3(0, 0, ANGULAR_SPEED)))
            self.local_vel_pub.publish(tw)
        
        # See if we can still see the beacon
        try:
            beacon_msg: AvalancheBeacon = rospy.wait_for_message('/mavros/avalanche_beacon', AvalancheBeacon, rospy.Duration(STALE_BEACON_TIME))
        except rospy.exceptions.ROSException as e:  # Catch the timeout
            ud.last_beacon_pos = self.pose
            return 'lost_beacon'
        
        if not beacon_msg or beacon_msg.range == beacon_msg.direction == 0:  # If the beacon was lost after the turn, go to handle lost beacon state
            ud.last_beacon_pos = self.pose
            return 'lost_beacon'
        else:
            ud.last_beacon_data = beacon_msg
            ud.last_beacon_pos = self.pose
            return 'finished_turning'

class GoToPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recovered_beacon', 'at_next_pose'], 
                                input_keys=['hover_pos', 'beacon_data'],
                                output_keys=['last_beacon_data', 'last_beacon_pos', 'hover_pos'])
        self.pose: PoseStamped = None
        self.beacon_data: AvalancheBeacon = None
        self.last_beacon_pos: PoseStamped = None

        # Publishers and Subscribers
        self.tl = TransformListener()
        rospy.Subscriber('mavros/avalanche_beacon', 
                                           AvalancheBeacon, callback = self.beacon_cb, 
                                           queue_size=1)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, 
                         callback=self.pose_cb, queue_size=10)
        self.pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        self.spin_rate = rospy.Rate(20)

    def beacon_cb(self, msg: AvalancheBeacon):
        if not (msg.direction == msg.range and msg.range == 0):
            self.beacon_data = msg
            self.last_beacon_pos = self.pose  # Record last seen position
    
    def pose_cb(self, msg: PoseStamped):
        self.pose = msg
    
    def execute(self, ud):
        target_pose: PoseStamped = ud.hover_pos
        start_pose: PoseStamped = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        self.beacon_data = None
        angle = np.arctan2(target_pose.pose.position.y - start_pose.pose.position.y,
                           target_pose.pose.position.x - start_pose.pose.position.x)
        # Specify trajectory
        radius = np.linalg.norm(np.array([target_pose.pose.position.x - start_pose.pose.position.x,
                                          target_pose.pose.position.y - start_pose.pose.position.y,
                                          target_pose.pose.position.z - start_pose.pose.position.z]))
        traj_gen = timed_trapezoidal_traj(0, radius, TRAJ_ACC, LINEAR_SPEED, TRAJ_TOTAL_TIME, TRAJ_TS)
        
        # Run through generated trajectory
        # for pos in traj_gen:  # rate.sleep seems weird with for loops...
        while not rospy.is_shutdown():
            # Check if beacon is seen
            if self.beacon_data:
                ud.last_beacon_pos = self.pose
                ud.last_beacon_data = self.beacon_data
                rospy.loginfo(f'FOUND BEACON: {self.beacon_data}')
                return 'recovered_beacon'
            
            # Attempt to generate a traj point
            try:
                pos = next(traj_gen)
            except StopIteration:  # If traj is empty, leave the loop
                break

            next_pose = PoseStamped(start_pose.header, Pose(Point(start_pose.pose.position.x + pos * np.cos(angle),
                                                                  start_pose.pose.position.y + pos * np.sin(angle),
                                                                  SETPOINT_ALT),
                                                            start_pose.pose.orientation))
            radius = np.linalg.norm(np.array([target_pose.pose.position.x - start_pose.pose.position.x,
                                          target_pose.pose.position.y - start_pose.pose.position.y,
                                          target_pose.pose.position.z - start_pose.pose.position.z]))
            rospy.loginfo(f'radius: {radius}')
            self.pose_pub.publish(next_pose)
            self.spin_rate.sleep()

        # Check if beacon is seen
        if self.beacon_data:
            ud.last_beacon_pos = self.pose
            ud.last_beacon_data = self.beacon_data
            return 'recovered_beacon'
        else:
            ud.hover_pos = self.pose
            return 'at_next_pose'
            

def timed_trapezoidal_traj(start, end, acc, max_vel, max_time, time_step):
    '''Yields a trapezoidal trajectory with a strictly positive velocity'''
    vel = 0
    pos = start
    for t in np.arange(0, max_time, time_step):
        if abs(vel) < max_vel and 0.5 * vel * t < (end - start) / 2:
            vel += acc * time_step
        if abs(vel) > 0.1 and end - pos < (0.5 * vel * max_vel / acc):
            vel -= acc * time_step
        pos += time_step * vel
        if pos > end:
            return
        yield pos