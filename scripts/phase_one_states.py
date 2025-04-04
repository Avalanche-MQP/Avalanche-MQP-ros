#! /usr/bin/env python3
import rospy
import smach
import smach_ros
from mavros_msgs.msg import State, AvalancheBeacon
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Point, Quaternion, Twist, Vector3, Vector3Stamped
from rospy.timer import TimerEvent
import numpy as np
from tf.listener import TransformListener
from tf.transformations import euler_from_quaternion
import copy

## Globals
FORWARD_STEP = 20  # m; how far to move goal forward
SIDEWAYS_STEP = 20  # m; how far to move goal sideways
POSE_RATE = 10  # hz; how fast to send pose
MAX_SPEED = 1  # m/s; speed to move the drone
FORWARD_TIME = FORWARD_STEP / MAX_SPEED  # s; how long the forward leg takes
SIDEWAYS_TIME = SIDEWAYS_STEP / MAX_SPEED  # s; how long the sideways leg takes
INITIAL_HEADING = 0  # initial yaw; used to set direction of snake


## Utility states 
class WaitForTakeoff(smach.State):
    '''Waits for the user to takeoff and enter OFFBOARD mode'''
    global INITIAL_HEADING
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
        # Store current heading as the heading to base snake off
        (r, p, y) = euler_from_quaternion([curr_pos.pose.orientation.x,
                                            curr_pos.pose.orientation.y,
                                            curr_pos.pose.orientation.z,
                                            curr_pos.pose.orientation.w,])
        INITIAL_HEADING = y
        return 'offboard_engaged'

class SimTakeoff(smach.State):
    '''Takes off; SIMULATION ONLY'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['sim_takeoff_complete'],
                             output_keys=['hover_pos'])
        self.position_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.spin_rate = rospy.Rate(20)

    def execute(self, ud):
        curr_pose = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        start_time = rospy.get_time()
        target = PoseStamped(rospy.Header(frame_id='map'),
                             Pose(position=Point(0, 0, 3), orientation=curr_pose.pose.orientation))
        while rospy.get_time() < start_time + 10 and not rospy.is_shutdown():
            self.position_pub.publish(target)
            self.spin_rate.sleep()
        ud.hover_pos = rospy.wait_for_message('mavros/local_position/pose', PoseStamped)
        return 'sim_takeoff_complete'

class SetAltitude(smach.State):
    '''Sets the current altitude'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['safe_altitude', 'unsafe_altitude'],
                             output_keys=['hover_pos'])
    
    def execute(self, ud):
        global SETPOINT_ALT
        curr_pos: PoseStamped = rospy.wait_for_message('mavros/local_position/pose', PoseStamped)
        ud.hover_pos = curr_pos
        SETPOINT_ALT = curr_pos.pose.position.z
        return 'safe_altitude'
    
## Actual phase one state
class Snake(smach.State):
    '''Handles phase one of the search'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'],
                             input_keys=['move_dir'],
                             output_keys=['move_dir'])
        self.pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped)
        rospy.Subscriber('/mavros/avalanche_beacon', AvalancheBeacon, self.handle_beacon)
        self.found_beacon = False
        self.t : rospy.Timer = None
        self.start_pose: PoseStamped = None

    def handle_beacon(self, msg: AvalancheBeacon):
        if msg.range != 0 and msg.direction != 0:
            self.found_beacon = True
        
    def fly_function(self, time: float) -> PoseStamped:
        '''Generates a point based on the time and moves the drone accordingly'''
        delta_x = self.goal_pose.pose.position.x - self.start_pose.pose.position.x
        delta_y = self.goal_pose.pose.position.y - self.start_pose.pose.position.y

        pose_to_send = copy.deepcopy(self.start_pose)
        if self.move_dir == 'lforward' or self.move_dir == 'rforward':
            pose_to_send.pose.position.x += (time / FORWARD_TIME) * delta_x
            pose_to_send.pose.position.y += (time / FORWARD_TIME) * delta_y
        elif self.move_dir == 'left' or self.move_dir == 'right':
            pose_to_send.pose.position.x += (time / SIDEWAYS_TIME) * delta_x
            pose_to_send.pose.position.y += (time / SIDEWAYS_TIME) * delta_y

        self.pose_pub.publish(pose_to_send)

        
    def execute(self, ud):
        # Initialize start pose if first execution
        if not self.start_pose:
            self.start_pose = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
            (r, p, y) = euler_from_quaternion([self.start_pose.pose.orientation.x,
                                            self.start_pose.pose.orientation.y,
                                            self.start_pose.pose.orientation.z,
                                            self.start_pose.pose.orientation.w,])
            self.initial_heading = y

        # Extract userdata
        self.goal_pose: PoseStamped = copy.deepcopy(self.start_pose)
        self.move_dir = ud.move_dir

        # Increment target position
        if self.move_dir == 'lforward':  # Left side of field; moving forward
            self.goal_pose.pose.position.x += np.cos(self.initial_heading) * FORWARD_STEP
            self.goal_pose.pose.position.y += np.sin(self.initial_heading) * FORWARD_STEP
        elif self.move_dir == 'rforward':  # Right side of field; moving forward
            self.goal_pose.pose.position.x += np.cos(self.initial_heading) * FORWARD_STEP
            self.goal_pose.pose.position.y += np.sin(self.initial_heading) * FORWARD_STEP
        elif self.move_dir == 'left':  # Moving left 
            self.goal_pose.pose.position.x += np.sin(self.initial_heading) * SIDEWAYS_STEP
            self.goal_pose.pose.position.y -= np.cos(self.initial_heading) * SIDEWAYS_STEP
        elif self.move_dir == 'right':  # Moving right
            self.goal_pose.pose.position.x -= np.sin(self.initial_heading) * SIDEWAYS_STEP
            self.goal_pose.pose.position.y += np.cos(self.initial_heading) * SIDEWAYS_STEP
        else:
            raise ValueError(f'Invalid move_dir: {self.move_dir}')

        # Log
        rospy.loginfo(f'Initial heading: {self.initial_heading}\n \
                        start_pose: ({self.start_pose.pose.position.x}, {self.start_pose.pose.position.y})\n \
                        goal_pose: ({self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y})\n  \
                        move_dir: {self.move_dir}')

       # Run in loop until reached point or found beacon
        wait = FORWARD_TIME if self.move_dir == 'lforward' or self.move_dir == 'rforward' else SIDEWAYS_TIME
        start = rospy.get_time()
        rate = rospy.Rate(POSE_RATE)
        while rospy.get_time() - start < wait:
            self.fly_function(rospy.get_time() - start)
            if self.found_beacon:
                return 'found'
            rate.sleep()

        # Update ud vars
        self.start_pose = self.goal_pose
        self.goal_pose = None
        if self.move_dir == 'lforward':
            ud.move_dir = 'right'
        elif self.move_dir == 'rforward':
            ud.move_dir = 'left'
        elif self.move_dir == 'left':
            ud.move_dir = 'lforward'
        elif self.move_dir == 'right':
            ud.move_dir = 'rforward'
        
        return 'not_found'