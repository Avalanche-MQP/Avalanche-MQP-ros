#! /usr/bin/env python3
import rospy
import smach
import smach_ros
import phase_two_states

INITIAL_WAIT_TIME = 30  # s; how long to wait to acquire beacon after hover
LOST_WAIT_TIME = 15  # s; how long to hover in place after losing the beacon and after returning to the previously seen spot
class PhaseTwoSmach:
    def __init__(self):
        rospy.init_node('state_machine')

        # Build the state machine
        self.sm = smach.StateMachine(outcomes=['found', 'not_found'])
        self.sm.userdata.beacon_data = None
        self.sm.userdata.hover_pos = None
        self.sm.userdata.initial_timeout = INITIAL_WAIT_TIME
        with self.sm:
            self.sm.add('WAIT_FOR_TAKEOFF', phase_two_states.WaitForTakeoff(), 
                   transitions={'offboard_engaged': 'SIM_TAKEOFF'})  # @@@ SIM ONLY @@@
                    # transitions={'offboard_engaged': 'CHECK_ALTITUDE'})  # @@ REAL DRONE @@
            self.sm.add('SIM_TAKEOFF', phase_two_states.SimTakeoff(),
                        transitions={'sim_takeoff_complete': 'CHECK_ALTITUDE'})
            self.sm.add('CHECK_ALTITUDE', phase_two_states.CheckAltitude(),
                   transitions={'safe_altitude': 'WAIT_FOR_BEACON',
                                'unsafe_altitude': 'not_found'})
            # TODO: Replace this with snake
            self.sm.add('WAIT_FOR_BEACON', phase_two_states.WaitForBeacon(),
                        transitions={'found_beacon': 'FOLLOW_ARROW',
                                     'timeout': 'not_found'},
                        remapping={'hover_pos' : 'hover_pos',
                                   'max_time' : 'initial_timeout'})
            self.sm.add('FOLLOW_ARROW', phase_two_states.FollowArrow(),
                        transitions={'located_beacon': 'found',
                                     'beacon_loss' : 'RECOVER_BEACON',
                                     'distance_increase': 'ABOUT_FACE'},
                        remapping={'last_beacon_data': 'beacon_data'})
            self.sm.add('ABOUT_FACE', phase_two_states.AboutFace(),
                        transitions={'lost_beacon': 'not_found',
                                     'finished_turning': 'FOLLOW_ARROW'})
            
            # Sub smach for recovering the signal of a lost beacon
            recovery_sm = smach.StateMachine(outcomes=['beacon_lost', 'beacon_recovered'], input_keys=['hover_pos', 'last_beacon_pos'], output_keys=['hover_pos', 'beacon_data'])
            recovery_sm.userdata.timeout_thresh = LOST_WAIT_TIME
            with recovery_sm:
                recovery_sm.add('HOLD_STILL', phase_two_states.WaitForBeacon(),
                                transitions={'found_beacon': 'beacon_recovered',
                                             'timeout': 'GO_TO_LAST_SEEN'},
                                remapping={'max_time': 'timeout_thresh'})
                recovery_sm.add('GO_TO_LAST_SEEN', phase_two_states.GoToPose(),
                                transitions={'recovered_beacon': 'beacon_recovered',
                                             'at_next_pose': 'HOVER_AT_POS'},
                                remapping={'last_beacon_data': 'beacon_data',
                                           'last_beacon_pos': 'last_beacon_pos'})
                recovery_sm.add('HOVER_AT_POS', phase_two_states.WaitForBeacon(),
                                transitions={'timeout': 'beacon_lost',
                                             'found_beacon': 'beacon_recovered'},
                                remapping={'max_time': 'timeout_thresh'})
                
            self.sm.add('RECOVER_BEACON', recovery_sm,
                        transitions={'beacon_lost': 'not_found',
                                     'beacon_recovered': 'FOLLOW_ARROW'},
                        remapping={'hover_pos': 'hover_pos'})
            
        self.sis = smach_ros.IntrospectionServer('sis_server', self.sm, '/SM_ROOT')
        self.sis.start()
    def run(self):
        outcome = self.sm.execute()
        print(f'SM exited: {outcome}')
        rospy.spin()
        self.sis.stop()

if __name__ == '__main__':
    PhaseTwoSmach().run()