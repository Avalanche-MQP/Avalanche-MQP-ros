#! /usr/bin/env python3
import rospy
import smach
import smach_ros
import phase_two_states

INITIAL_WAIT_TIME = 30  # s; how long to wait to acquire beacon after hover
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
                                     'beacon_loss' : 'not_found',
                                     'distance_increase': 'ABOUT_FACE'},
                        remapping={'last_beacon_data': 'beacon_data',
                                   'last_beacon_pos': 'hover_pos',
                                   'about_face_pos' : 'hover_pos'})
            self.sm.add('ABOUT_FACE', phase_two_states.AboutFace(),
                        transitions={'lost_beacon': 'not_found',
                                     'finished_turning': 'FOLLOW_ARROW'})
            
        self.sis = smach_ros.IntrospectionServer('sis_server', self.sm, '/SM_ROOT')
        self.sis.start()
    def run(self):
        outcome = self.sm.execute()
        print(f'SM exited: {outcome}')
        rospy.spin()
        self.sis.stop()

if __name__ == '__main__':
    PhaseTwoSmach().run()