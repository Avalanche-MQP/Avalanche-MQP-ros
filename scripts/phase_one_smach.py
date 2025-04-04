#! /usr/bin/env python3
import rospy
import smach
import smach_ros
import phase_one_states

INITIAL_WAIT_TIME = 30  # s; how long to wait to acquire beacon after hover
LOST_WAIT_TIME = 15  # s; how long to hover in place after losing the beacon and after returning to the previously seen spot
class PhaseTwoSmach:
    def __init__(self):
        rospy.init_node('state_machine')

        # Build the state machine
        self.sm = smach.StateMachine(outcomes=['found', 'not_found'])
        self.sm.userdata.beacon_data = None
        self.sm.userdata.hover_pos = None
        self.sm.userdata.move_dir = 'rforward'
        self.sm.userdata.initial_timeout = INITIAL_WAIT_TIME
        with self.sm:
            self.sm.add('WAIT_FOR_TAKEOFF', phase_one_states.WaitForTakeoff(), 
                   transitions={'offboard_engaged': 'SIM_TAKEOFF'})  # @@@ SIM ONLY @@@
                    # transitions={'offboard_engaged': 'SET_ALTITUDE'})  # @@ REAL DRONE @@
            self.sm.add('SIM_TAKEOFF', phase_one_states.SimTakeoff(),
                        transitions={'sim_takeoff_complete': 'SET_ALTITUDE'})
            self.sm.add('SET_ALTITUDE', phase_one_states.SetAltitude(),
                   transitions={'safe_altitude': 'SNAKE',
                                'unsafe_altitude': 'not_found'})
            self.sm.add('SNAKE', phase_one_states.Snake(),
                        transitions={'not_found': 'SNAKE'})
            
    def run(self):
        outcome = self.sm.execute()
        print(f'SM exited: {outcome}')
        rospy.spin()
        # self.sis.stop()

if __name__ == '__main__':
    PhaseTwoSmach().run()