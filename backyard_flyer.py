import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

##### project submitted by Soumic Sarkar

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.current_wp = 0 #new memeber added to track waypoints
        self.side_of_square = 10 # length of the side of a square

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:

            # coordinate conversion 
            altitude = -1.0 * self.local_position[2]

            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                self.calculate_box() # directly sets the waypoints instead of returning
                self.waypoint_transition()
        
        if self.flight_state == States.WAYPOINT:
            
            distance_to_target = np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) # distance to target
            
            if distance_to_target<0.2:
                
                
                print("waypoint", self.current_wp, ":", self.all_waypoints[self.current_wp][0], self.all_waypoints[self.current_wp][1], " -> reached") 
                self.current_wp=self.current_wp+1 # next waypoint
                
                if self.current_wp==len(self.all_waypoints): # if final waypoint is reached
                    self.landing_transition() # prepare landing
                else: # else a call to move to the next waypoint
                    self.waypoint_transition() # to the next waypoint
                              
    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and
            abs(self.local_position[2]) < 0.01):
                self.disarming_transition()

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()

    def calculate_box(self):
        
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        print("waypoints created")
        d=self.side_of_square # sets the side of a square to 'd'
        
        self.all_waypoints = [np.array([d, 0.0, 3.0, 0.0]),
                                np.array([d, 2*d, 3.0, 0.0]), # what if you set '2d' in east ;)
                                np.array([0, 2*d, 3.0, 0.0]), # what if you set '2d' in east ;)
                                np.array([0, 0, 3.0, 0.0])] # sets the waypoints
        

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()

        # set the current location to be the home position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        
        target_northing=self.all_waypoints[self.current_wp][0]
        target_easting=self.all_waypoints[self.current_wp][1]
        print(target_northing,",",target_easting)
        target_altitude = 3.0
        self.target_position[0]=target_northing
        self.target_position[1]=target_easting
        self.cmd_position(target_northing,target_easting,target_altitude,0)
        
        self.flight_state=States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
    
