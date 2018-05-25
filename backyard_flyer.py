import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


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
        self.square=[(10,0, 5, 0),
        (10,10, 5, 0),
        (0,10, 5, 0),
        (0,0, 5, 0),]
        self.corner=-1

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == States.TAKEOFF :

            # coordinate conversion 
            altitude = -1.0 * self.local_position[2]

            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                self.waypoint_transition()

        elif self.flight_state == States.WAYPOINT:
            front=self.local_position[0]
            right=self.local_position[1]
            altitude=self.local_position[2]*-1.0
            print("---------***----------")
            print(front)
            print(self.target_position[0])
            print(right)
            print(self.target_position[1])
            print(altitude)
            print(self.target_position[2])
            print(front > 0.95 * self.target_position[0] and front < 1.05 * self.target_position[0])
            print(right > 0.95 * self.target_position[1]and right < 1.05 * self.target_position[1])
            print(altitude > 0.95 * self.target_position[2] and altitude < 1.05 * self.target_position[2])
            print("---------***----------")

            if front > self.target_position[0]-0.5 and right > self.target_position[1]-0.5 and altitude >self.target_position[2]-0.5 and front < self.target_position[0]+0.5 and right < self.target_position[1]+0.5 and altitude <  self.target_position[2]+0.5:
                if self.corner < (len(self.square)-1):
                        self.waypoint_transition()
                else:
                    self.landing_transition()


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
        self.corner += 1
        print(self.corner)
        return self.square[self.corner]

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
        print("waypoint transition")
        self.target_position = self.calculate_box()
        print(self.target_position)
        self.cmd_position(*self.target_position)
        self.flight_state = States.WAYPOINT


    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):

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
