import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        print('local position', self.local_position)
        print('global position', self.global_position)

        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        print('global position', self.global_position)
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # Read lat0, lon0 from colliders into floating point values
        first_row = np.loadtxt('colliders.csv', delimiter=',', dtype='str', usecols=(0,1))[0]
        lat0 = float(first_row[0][5:])
        lon0 = float(first_row[1][5:])

        print('Lattitude: {}, Longitude: {}'.format(lat0, lon0))
        
        # set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0.)

        # TODO: retrieve current global position ? 
        # Convert to current local position using global_to_local()
        local_north, local_east, _ = global_to_local(self.global_position, self.global_home)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Convert start position to current position rather than map center
        grid_start_north = int(np.round(local_north - north_offset))
        grid_start_east = int(np.round(local_east - east_offset))
        grid_start = (grid_start_north, grid_start_east)
        print("Local North: {}".format(local_north))
        print("Local East: {}".format(local_east))
        print("Grid Start: {}".format(grid_start))
        print("Grid Size North: {}".format(grid.shape[0]))
        print("Grid Size East: {}".format(grid.shape[1]))
        print("North Max: {}".format(np.ceil(np.max(data[:, 0] + data[:, 3]))))
        print("East Max: {}".format(np.ceil(np.max(data[:, 1] + data[:, 4]))))

        
        # Set goal as some arbitrary position on the grid
        '''
        while True: 
            rand_goal_n = int(np.random.randint(0.2*grid.shape[1], 0.8*grid.shape[0]))
            #rand_goal_n = rand_goal_n - 1
            rand_goal_e = int(np.random.randint(0.2*grid.shape[1], 0.8*grid.shape[1]))
            if grid[rand_goal_n][rand_goal_e] == 0:
                grid_goal = (rand_goal_n, rand_goal_e)
                break
        '''


        # Adapt to set goal as latitude / longitude position and convert
        #global_goal = [-122.3972262, 37.7926609, 0.] #+20m test position
        global_goal = [-122.397550, 37.790777, 0.] #test position south behind building
        global_goals = np.asarray([
            [-122.397451, 37.792480, 0.],
            [-122.400499, 37.792931, 0.],
            [-122.400370, 37.794665, 0.],
            [-122.398914, 37.796600, 0.],
            [-122.395629, 37.795455, 0.],
            [-122.393286, 37.792623, 0.],
            [-122.394895, 37.791220, 0.],
            [-122.397824, 37.790509, 0.],
            [-122.401280, 37.790414, 0.]])

        random_goal = global_goals[np.random.randint(0, len(global_goals))]

        #grid_goal = (468, 293) # test case to show issue with many waypoints

        print('Random Target Position: ', random_goal)


        local_goal_north, local_goal_east, _ = global_to_local(random_goal, self.global_home)
        grid_goal = (int(np.round(grid_start_north + local_goal_north)), int(np.round(grid_start_east + local_goal_east)))


        print('Local Goal: ', grid_goal)

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation (done)
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)


        # TODO: prune path to minimize number of waypoints (done)
        pruned_path = prune_path(path, epsilon=1e-2)
        print("Path length: {} vs. pruned path length: {}.".format(len(path), len(pruned_path)))
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
