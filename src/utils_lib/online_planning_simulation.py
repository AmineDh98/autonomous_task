import numpy as np
import math
from ompl import base as ob
from ompl import geometric as og
import dubins 
from scipy.interpolate import interp1d

def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )
def get_angle(p1, p2):
    # Calculate the angle between two points
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])

class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.2, is_unknown_valid=True): ## 0.2, 0.3 for buffle  
        ## is_unknown_valid=True says unkonwn is free initially and explore and find out whether occupied or not. 
        # map: 2D array of integers which categorizes world occupancy
        self.map = None  ## 0, 1000, -1
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # world position of cell (0, 0) in self.map                      
        self.origin = None  ## it will change as robot moves
        # set method has been called                          
        self.there_is_map = False
        # radius arround the robot used to check occupancy of a given position                 
        self.distance = distance                    
        # if True, unknown space is considered valid
        self.is_unknown_valid = is_unknown_valid    
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin): ## set the map every time
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
    
  
  
    
    def is_valid(self, pose):
        """Given a pose, returs true if the pose is not in collision and false othewise.

        Args:
            pose (tuple): position

        Returns:
            boolean: Valid or not
        """
        coord = self.__position_to_map__(pose)
        new_distance = int(self.distance/self.resolution) 
        valid = True
                
        edge_x = coord[0]
        edge_y = coord[1]
        for i in range (edge_x-new_distance, edge_x + new_distance  ):
            for j in range (edge_y - new_distance, edge_y + new_distance ):         
                if i < 0 or j < 0 or i >= (self.map.shape[0]) or j >= (self.map.shape[1]) :
                    return False
                else:
                    if self.map[i,j] >= -1:
                        valid = True
                    if self.map[i,j] >= 50:
                        return False
                              
        return valid
    
    
    def get_closest_obstacle(self, pose):
        """Given a pose, Returns the closest obstacle.

        Args:
            pose (tuple): position

        Returns:
            list: obstacle position
        """
        coord = self.__position_to_map__(pose)
        new_distance = int(self.distance/self.resolution) 
                
        edge_x = coord[0]
        edge_y = coord[1]
        for i in range (edge_x-new_distance, edge_x + new_distance  ):
            for j in range (edge_y - new_distance, edge_y + new_distance ):         
                if self.map[i,j] >= 50:
                    return [i,j]

    # Given a path, returs true if the path is not in collision and false othewise.
    def check_path(self, path, step_size=0.1): 

        # TODO: Discretize the positions between 2 waypoints with step_size
        # TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True. 
        new_path = []
        for i in range(len(path)-1):
            p1 = path[i][0:2] 
            p2 = path[i+1][0:2]
            # print('p', p1, p2)  
            dist = np.linalg.norm(np.array(p2) - np.array(p1))
            num_steps = max(int(dist / step_size), 1)
            for j in range(num_steps+1):
                s = j / num_steps
                new_point = (1-s)*np.array(p1) + s*np.array(p2)
                new_path.append(list(new_point))
        
        # Check if each point is valid
        for point in new_path:
            if not self.is_valid(point):
                return False
        
        return True
    
    # Transform position with respect the map origin to cell coordinates
    def __position_to_map__(self, p):

        # TODO: convert world position to map coordinates. If position outside map return `[]` or `None`
        relative_pos = (np.array([p[0], p[1]]) - self.origin) / self.resolution
        # Round the cell position to get integer indices
        map_pos = np.round(relative_pos).astype(int)
        if np.any(map_pos < 0) or np.any(map_pos >= np.shape(self.map)):
            # Position is outside the map
            return None
        else:
            # Position is within the map
            return map_pos 
    

# Planner: This function has to plan a path from start_p to goal_p. To check if a position is valid the 
# StateValidityChecker class has to be used. The planning dominion must be specified as well as the maximum planning time.
# The planner returns a path that is a list of poses ([x, y]).
def compute_path(start_p, goal_p, state_validity_checker, dominion, max_time=1.0):

    # TODO: Plan a path from start_p to goal_p inside dominion using the OMPL and the state_validity_checker object. Follow notebook example.
    # some code

    print('Start State: ',start_p)
    ret = []
    # create an SE2 state space
    space = ob.RealVectorStateSpace(2)

    # Set the bounds of space to be in low to high.
    space.setBounds(dominion[0], dominion[1])

    # When performing discrete validation of motions, the length of the longest segment 
    # that does not require state validation needs to be specified. 
    # This function sets this length as a fraction of the space's maximum extent. 
    # The call is passed to all contained subspaces.
    space.setLongestValidSegmentFraction(0.0008) 

    # construct an instance of space information from this state space
    si = ob.SpaceInformation(space)
    
    # set state validity checking for this space
    si.setStateValidityChecker(ob.StateValidityCheckerFn(state_validity_checker.is_valid))

    # create a start state
    start = ob.State(space)
    start[0] = start_p[0]
    start[1] = start_p[1]
    
    # create a goal state
    goal = ob.State(space)
    goal[0] = goal_p[0]
    goal[1] = goal_p[1]

    # create a problem instance
    pdef = ob.ProblemDefinition(si)
    
    # set the start and goal states
    pdef.setStartAndGoalStates(start, goal)
    
    # Create the optimization objective. Here it is to optimize the path lenght
    pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(si))
    

    # Construct the optimal planner. An RRT* planner is used.
    optimizingPlanner = og.RRTstar(si)
    #optimizingPlanner = og.RRTConnect(si)
    # optimizingPlanner = og.RRT(si)

    # it represents the maximum length of a motion to be added in the tree of motions.
    optimizingPlanner.setRange(0.1)  
    
    # In the process of randomly selecting states in the state space to attempt to go towards, 
    # the algorithm may in fact choose the actual goal state, if it knows it, with some probability. 
    # This probability is a real number between 0.0 and 1.0; 
    # its value should usually be around 0.05 and should not be too large. 
    # It is probably a good idea to use the default value.
    optimizingPlanner.setGoalBias(0.2) 

    # Set the problem instance for our planner to solve and call setup
    optimizingPlanner.setProblemDefinition(pdef)
    optimizingPlanner.setup()

    # attempt to solve the planning problem in the given runtime
    solved = optimizingPlanner.solve(max_time)
    
    # Get planner data
    pd = ob.PlannerData(si)
    optimizingPlanner.getPlannerData(pd)
    
    if solved:
        # get the path and transform it to a list
        path = pdef.getSolutionPath()
        # print("Found solution:\n%s" % path)
        for i in path.getStates():
            ret.append((i[0], i[1]))
    else:
        print("No solution found")
    # TODO: if solved fill ret with the points [x, y] in the solution path
    # TODO: Ensure that the path brings the robot to the goal (with a small tolerance)!

    return ret 


# Controller: Given the current position and the goal position, this function computes the desired 
# lineal velocity and angular velocity to be applied in order to reah the goal.
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    
    # TODO: Implement a proportional controller which sets a velocity command to move from current position to goal (u = Ke)
    # Make it sequential to avoid strange curves. First correct orientation and then distance. 
    # Hint: use wrap_angle function to maintain yaw in [-pi, pi]
    # This function should return only  linear velocity (v) and angular velocity (w)
    dx = goal[0] - current[0]
    dy = goal[1] - current[1]
    dist = math.sqrt(dx**2 + dy**2)
    angle = wrap_angle(math.atan2(dy, dx) - current[2])

    # Compute linear and angular velocities
    v = Kv * dist
    w = Kw * angle 

    if angle > 0.25 or angle < -0.25:
        v = 0
    return v, w


def velocity_control_with_interpolation(robot_x, robot_y, robot_yaw, path_points, max_velocity = 1, max_acceleration = 0.5, max_angular_velocity = 0.8):
    # Calculate distance between consecutive points in the path
    distances = [math.sqrt((path_points[i+1][0] - path_points[i][0])**2 + (path_points[i+1][1] - path_points[i][1])**2) for i in range(len(path_points) - 1)]

    # Calculate desired velocity profile
    velocity_profile = []
    total_distance = sum(distances)
    accumulated_distance = 0.0
    for distance in distances:
        desired_velocity = min(max_velocity, math.sqrt(2 * max_acceleration * (total_distance - accumulated_distance)))
        velocity_profile.append(desired_velocity)
        accumulated_distance += distance

    # Create time vector based on distance traveled
    time_vector = np.cumsum(distances) / max(max_velocity, 1e-6)  # Prevent division by zero

    # Interpolate between points to obtain a continuous trajectory
    x_points = [point[0] for point in path_points]
    y_points = [point[1] for point in path_points]
    trajectory_x = np.interp(time_vector, time_vector[-1] * np.array([0, 1]), np.array([x_points[0], x_points[-1]]))
    trajectory_y = np.interp(time_vector, time_vector[-1] * np.array([0, 1]), np.array([y_points[0], y_points[-1]]))

    # Calculate desired heading angle at each point in the trajectory
    target_headings = [math.atan2(trajectory_y[i+1] - trajectory_y[i], trajectory_x[i+1] - trajectory_x[i]) for i in range(len(trajectory_x) - 1)]

    # Calculate linear and angular velocities
    v = np.interp(time_vector, time_vector[-1] * np.array([0, 1]), np.array([0, max_velocity]))[:-1]
    w = np.interp(time_vector[:-1], time_vector[-1] * np.array([0, 1]), np.array([0, max_angular_velocity]))

    # Find the desired linear and angular velocities at the current robot position
    min_distance = float("inf")
    target_index = 0
    for i in range(len(trajectory_x)):
        distance = math.sqrt((trajectory_x[i] - robot_x)**2 + (trajectory_y[i] - robot_y)**2)
        if distance < min_distance:
            min_distance = distance
            target_index = i

    desired_v = v[target_index]
    desired_w = w[target_index]

    # Adjust the desired angular velocity based on the desired heading angle
    desired_w += 2.0 * max_velocity * math.sin(target_headings[target_index] - robot_yaw) / min_distance

    return desired_v, desired_w

def compute_dynamic_window(current_pose, current_speed, current_yaw_rate, path, state_validity_checker):
    # Parameters for dynamic window approach
    max_speed = 3  # Maximum linear speed
    max_yaw_rate = 1.8  # Maximum angular speed
    max_accel = 0.5  # Maximum linear acceleration
    max_delta_yaw_rate = 0.44  # Maximum angular acceleration
    dt = 0.1  # Time step for simulation
    v_resolution = 0.1  # Linear velocity resolution
    yaw_rate_resolution = 0.1  # Angular velocity resolution
    v=current_speed
    w=current_yaw_rate
    

    # Create the dynamic window
    v_min = max(0, current_speed - max_accel * dt)
    v_max = min(max_speed, current_speed + max_accel * dt)
    yaw_rate_min = max(-max_yaw_rate, current_yaw_rate - max_delta_yaw_rate * dt)
    yaw_rate_max = min(max_yaw_rate, current_yaw_rate + max_delta_yaw_rate * dt)

    # Find the best trajectory within the dynamic window
    best_trajectory = None
    best_cost = float('inf')
    print("min = ",v_min )
    print("max = ",v_max )
    
    for v in np.arange(v_min, v_max, v_resolution):
        for yaw_rate in np.arange(yaw_rate_min, yaw_rate_max, yaw_rate_resolution):
            trajectory = generate_trajectory(v, yaw_rate, dt, current_pose)
            cost = evaluate_trajectory(trajectory, path, max_speed, state_validity_checker, current_pose, current_speed, max_accel)
            
            

            if cost < best_cost:
                best_cost = cost
                best_trajectory = trajectory

    # Extract linear and angular velocities from the best trajectory
    #print("cost", best_cost)
    print("trajectory", best_trajectory)
    if best_trajectory:
        v = best_trajectory[0][3]
        w = best_trajectory[0][4]
    # else:
    #     print("ok")
        
    print("v = ", v)
    print("w = ", w)
        
    

    return v, w

def generate_trajectory(v, yaw_rate, dt, current_pose):
    # Generate the trajectory using the current velocity and yaw rate
    trajectory = []
    duration = 1.0  # Total duration of the trajectory

    # Simulate the trajectory over time steps
    for t in np.arange(0, duration, dt):
        x = current_pose[0] + v * np.cos(current_pose[2]) * t
        y = current_pose[1] + v * np.sin(current_pose[2]) * t
        yaw = current_pose[2] + yaw_rate * t
        trajectory.append((x, y, yaw,v, yaw_rate))
        
    #print("eachhhhhhhh ",trajectory)

    return trajectory

def evaluate_trajectory( trajectory, path, target_velocity, state_validity_checker, current_pose, current_speed, max_accel):
    # Evaluate the trajectory using a cost function
  # Evaluate the trajectory using a cost function
    if len(path)> 3:
        goal = path[3][0:2]
    else:
        goal = path[0][0:2]  # Goal position

    # Calculate the Euclidean distance between the final position and the goal
    final_pose = trajectory[-1]
    dx = goal[0] - final_pose[0]
    dy = goal[1] - final_pose[1]
    distance_cost = np.sqrt(dx**2 + dy**2)
    
    # dx = goal[0] - trajectory[-1, 0]
    # dy = goal[1] - trajectory[-1, 1]
    # error_angle = math.atan2(dy, dx)
    # cost_angle = error_angle - trajectory[-1, 2]
    # distance_cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    # Calculate the difference in orientation between the final orientation and the goal orientation
    goal_orientation = np.arctan2(dy, dx)
    final_orientation = final_pose[2]
    orientation_cost = np.abs(wrap_angle(goal_orientation - final_orientation))
    
    # Calculate the velocity cost as the difference between the target velocity and the achieved velocity
    achieved_velocity = compute_achieved_velocity(trajectory)
    velocity_cost = (target_velocity - achieved_velocity) # **2
    
    # acceleration_penalty = max(0, achieved_velocity - current_speed) * max_accel  # Penalize exceeding current speed
    # velocity_cost += acceleration_penalty

    
    
    # Calculate the clearance cost as the distance to the closest obstacle on the curvature
    clearance_cost = check_obstacle_distance(trajectory, state_validity_checker, current_pose)

    # Calculate the total cost as a weighted sum of the distance cost and orientation cost
    distance_weight = 6.0
    orientation_weight = 6.0
    velocity_weight = 2.0
    clearance_weight = 0.5
    cost = (
        distance_weight * distance_cost +
        orientation_weight * orientation_cost +
        velocity_weight * velocity_cost +
        clearance_weight * clearance_cost
    )

    return cost


def check_obstacle_distance(trajectory, state_validity_checker, current_pose):
    
    obstacle_distance = 1000  # Initialize obstacle distance to a large constant

    for i in trajectory:
        # yaw_rate = (i + 1) * step_size  # Calculate the yaw rate based on the step number and step size
        # v = 1.0  # Define the velocity for sampling the position
        #trajectory = [(v, yaw_rate)]  # Create a trajectory with the current velocity and yaw rate

        # Sample the position along the curvature based on the current pose and trajectory
        sampled_pose = i
        

        # Check if the sampled position is valid (not occupied by an obstacle)
        if state_validity_checker.is_valid((sampled_pose[0], sampled_pose[1])):
            continue  # Move to the next sampled position

        # Calculate the distance to the obstacle at the sampled position
        dx = current_pose[0] - sampled_pose[0]
        dy = current_pose[1] - sampled_pose[1]
        distance = np.sqrt(dx**2 + dy**2)

        # Update the obstacle distance if the calculated distance is smaller
        if distance < obstacle_distance:
            obstacle_distance = distance

    return obstacle_distance

def compute_achieved_velocity(trajectory):
    # Compute the achieved velocity based on the trajectory
    achieved_velocity = 0.0

    # Accumulate the linear velocities from the trajectory
    for traj in trajectory:
        achieved_velocity += traj[3]

    # Calculate the average achieved velocity
    num_steps = len(trajectory)
    if num_steps > 0:
        achieved_velocity /= num_steps

    return achieved_velocity
# def simulate_trajectory(trajectory, current_pose):
#     # Simulate the trajectory to obtain the final pose

#     for v, yaw, yaw_rate in trajectory:
#         dx = v * np.cos(current_pose[2])
#         dy = v * np.sin(current_pose[2])
#         current_pose[0] += dx
#         current_pose[1] += dy
#         current_pose[2] += yaw

#     return current_pose

def dubins_maz(turning_radius, step_size, rrt_star_path):  
    # Generate Dubins path between adjacent nodes in the RRT*_star* path
        dubins_path = []
        if rrt_star_path == []:
            return dubins_path
        for i in range(len(rrt_star_path)-1):
            q0 = np.array(rrt_star_path[i])
            q1 = np.array(rrt_star_path[i+1]) 
            qs_r, _ = dubins.path_sample(q0, q1, turning_radius, step_size)
            qs = [(t[0], t[1], wrap_angle(t[2])) for t in qs_r] 
            print('qs',qs)
            dubins_path.append(qs) 

        # Concatenate the Dubins paths to form a single path
       
        dubins_path = np.concatenate(dubins_path)
        dubins_path = dubins_path.tolist() 
        return dubins_path

def point_to_pose(rrt_star_path, initial_orientation, final_orientation): # path as a list of tuples  
    angles = []
    angles.append(wrap_angle(initial_orientation)) # initial angle  
    for i in range(len(rrt_star_path)-2):
        dx = rrt_star_path[i+1][0] - rrt_star_path[i][0]
        dy = rrt_star_path[i+1][1] - rrt_star_path[i][1]
        angle = np.arctan2(dy, dx)
        angle = wrap_angle(angle) 
        angles.append(angle) 
    angles.append(wrap_angle(final_orientation))  # final angle  
    # Add orientation angle to each point 
    rrt_path_pose = [(rrt_star_path[i][0], rrt_star_path[i][1], angles[i]) for i in range(len(rrt_star_path))]
    #print('rrt_path_pose', rrt_path_pose)  
    return rrt_path_pose  




def pure_pursuit_control(robot_x, robot_y, robot_yaw, path_points):
    # Parameters
    lookahead_distance = 0.5  # Distance to look ahead on the path
    L = 0.23  # Wheelbase of the robot (distance between front and rear wheels)

    # Find the point on the path closest to the robot
    min_distance = float("inf")
    target_index = 0
    for i in range(len(path_points)):
        point = path_points[i]
        distance = math.sqrt((point[0] - robot_x)**2 + (point[1] - robot_y)**2)
        if distance < min_distance:
            min_distance = distance
            target_index = i

    # Find the target point a certain distance ahead of the robot
    lookahead_index = target_index
    total_distance = 0.0
    while total_distance < lookahead_distance and lookahead_index < len(path_points)-1:
        point = path_points[lookahead_index]
        next_point = path_points[lookahead_index+1]
        dx = next_point[0] - point[0]
        dy = next_point[1] - point[1]
        segment_distance = math.sqrt(dx**2 + dy**2)
        if total_distance + segment_distance > lookahead_distance:
            break
        total_distance += segment_distance
        lookahead_index += 1

    # Calculate the desired heading angle
    alpha = math.atan2(path_points[lookahead_index][1] - robot_y, path_points[lookahead_index][0] - robot_x) - robot_yaw

    # Calculate the steering command (angular velocity)
    Lf = L / math.tan(alpha)
    w = 2.0 * Lf / (lookahead_distance**2)

    # Set a constant forward velocity
    v = 20
     # Limit acceleration
    max_acceleration = 0.6
    max_angular_velocity = 0.5
    dt = 0.1
    v_prev = 0.0  # Previous velocity
    if abs(v - v_prev) > max_acceleration * dt:
        v = v_prev + max_acceleration * dt * math.copysign(1, v - v_prev)

    # Limit angular velocity
    w = max(-max_angular_velocity, min(max_angular_velocity, w))

    return v, w
