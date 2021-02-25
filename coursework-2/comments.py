##################################################################################
#                                   TASK A.i  
##################################################################################

def expand_map(img, robot_width):
    robot_px = int(robot_width * scale)   # size of the robot in pixels x axis
    ############################################################### TASK A
    # SQUARE MASK
    # create a square array of ones of the size of the robot
    robot_mask = np.ones((robot_px, robot_px)) # Creates a 2D array of size robot_px * robot_px of ones

    expanded_map = binary_dilation(img, robot_mask) # Dilate the obstacle map by the square mask and return the result.
    
    return expanded_map
 

##################################################################################
#                                   TASK A.ii   
##################################################################################

def circle_array(arr, r): # Basic implementation of a quantised "circle"
    width = len(arr) # What dimension is our square input array?
    offset = math.ceil(-(width / 2)) + (0.5 if width % 2 == 0 else 0) # This offset value accounts for i, j coords ranging from 0 -> i 
                                                                      # and the circle having its origin at 0, 0 and ranging from -r, r
    r2 = r**2 
    for i in range(width):
        for j in range(width): # Loop over the 2D array
            if (i+offset)**2 + (j+offset)**2 > r2: # Circle equation x^2 + y^2 = r^2. If left > right, then we are in an array element outside of the circle, so set it's value to 0.
                arr[i][j] = 0

def expand_map(img, robot_width):
    robot_px = int(robot_width * scale)   # size of the robot in pixels x axis
    ############################################################### TASK A
    # SQUARE MASK
    robot_mask = np.ones((robot_px, robot_px))
    
    # CIRCULAR MASK
    circle_array(robot_mask, robot_px / 2) # Since circle_array modifies its input array in-place, we simply run it through the function
    expanded_map = binary_dilation(img, robot_mask) # Dilate the map using the circular mask.
    
    return expanded_map 

##################################################################################
#                                   TASK B.i  
##################################################################################

def setup_waypoints(self):
    ############################################################### TASK B
    # Create an array of waypoints for the robot to navigate via to reach the goal    
    waypoints = np.array([[1.97, -3.78], [5.97, -1.28],  [7.78, 4.59]]) # Waypoints were converted manually to world coords an listed in this array

    waypoints = np.vstack([initial_position, waypoints, self.goal]) # Add in the start position and goal to the list of waypoints to make a complete path
    pixel_waypoints = self.map_position(waypoints) # Convert the waypoints to map coords
    
    print('Waypoints:\n', waypoints) # Print for convenience in testing
    print('Waypoints in pixel coordinates:\n', pixel_waypoints)
    
    self.waypoints = waypoints # Sets the global values for Gazebo to access
    self.waypoint_index = 0

##################################################################################
#                                   TASK B.ii   
##################################################################################

def setup_waypoints(self):
     ... 

    # A quick function to calculate the length of the path
    total_len = 0 # Initilise a counter variable
    for i in range(len(waypoints) - 1): # For each waypoint (we don't include the last waypoint here because in the next line we take to the distance to the next waypoint, so we would get an IndexOutOfBounds exception)
        total_len += sqrt((waypoints[i + 1][0] - waypoints[i][0])**2 + (waypoints[i + 1][1] - waypoints[i][1])**2) # Add the distance from this waypoint to the next waypoint to the total
    print(total_len)

    ...

##################################################################################
#                                   TASK C.i   
##################################################################################

def potential_field(self):
      ######################## TASK C Parameter Iterator
      #
      # Brief explanation: What we are doing here is looping over each pixel in the C-space map and calculating the force
      #                    DeNiro would experience there. We then plot the force vectors to a quiver plot using MatPlotLib
      #                    an iterate over a range of K_att & K_rep values to test and visualise their effect.
      
      for i in range(1,50,2):           # We iterate over a reasonable range of K_rep and K_att values to test
         for j in range(50,1000,50):
              
            print(i,j)                  # Print the current parameter combination we are testing for convenience
      
            deniro_position = np.array([0, 0]) # Reset deniro position so we don't accidentally use the global value
                
            map_array_x_combined = [] # Initialise some lists to hold force vectors at each pixel location on the C-space map
            map_array_y_combined = [] # T
                
            resolution = 10 # Step size for iterating over each pixel in the map. We don't want to sample every pixel to save some time.
                
            window = int(320/resolution) # Set up values for proper iteration with a step size of 10
                
            start = -window # Iteration limits in pixel coordinates
            end = window
                
            factor = end/8 # Conversion ratio from pixel coordinates to world coordinates
                
            for y in range(start,end): # Loop over rows in pixel_map. Only sample every [resolution = 10] rows
               map_array_x_combined_row = [] # Stores rows of 
               map_array_y_combined_row = []
                    
               for x in range(start,end): # Loop over columns in a single row of pixel_map. Only sample every [resolution = 10] pixels
                  deniro_position = np.array([x/factor,y/factor])
                
                  ###### Begin potential_field function from motion_planning.py ######
                  goal_vector = goal - deniro_position
                  distance_to_goal = np.linalg.norm(goal_vector)
                  pos_force_direction = goal_vector / distance_to_goal
                        
                  pos_force_magnitude = 1           # By inspection, this part of equation (3) is just a constant
                  K_att = i                         # The positive force scalar parameter to be tuned
                  positive_force = K_att * pos_force_direction * pos_force_magnitude
                  
                  obstacle_pixel_locations = np.argwhere(self.pixel_map == 1)
                  obstacle_pixel_coordinates = np.array([obstacle_pixel_locations[:, 1], obstacle_pixel_locations[:, 0]]).T
                  obstacle_positions = self.world_position(obstacle_pixel_coordinates)
                  
                  obstacle_vector = obstacle_positions - deniro_position 
                  distance_to_obstacle = np.linalg.norm(obstacle_vector, axis=1).reshape((-1, 1))
                  force_direction = obstacle_vector / distance_to_obstacle
                  
                  force_magnitude = -(1/distance_to_obstacle)   # By inspection, this is the part of equation (2), or p(x) = -1/x. 
                  K_rep = j                         # The negative force scalar parameter to be tuned
                  
                  obstacle_force = force_direction * force_magnitude
                  negative_force = K_rep * np.sum(obstacle_force, axis=0) / obstacle_pixel_locations.shape[0]
                  ###### End potential_field function from motion_planning.py ######
                  
                  map_array_x_combined_row.append(negative_force[0] + positive_force[0])
                  map_array_y_combined_row.append(negative_force[1] + positive_force[1])

               map_array_x_combined.append(map_array_x_combined_row) # Having looped over every row, combine all rows into a single 2D array to be plotted.
               map_array_y_combined.append(map_array_y_combined_row)
            
            com_map_x = np.array(map_array_x_combined) # Convert sampled force vectors to an np array to be plotted.
            com_map_y = np.array(map_array_y_combined)

            U = com_map_x # x component of force vector to plot
            V = com_map_y # y component

            plt.imshow(self.pixel_map, vmin=0, vmax=1, origin='lower', cmap='Greys') # Plots the C-space map
            
            res_over_2 = resolution/2 # To account for the coordinate space being +-
            
            X, Y = np.meshgrid(np.arange(0, 320, res_over_2), np.arange(0,320, res_over_2)) # Creates a 2D array of cartesian coordinates properly scaled to account for our step size of [resolution = 10].

            plt.quiver(X,Y,U,V,angles='xy', scale_units='xy', scale=0.2) # Overlays force vectors onto plot of C-space map
            plt.savefig("linear_K_att:"+ str(K_att) + " K_rep:" + str(K_rep) + ".png", dpi=300) # Saves the plot to the disk with a useful title containing the K_att and K_rep params
            plt.clf() # Clears figure for next iteration

##################################################################################
#                                   TASK C.ii   
##################################################################################

def potential_field(self): 
    ############################################################### TASK C
    complete = False

    goal_vector = goal - deniro_position
    distance_to_goal = np.linalg.norm(goal_vector)
    pos_force_direction = goal_vector / distance_to_goal
    
    pos_force_magnitude = 1 # By inspection, this part of equation (3) is just a constant
    K_att = 1 # For the method it seems the best results are when the K params are equal, and because we normalise the output force, it may as well be 1
    
    positive_force = K_att * pos_force_direction * pos_force_magnitude
    
    obstacle_pixel_locations = np.argwhere(self.pixel_map == 1)
    obstacle_pixel_coordinates = np.array([obstacle_pixel_locations[:, 1], obstacle_pixel_locations[:, 0]]).T
    obstacle_positions = self.world_position(obstacle_pixel_coordinates)
    
    obstacle_vector = obstacle_positions - deniro_position
    distance_to_obstacle = np.linalg.norm(obstacle_vector, axis=1).reshape((-1, 1))
    force_direction = obstacle_vector / distance_to_obstacle
    
    force_magnitude = -1 / (distance_to_obstacle) # By inspection, this is the part of equation (2), or p(x) = -1/x. 
    # NOTE To implement Method 3 this line would be: force_magnitude = -1 / (distance_to_obstacle ** 2)
    K_rep = 1 # For the method it seems the best results are when the K params are equal, and because we normalise the output force, it may as well be 1
    
    obstacle_force = force_direction * force_magnitude

    #################################################### Implementation of Methods 1 and 2
    mags = np.linalg.norm(obstacle_force, axis=1) # Get all the magnitudes of the negative forces
    avg = 40                                      # How many of the largest magnitudes should we average over? Top-N Avg, this is the N
    ind = np.argpartition(mags, -avg)[-avg:]      # argpartition gives you the top N values from a list, without sorting the whole list, so it is slightly faster than sorting first then sampling with array indices

    negative_force = K_rep * np.sum(obstacle_force[ind], axis=0) / avg # Implementation of Top-N Avg, sum up the top 40 obstacles forces, then divide by 40
    ####################################################
    
    vref = positive_force + negative_force
    vref = vref / np.linalg.norm(vref)

    if distance_to_goal < 0.05:
        vref = np.array([[0, 0]])
        complete = True

    vref = np.reshape(vref, (-1, 2)) # Sometime there was a weird error where the vref array was the wrong shape so this is a bodge fix

    return vref, complete

##################################################################################
#                                   TASK D.i   
##################################################################################
def generate_random_points(self, N_points):
    N_accepted = 0  # number of accepted samples
    accepted_points = np.empty((1, 2))  # empty array to store accepted samples
    rejected_points = np.empty((1, 2))  # empty array to store rejected samples
    
    while N_accepted < N_points:    # keep generating points until N_points have been accepted
    
        points = np.random.uniform(-10, 10, (N_points - N_accepted, 2))  # generate random coordinates
        pixel_points = self.map_position(points)    # get the point locations on our map

        ########
        rejected = np.array([self.pixel_map[point[1], point[0]] for point in pixel_points])
        # This task can be done in a one-liner:
        # Define the rejected array as either 1 if the corresponding point is rejected, or 0 if the point is accepted.
        # A point is accepted if it's corresponding location in the pixel map is not an obstacle.
        # What's nice is that the obstacle values 0, 1 correspond directly with a point being rejected 0, 1
        #######

        new_accepted_points = pixel_points[np.argwhere(rejected == 0)].reshape((-1, 2))
        new_rejected_points = pixel_points[np.argwhere(rejected == 1)].reshape((-1, 2))
        accepted_points = np.vstack((accepted_points, new_accepted_points))
        rejected_points = np.vstack((rejected_points, new_rejected_points))

##################################################################################
#                                   TASK D.ii   
##################################################################################

######
# While code implementation for this task is not a requirement, here is a basic implementation of creating a route graph using corner detection in SciPy
######

def generate_random_points(self, N_points):
    #NOTE This corner detection method works best when the robot_mask from map.py is a square, not a circle, as the C-space map is produces has sharp corners
    accepted_points = np.empty((1, 2)) # Initialise accepted points

    bigger_map = binary_dilation(self.pixel_map, np.ones((3, 3))) # Slightly expand the C-space map to give some "breathing room" around obstacles (sometimes a valid path cannot be found otherwise)
    bigger_map.astype(int) # Convert to ints for the corner detection function

    ##### from skimage.feature import corner_harris, corner_subpix, corner_peaks
    coords = corner_peaks(corner_harris(bigger_map), min_distance=1, threshold_rel=0.002) # Perform corner detection on the C-space map and return a list of points
    #accepted_points = corner_subpix(bigger_map, coords, window_size=20)   # Optional subpixel corner detection, not used in the case of a rectangular map
    accepted_points = np.flip(coords, axis=1) # The resulting points are in col, row form rather than x, y so we flip them around

    world_points = self.world_position(accepted_points) # calculate the position of the accepted points in world coordinates
    world_points = np.vstack((initial_position, world_points, goal)) # add DE NIRO's position to the beginning of these points, and the goal to the end

    return world_points

##################################################################################
#                                   TASK E.i  
##################################################################################
def create_graph(self, points):
    # Choose your minimum and maximum distances to produce a suitable graph
    mindist = 0.2
    maxdist = 3.8

    # Calculate a distance matrix between every node to every other node
    distances = cdist(points, points)

    # Create two dictionaries
    graph = {}  # dictionary of each node, and the nodes it connects to
    distances_graph = {}    # dictionary of each node, and the distance to each node it connects to

    plt.imshow(self.pixel_map, vmin=0, vmax=1, origin='lower')  # setup a plot of the map

            
    for i in range(points.shape[0]):    # loop through each node
        points_in_range = points[(distances[i] >= mindist) & (distances[i] <= maxdist)]     # get nodes an acceptable distance of the current node
        distances_in_range = distances[i, (distances[i] >= mindist) & (distances[i] <= maxdist)]    # get the corresponding distances to each of these nodes

    if points_in_range.shape[0] > 0:    # if there are any nodes in an acceptable range
        # set up arrays of nodes with edges that don't collide with obstacles, and their corresponding distances
        collision_free_points = np.empty((1, 2))
        collision_free_distances = np.empty((1, 1))

##################################################################################
#                                   TASK E.ii   
##################################################################################

def check_collisions(self, pointA, pointB):
    ############################################################### TASK E ii     
    # Calculate the distance between the two point
    distanceVector = pointB - pointA
    distance = np.linalg.norm(distanceVector)
    # Calculate the UNIT direction vector pointing from pointA to pointB
    direction = distanceVector / distance
    # Choose a resolution for collision checking
    resolution = 0.5   # resolution to check collision to in m

    # Create an array of points to check collisions at
    edge_points = pointA.reshape((1, 2)) + np.arange(0, distance, resolution).reshape((-1, 1)) * direction.reshape((1, 2))
    # Convert the points to pixels
    edge_pixels = self.map_position(edge_points)

    for pixel in edge_pixels:   # loop through each pixel between pointA and pointB
        collision = self.pixel_map[int(pixel[1]), int(pixel[0])]    # if the pixel collides with an obstacle, the value of the pixel map is 1
        if collision == 1:
            return True     # if there's a collision, immediately return True
    return False    # if it's got through every pixel as hasn't returned yet, return False

##################################################################################
#                                   TASK F.i   
##################################################################################

def dijkstra(self, graph, edges):
    ############################################################### TASK F
    goal_node = goal
    nodes = list(graph.keys())
    
    initial_cost = 1000000.0  # This is set to a very high value so that the initial cost is never lower than the first time a node is visited
    
    unvisited = pd.DataFrame({'Node': nodes, 'Cost': [initial_cost for node in nodes], 'Previous': ['' for node in nodes]})
    unvisited.set_index('Node', inplace=True)
    unvisited.loc[[str(initial_position)], ['Cost']] = 0.0
    
    visited = pd.DataFrame({'Node':[''], 'Cost':[0.0], 'Previous':['']})
    visited.set_index('Node', inplace=True)
    
    # Dijkstra's algorithm
    # Theory is explained in depth in the report, comments here only highlight the lines we modified to complete the task
    while str(goal_node) not in visited.index.values:
        
        current_node = unvisited[unvisited['Cost']==unvisited['Cost'].min()]
        current_node_name = current_node.index.values[0]
        current_cost = current_node['Cost'].values[0]
        current_tree = current_node['Previous'].values[0]
        
        connected_nodes = graph[current_node.index.values[0]]
        connected_edges = edges[current_node.index.values[0]]
        
        for next_node_name, edge_cost in zip(connected_nodes, connected_edges):
            next_node_name = str(next_node_name)
            
            if next_node_name not in visited.index.values:
                next_cost_trial = current_cost + edge_cost # This updates the cost by adding the current best path cost to the edge weight
                next_cost = unvisited.loc[[next_node_name], ['Cost']].values[0]
                
                if next_cost_trial < next_cost: # Should we update the value for the cost of the node?
                    unvisited.loc[[next_node_name], ['Cost']] = next_cost_trial
                    unvisited.loc[[next_node_name], ['Previous']] = current_tree + current_node_name
        
        unvisited.drop(current_node_name, axis=0, inplace=True)

        visited.loc[current_node_name] = [current_cost, current_tree]
        
# ... 