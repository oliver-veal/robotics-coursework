#########TASK A########################################################################
def expand_map(img, robot_width):
    robot_px = int(robot_width * scale)   # size of the robot in pixels x axis
    
    ############################################################### TASK A
    # SQUARE MASK
    # create a square array of ones of the size of the robot
    robot_mask = np.ones(0,0)
    
    # CIRCULAR MASK - optional for individual students
    # create a square array of the size of the robot
    # where a circle the size of the robot is filled with ones

    expanded_img = binary_dilation(img, robot_mask)
    
    plt.imshow(robot_mask, vmin=0, vmax=1, origin ='lower')
    plt.show
    
    return expanded_img

#######################################################################################

#########TASK B########################################################################
    def setup_waypoints(self):
        ############################################################### TASK B
        # Create an array of waypoints for the robot to navigate via to reach the goal
        waypoints = np.array([[0.0, 0.0],
                              [0.0, 0.0],
                              [0.0, 0.0]])  # fill this in with your waypoints

        waypoints = np.vstack([initial_position, waypoints, self.goal])
        pixel_goal = self.map_position(self.goal)
        pixel_waypoints = self.map_position(waypoints)
        
        print('Waypoints:\n', waypoints)
        print('Waypoints in pixel coordinates:\n', pixel_waypoints)
        
        # Plotting
        plt.imshow(self.pixel_map, vmin=0, vmax=1, origin='lower')
        plt.scatter(pixel_waypoints[:, 0], pixel_waypoints[:, 1])
        plt.plot(pixel_waypoints[:, 0], pixel_waypoints[:, 1])
        plt.show()
        
        self.waypoints = waypoints
        self.waypoint_index = 0
#######################################################################################

#########TASK C########################################################################
                ############################################################### TASK C
        complete = False
        
        # compute the positive force attracting the robot towards the goal
        # vector to goal position from DE NIRO
        goal_vector = goal - deniro_position
        # distance to goal position from DE NIRO
        distance_to_goal = np.linalg.norm(goal_vector)
        # unit vector in direction of goal from DE NIRO
        pos_force_direction = goal_vector / distance_to_goal
        
        # potential function
        pos_force_magnitude = 0.0     # your code here!
        # tuning parameter
        K_att = 1.0     # tune this parameter to achieve desired results
        
        # positive force
        positive_force = K_att * pos_force_direction * pos_force_magnitude  # normalised positive force
        
        # compute the negative force repelling the robot away from the obstacles
        obstacle_pixel_locations = np.argwhere(self.pixel_map == 1)
        # coordinates of every obstacle pixel
        obstacle_pixel_coordinates = np.array([obstacle_pixel_locations[:, 1], obstacle_pixel_locations[:, 0]]).T
        # coordinates of every obstacle pixel converted to world coordinates
        obstacle_positions = self.world_position(obstacle_pixel_coordinates)
        
        # vector to each obstacle from DE NIRO
        obstacle_vector = obstacle_positions - deniro_position   # vector from DE NIRO to obstacle
        # distance to obstacle from DE NIRO
        distance_to_obstacle = np.linalg.norm(obstacle_vector, axis=1).reshape((-1, 1))  # magnitude of vector
        # unit vector in direction of obstacle from DE NIRO
        force_direction = obstacle_vector / distance_to_obstacle   # normalised vector (for direction)
        
        # potential function
        force_magnitude = 0.0   # your code here!
        # tuning parameter
        K_rep = 100     # tune this parameter to achieve desired results
        
        # force from an individual obstacle pixel
        obstacle_force = force_direction * force_magnitude
        # total negative force on DE NIRO
        negative_force = K_rep * np.sum(obstacle_force, axis=0) / obstacle_pixel_locations.shape[0]
        
        
        # Uncomment these lines to visualise the repulsive force from each obstacle pixel
        # Make sure to comment it out again when you run the motion planner fully
#        plotskip = 10   # only plots every 10 pixels (looks cleaner on the plot)
#        plt.imshow(self.pixel_map, vmin=0, vmax=1, origin='lower')
#        plt.quiver(obstacle_pixel_coordinates[::plotskip, 0], obstacle_pixel_coordinates[::plotskip, 1],
#                   obstacle_force[::plotskip, 0] * self.xscale, obstacle_force[::plotskip, 1] * self.yscale)
#        plt.show()
#######################################################################################

##################################TASK D#####################################################
############################################################### TASK D
        N_accepted = 0  # number of accepted samples
        accepted_points = np.empty((1, 2))  # empty array to store accepted samples
        rejected_points = np.empty((1, 2))  # empty array to store rejected samples
        
        while N_accepted < N_points:    # keep generating points until N_points have been accepted
        
            points = np.random.uniform(-10, 10, (N_points - N_accepted, 2))  # generate random coordinates
            pixel_points = self.map_position(points)    # get the point locations on our map
            rejected = np.zeros(N_points - N_accepted)   # create an empty array of rejected flags
            
            # Your code here!
            # Loop through the generated points and check if their pixel location corresponds to an obstacle in self.pixel_map
            # self.pixel_map[px_y, px_x] = 1 when an obstacle is present
            # Remember that indexing a 2D array is [row, column], which is [y, x]!
            # You might have to make sure the pixel location is an integer so it can be used to index self.pixel_map
                
            new_accepted_points = pixel_points[np.argwhere(rejected == 0)].reshape((-1, 2))
            new_rejected_points = pixel_points[np.argwhere(rejected == 1)].reshape((-1, 2))
            # keep an array of generated points that are accepted
            accepted_points = np.vstack((accepted_points, new_accepted_points))
            # keep an array of generated points that are rejected (for visualisation)
            rejected_points = np.vstack((rejected_points, new_rejected_points))
            
################################################################################################
            
#####################################TASK E i####################################################
              ############################################################### TASK E i
        # Choose your minimum and maximum distances to produce a suitable graph
        mindist = 0.0
        maxdist = 20.0
        
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
#####################################################################################################
        
#######################################TASK E ii#####################################################
        # Calculate the distance between the two point
        distance = 0.5
        # Calculate the UNIT direction vector pointing from pointA to pointB
        direction = np.array([1.0, 0.0])
        # Choose a resolution for collision checking
        resolution = 5.0   # resolution to check collision to in m
        
        # Create an array of points to check collisions at
        edge_points = pointA.reshape((1, 2)) + np.arange(0, distance, resolution).reshape((-1, 1)) * direction.reshape((1, 2))
        # Convert the points to pixels
        edge_pixels = self.map_position(edge_points)
#####################################################################################################
        
#########################################TASK F#######################################################
            def dijkstra(self, graph, edges):
        ############################################################### TASK F
        goal_node = goal
        nodes = list(graph.keys())
        
        # Create a dataframe of unvisited nodes
        # Initialise each cost to a very high number
        initial_cost = 5.0  # Set this to a suitable value
        
        unvisited = pd.DataFrame({'Node': nodes, 'Cost': [initial_cost for node in nodes], 'Previous': ['' for node in nodes]})
        unvisited.set_index('Node', inplace=True)
        # Set the first node's cost to zero
        unvisited.loc[[str(initial_position)], ['Cost']] = 0.0
        
        # Create a dataframe of visited nodes (it's empty to begin with)
        visited = pd.DataFrame({'Node':[''], 'Cost':[0.0], 'Previous':['']})
        visited.set_index('Node', inplace=True)
        
        # Take a look at the initial dataframes
        print('--------------------------------')
        print('Unvisited nodes')
        print(unvisited.head())
        print('--------------------------------')
        print('Visited nodes')
        print(visited.head())
        print('--------------------------------')
        print('Running Dijkstra')
        
        # Dijkstra's algorithm!
        # Keep running until we get to the goal node
        while str(goal_node) not in visited.index.values:
            
            # Go to the node that is the minimum distance from the starting node
            current_node = unvisited[unvisited['Cost']==unvisited['Cost'].min()]
            current_node_name = current_node.index.values[0]    # the node's name (string)
            current_cost = current_node['Cost'].values[0]       # the distance from the starting node to this node (float)
            current_tree = current_node['Previous'].values[0]   # a list of the nodes visited on the way to this one (string)
            
            connected_nodes = graph[current_node.index.values[0]]   # get all of the connected nodes to the current node (array)
            connected_edges = edges[current_node.index.values[0]]   # get the distance from each connected node to the current node   
            
            # Loop through all of the nodes connected to the current node
            for next_node_name, edge_cost in zip(connected_nodes, connected_edges):
                next_node_name = str(next_node_name)    # the next node's name (string)
                
                if next_node_name not in visited.index.values:  # if we haven't visited this node before
                    
                    # update this to calculate the cost of going from the initial node to the next node via the current node
                    next_cost_trial = 1234  # set this to calculate the cost of going from the initial node to the next node via the current node
                    next_cost = unvisited.loc[[next_node_name], ['Cost']].values[0] # the previous best cost we've seen going to the next node
                    
                    # if it costs less to go the next node from the current node, update then next node's cost and the path to get there
                    if next_cost_trial < next_cost:
                        unvisited.loc[[next_node_name], ['Cost']] = next_cost_trial
                        unvisited.loc[[next_node_name], ['Previous']] = current_tree + current_node_name    # update the path to get to that node
            
            unvisited.drop(current_node_name, axis=0, inplace=True)     # remove current node from the unvisited list

            visited.loc[current_node_name] = [current_cost, current_tree]   # add current node to the visited list
    
    #######################################################################################################################