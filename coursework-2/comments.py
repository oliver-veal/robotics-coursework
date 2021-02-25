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



##################################################################################
#                                   TASK D.ii   
##################################################################################



##################################################################################
#                                   TASK E.i  
##################################################################################



##################################################################################
#                                   TASK E.ii   
##################################################################################



##################################################################################
#                                   TASK E.iii   
##################################################################################



##################################################################################
#                                   TASK F.i   
##################################################################################



##################################################################################
#                                   TASK F.ii   
##################################################################################