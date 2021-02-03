##################################################################################
#                                   TASK A   
##################################################################################

class RobotKineClass():

    def __init__(self,link_lengths):                            # Initialise the class with link lenghs, DH parameters
                                                                # and joint types of the robot.
        self.ROSPublishers = set_joint_publisher()

        self.nj = 3                                             # Number of joints in the robot.
        self.links = link_lengths                               # Array containing the length of each joint in the robot.

        #Define DH table for each link. DH_tab in R^njx4
        #d,theta,a,alpha
        self.DH_tab = np.array([[self.links[0], 0., 0., 0.],    # NumPy array representation of DH matrix.
                                [0., 0., 0., pi/2.],            # The four columns represent the DH parameters d, theta, a and alpha.
                                [0., 0., self.links[1], 0.],    # Each row contains the DH parameter values for each frame, with the values being
                                [0., 0., self.links[2], 0.]])   # with respect to the previous frame (there are 3 joints and 1 end effector,
                                                                # each having a reference frame attached, so 4 rows total).
        
        self.joint_types = 'rrr'                                # Defining the type of each joint as a string with the ith character
                                                                # representing the type of the ith joint,
                                                                # with r representing revolute and p prismatic; all are revolute in this case.

##################################################################################
#                                   TASK B   
##################################################################################

#DH_params = parameters for link i
#d,theta,a,alpha
def DH_matrix(DH_params):
    d = DH_params[0]        # Unpacking the DH_params array for convenience.
    theta = DH_params[1]
    a = DH_params[2]
    alpha = DH_params[3]
    
    ################################################ TASK 2
    DH_matrix = np.array(
        [[cos(theta), -sin(theta), 0., a],
        [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
        [0., 0., 0., 1.]])
    # NumPy array representation of the standard DH transformation matrix from the previous frame to the current frame,
    # as shown in the above figure.
    # Updated each discrete simulation time interval with current DH parameter values.
    # Note: cos() and sin() are imported explicitly so can be used as shorthand (instead of math.sin() etc).
    
    return DH_matrix

##################################################################################
#                                   TASK C   
##################################################################################

#Computes Forward Kinematics. Returns 3x1 position vector
    def getFK(self,q):

        T_0_i_1 = np.identity(4)                        # Initialise matrix describing transformation from base frame to "previous" frame, to indentity.
        for i in range(self.nj):                        # Loop over all joints.
            
            DH_params = np.copy(self.DH_tab[i,:])       # Copy DH params of ith link from ith row of DH_tab matrix, include all columns.
            #print('q',q)
            #print(DH_params)
            if self.joint_types[i] == 'r':              # Add the current joint angle to the "home" angle.
                DH_params[1] = DH_params[1]+q[i]        # If the joint is a revolute joint, add the joint angle to the theta parameter (the joint is rotating).
            elif self.joint_types[i] == 'p':
                DH_params[0] = DH_params[0]+q[i]        # If prismatic, add the joint position to the d parameter (the joint is sliding).
            
            T_i_1_i = DH_matrix(DH_params)              # Transformation matrix describing pose of joint i w.r.t. joint i-1, obtained from DH_matrix function.
            ############################################# TASK 3 (replace np.eye(4) with the correct matrices)
            T_0_i = np.matmul(T_0_i_1,T_i_1_i)          # Transformation matrix describing pose of joint i w.r.t. base frame, obtained through matrix multiplication as described above.
            T_0_i_1 = T_0_i                             # The base-to-previous-frame matrix now refers to this frame's base transformation matrix, such that next iteration it does indeed refer to the "previous" frame.
        
        T_0_n_1 = T_0_i                                 
        DH_params = np.copy(self.DH_tab[self.nj, :])    # The q array only has 3 joint angles, so the end effector frame is handled seperately at the end.
        T_n_1_n = DH_matrix(DH_params)                  
        T_0_n = np.matmul(T_0_n_1, T_n_1_n)             
                                                        
        return T_0_n[0:3,3]

##################################################################################
#                                   TASK D   
##################################################################################

    # Check if point is in WS. returns true or false
    def checkInWS(self, P):
        xP, yP, zP = P                                          # Unpacking the x, y, z components of vector (point) P.
        l0, l1, l2 = self.links                                 # Unpacking robot link lengths from self.links array.
        
        ################################################ TASK 4
        val = np.power(xP,2)+np.power(yP,2)+np.power((zP-l0),2) # Calculate the square of the distance of the end effector from the center of the spherical workspace.
        r_max = (l1+l2)                                         # Calculate the maximum distance the end effector can be from the center of the workspace.
        r_min = (l1-l2)                                         # Calculate the minimum distance the end effector can be from the center of the workspace.

        inWS = True                                             # Initialise default return value to True.

        if val > r_max**2 or val < r_min**2:                    # If the end effector is outside of the big sphere, or inside the small sphere, it is outside of the workspace.
            inWS = False                                        # Since val is the square of the distance, r_max and r_min must also be squared such that their magnitudes are validly comparable.


        return inWS

##################################################################################
#                                   TASK F   
##################################################################################

    # Solve IK gemoetrically. Returns list of all possible solutions
    def getIK(self,P):

        l1 = self.links[0]          # Unpacking robot link lengths from self.links array.
        l2 = self.links[1]
        l3 = self.links[2]

        xP = P[0]                   # Unpacking the x, y, z components of vector (point) P.
        yP = P[1]
        zP = P[2]

        inWS = self.checkInWS(P)    # Firstly, check that the target end effector position is in the reachable workspace.

        q = []                      # Initialise empty arrays to return the joint angles and the end effector positions these angles imply.
        Poses = []                  # Barring floating point errors the two poses should be identical if the IK is working correctly.

        if not inWS:                # Exit early if no solution is possible.
            print("OUT OF WS. NO SOLUTION FOUND")
            return q,Poses
        
        ################################################ TASK 6
        q_a = np.zeros(3)           # Variables ending in _a represent values for the first possible solution, and _b the second.
        q_b = np.zeros(3)           # Initialise joint angle arrays for the two solutions.
        
        r_1 = sqrt(xP**2 + yP**2)   # The following lines compute the variable values required by the two derivations to compute the joint angles.
        r_2 = zP - l1
        D=(r_1**2+r_2**2-l2**2-l3**2)/(2*l2*l3)
        
        q_a[0] = np.arctan2(yP,xP)  # Theta 1 value from derivation 1.
        q_b[0] = np.arctan2(yP,xP)  # Value is identical to solution 1, as demonstrated in the note above.

        q_a[2] = np.arctan2(sqrt(1-D**2),D)     # Theta 3 value from derivation 1.
        q_b[2] = np.arctan2(-sqrt(1-D**2),D)    # Theta 3 value from derivation 2.
        
        q_a[1] = np.arctan2(r_2,r_1)-np.arctan2(l3*sin(q_a[2]),l2+l3*cos(q_a[2]))   # Theta 2 value from derivation 1.
        q_b[1] = np.arctan2(r_2,r_1)-np.arctan2(l3*sin(q_b[2]),l2+l3*cos(q_b[2]))   # Theta 2 value from derivation 2.
        
        q = [q_a, q_b]  # Return the calculated joint angles and poses in an array, as required by the function.
        Poses = [self.getFK(q_a), self.getFK(q_b)]  

        return q, Poses

##################################################################################
#                                   TASK G   
##################################################################################

    # Computes Differential Kinematics
    def getDK(self, q, q_dot):
        q0, q1, q2 = q                                  # Unpack the q and self.links arrays for convenience.
        l1, l2, l3 = self.links
        
        ################################################# TASK 7
        
        J_11=-(l1*cos(q1)+l2*cos(q1+q2))*sin(q0)        # Compute each element of the Jacobian matrix, as defined in the derivation above.
        J_21=(l1*cos(q1)+l2*cos(q1+q2))*cos(q0)
        J_31=0
        
        J_12=-(l1*sin(q1)+l2*sin(q1+q2))*cos(q0)
        J_22=-(l1*sin(q1)+l2*sin(q1+q2))*sin(q0)
        J_32=l1*cos(q1)+l2*cos(q1+q2)
        
        J_13=-(l2*sin(q1+q2))*cos(q0)
        J_23=-(l2*sin(q1+q2))*sin(q0)
        J_33=l2*cos(q1+q2)
        
        self.Jacobian = np.array([[J_11, J_12, J_13],   # Create a matrix using the above elements.
                                  [J_21, J_22, J_23],
                                  [J_31, J_32, J_33]])
        x_dot = np.matmul(self.Jacobian, q_dot)         # Calculates end effector velocity vector by multiplying the Jacobian with the join velocity vector.
        return x_dot

##################################################################################
#                                   TASK H   
##################################################################################

#.yaml file
DESE3R:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 100  
  
  # Position Controllers ---------------------------------------        # Note: PID values are identical for each joint.
  joint_0_position_controller:                                          # Final PID values for 0.01kg end effector mass, Task J.
    type: effort_controllers/JointPositionController                    # The values are p=3250, i=0, d=1600.
    joint: joint_0
    pid: {p: 110, i: 0, d: 90, i_clamp_max: 1000, i_clamp_min: 1000}    # Define p, i and d values for this joint.
  joint_1_position_controller:                                          # Note: i_clamp_max and i_clamp_min are set here such that the i value
    type: effort_controllers/JointPositionController                    # can be set and is not clamped to 0, as is the default behaviour in ROS.
    joint: joint_1
    pid: {p: 110, i: 0, d: 90, i_clamp_max: 1000, i_clamp_min: 1000}
  joint_2_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint_2
    pid: {p: 110, i: 0, d: 90, i_clamp_max: 1000, i_clamp_min: 1000}
      
##################################################################################
#                                   TASK I   
##################################################################################

#.xacro file
<?xml version="1.0"?>
<robot name="DESE3R" xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:property name="link_length_0_init" value="1"/>
<xacro:property name="link_length_1_init" value="1"/>
<xacro:property name="link_length_2_init" value="1"/>

<xacro:property name="link_radius" value="0.05"/>
<xacro:property name="link_mass" value="0.5"/>
<xacro:property name="base_height" value="0.1"/>
<xacro:property name="base_side" value="0.6"/>
<xacro:property name="case_radius" value="${link_radius}"/>
<xacro:property name="case_length" value="${case_radius*2}"/>
<xacro:property name="ee_length" value="0.04"/>
<xacro:property name="ee_side" value="0.02"/>

<xacro:property name="ee_mass" value="30"/> # End effector mass defined here, no other lines changed for this task.

<xacro:property name="link_length_0" value="${link_length_0_init-case_length}"/>
<xacro:property name="link_length_1" value="${link_length_1_init-case_radius}"/>
<xacro:property name="link_length_2" value="${link_length_2_init-case_radius}"/>

##################################################################################
#                                   TASK J   
##################################################################################

#.yaml file
DESE3R:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 100  
  
  # Position Controllers ---------------------------------------        # Note: PID values are identical for each joint.
  joint_0_position_controller:                                          # Final PID values for 30kg end effector mass, Task J.
    type: effort_controllers/JointPositionController                    # The values are p=3250, i=0, d=1600.
    joint: joint_0
    pid: {p: 3250, i: 0, d: 1600, i_clamp_max: 1000, i_clamp_min: 1000} # Define p, i and d values for this joint.
  joint_1_position_controller:                                          # Note: i_clamp_max and i_clamp_min are set here such that the i value
    type: effort_controllers/JointPositionController                    # can be set and is not clamped to 0, as is the default behaviour in ROS.
    joint: joint_1
    pid: {p: 3250, i: 0, d: 1600, i_clamp_max: 1000, i_clamp_min: 1000}
  joint_2_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint_2
    pid: {p: 3250, i: 0, d: 1600, i_clamp_max: 1000, i_clamp_min: 1000}