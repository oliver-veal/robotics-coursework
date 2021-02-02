##################################################################################
#                                   TASK A   
##################################################################################

class RobotKineClass():

    def __init__(self,link_lengths):

        self.ROSPublishers = set_joint_publisher()

        self.nj = 3    #number of joints
        self.links = link_lengths    # length of links

        ################################################ TASK 1
        #Define DH table for each link. DH_tab in R^njx4
        #d,theta,a,alpha
        self.DH_tab = np.array([[self.links[0], 0., 0., 0.],
                                [0., 0., 0., pi/2.],
                                [0., 0., self.links[1], 0.],
                                [0., 0., self.links[2], 0.]])
        self.joint_types = 'rrr'	# three revolute joints

##################################################################################
#                                   TASK B   
##################################################################################

#DH_params = parameters for link i
#d,theta,a,alpha
def DH_matrix(DH_params):
    d = DH_params[0]
    theta = DH_params[1]
    a = DH_params[2]
    alpha = DH_params[3]
    
    ################################################ TASK 2
    DH_matrix = np.array([[cos(theta) , -sin(theta), 0., a],
                          [sin(theta)*cos(alpha) , cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                          [sin(theta)*sin(alpha) , cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                          [0., 0., 0., 1.]])
    
    return DH_matrix

##################################################################################
#                                   TASK C   
##################################################################################

#Computes Forward Kinematics. Returns 3x1 position vector
    def getFK(self,q):

        T_0_i_1 = np.identity(4)
        for i in range(self.nj):
            
            DH_params = np.copy(self.DH_tab[i,:])
            #print('q',q)
            #print(DH_params)
            if self.joint_types[i] == 'r':
                DH_params[1] = DH_params[1]+q[i]
            elif self.joint_types[i] == 'p':
                DH_params[0] = DH_params[0]+q[i]
            
            T_i_1_i = DH_matrix(DH_params) #Pose of joint i wrt i-1
            ################################################ TASK 3 (replace np.eye(4) with the correct matrices)
            T_0_i = np.matmul(T_0_i_1,T_i_1_i)#Pose of joint i wrt base
            T_0_i_1 = T_0_i
        T_0_n_1 = T_0_i
        DH_params = np.copy(self.DH_tab[self.nj, :])
        T_n_1_n = DH_matrix(DH_params)
        T_0_n = np.matmul(T_0_n_1, T_n_1_n)

        return T_0_n[0:3,3]

##################################################################################
#                                   TASK D   
##################################################################################

    #Check if point is in WS. returns true or false
    def checkInWS(self, P):
        xP, yP, zP = P
        l0, l1, l2 = self.links
        
        ################################################ TASK 4
        val = np.power(xP,2)+np.power(yP,2)+np.power((zP-l0),2)
        r_max = (l1+l2)
        r_min = (l1-l2)

        inWS = True

        if val > r_max**2 or val < r_min**2:
            inWS = False


        return inWS

##################################################################################
#                                   TASK F   
##################################################################################

    # Solve IK gemoetrically. Returns list of all possible solutions
    def getIK(self,P):

        l1 = self.links[0]
        l2 = self.links[1]
        l3 = self.links[2]

        xP = P[0]
        yP = P[1]
        zP = P[2]

        inWS = self.checkInWS(P)

        q = []
        Poses = []

        if not inWS:
            print("OUT OF WS. NO SOLUTION FOUND")
            return q,Poses
        
        ################################################ TASK 6
        q_a = np.zeros(3)
        q_b = np.zeros(3)
        
        r_1 = sqrt(xP**2 + yP**2)
        r_2 = zP - l1
        D=(r_1**2+r_2**2-l2**2-l3**2)/(2*l2*l3)
        
        q_a[0] = np.arctan2(yP,xP) #theta 1
        q_b[0] = np.arctan2(yP,xP) #theta 1

        q_a[2] = np.arctan2(sqrt(1-D**2),D) #theta 3
        q_b[2] = np.arctan2(-sqrt(1-D**2),D) #theta 3
        
        q_a[1] = np.arctan2(r_2,r_1)-np.arctan2(l3*sin(q_a[2]),l2+l3*cos(q_a[2])) #theta 2
        q_b[1] = np.arctan2(r_2,r_1)-np.arctan2(l3*sin(q_b[2]),l2+l3*cos(q_b[2])) # theta 2
        
        q = [q_a, q_b]
        
        Poses = [self.getFK(q_a), self.getFK(q_b)]
        return q, Poses

##################################################################################
#                                   TASK G   
##################################################################################

    # Computes Differential Kinematics
    def getDK(self, q, q_dot):
        q0, q1, q2 = q
        l1, l2, l3 = self.links
        
        ################################################ TASK 7
        
        J_11=-(l1*cos(q1)+l2*cos(q1+q2))*sin(q0)
        J_21=(l1*cos(q1)+l2*cos(q1+q2))*cos(q0)
        J_31=0
        
        J_12=-(l1*sin(q1)+l2*sin(q1+q2))*cos(q0)
        J_22=-(l1*sin(q1)+l2*sin(q1+q2))*sin(q0)
        J_32=l1*cos(q1)+l2*cos(q1+q2)
        
        J_13=-(l2*sin(q1+q2))*cos(q0)
        J_23=-(l2*sin(q1+q2))*sin(q0)
        J_33=l2*cos(q1+q2)
        
        self.Jacobian = np.array([[J_11, J_12, J_13],
                                  [J_21, J_22, J_23],
                                  [J_31, J_32, J_33]])
        x_dot = np.matmul(self.Jacobian, q_dot)
        return x_dot

##################################################################################
#                                   TASK H   
##################################################################################

#Final PID values for 0.01kg End effector mass, Task H
#i_clamp_min and max added to allow adjustment of i values, otherwise default min/max is 0.
#final values p 110, i 0, d 90
#.yaml file
DESE3R:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 100  
  
  # Position Controllers ---------------------------------------
  joint_0_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint_0
    pid: {p: 110, i: 0, d: 90, i_clamp_max: 1000, i_clamp_min: 1000}
  joint_1_position_controller:
    type: effort_controllers/JointPositionController
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

<xacro:property name="ee_mass" value="30"/> # END EFFECTOR MASS CHANGED HERE, ONLY LINE CHANGED FOR THIS TASK.

<xacro:property name="link_length_0" value="${link_length_0_init-case_length}"/>
<xacro:property name="link_length_1" value="${link_length_1_init-case_radius}"/>
<xacro:property name="link_length_2" value="${link_length_2_init-case_radius}"/>

##################################################################################
#                                   TASK J   
##################################################################################

#Final PID values for 30kg End effector mass, Task J
#final values p 3250, i 0, d 1600
#.yaml file
DESE3R:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 100  
  
  # Position Controllers ---------------------------------------
  joint_0_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint_0
    pid: {p: 3250, i: 0, d: 1600, i_clamp_max: 1000, i_clamp_min: 1000}
  joint_1_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint_1
    pid: {p: 3250, i: 0, d: 1600, i_clamp_max: 1000, i_clamp_min: 1000}
  joint_2_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint_2
    pid: {p: 3250, i: 0, d: 1600, i_clamp_max: 1000, i_clamp_min: 1000}