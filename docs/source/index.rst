.. DE3 Robotics Coursework documentation master file, created by
   sphinx-quickstart on Wed Jan 27 19:02:39 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

========================================================
Welcome to DE3 Robotics Group 1 Coursework Documentation
========================================================

We are the **BROBOTICISTS** and this is our journey.

We have decided to create a Readthedocs page for our coursework submissions instead of a Google Drive document. This way this documentation can be used for teaching ROS at any point in the future, as well as integrated code blocks and interactive videos.

.. warning::

    For an epic ROS gamer UI experience, please use the web_ version instead of the PDF. Thanks!

.. _web: https://robotics-coursework-de3.readthedocs.io/en/latest/#

============
Introduction
============

**Login for VM machine**

Password: deniro123


Open Terminator to input commands to run different services or files

.. note::
   Top Tip! The shortcut for opening new terminal instances:

   - ‘Ctrl+Shift+o’ - split horizontally
   - ‘Ctrl+Shift+e’ - split vertically

================================================
Starting the Virtual Machine and Troubleshooting
================================================

Troubleshooting notes here

==================
Forward Kinematics
==================

-------------------------------
Task A: Computing the D-H Table
-------------------------------

This task initialises the Denavit-Hartenberg, D-H, table. The table contains all the necessary information to orientate each link of the robot in a consistent manner so that the position of each link can be found relative to the other. As the robot moves, the D-H table is updated. The D-H table is a convenient way to store this information as the transformation matrix for each link can be evaluated using the corresponding row in the table.

The numpy array, self.DH_tab, in line 230 of kinematics.py is the live D-H table for the robot. It’s initial state is entered, with link lengths stored in self.links[n], where n is link number. This refers to a list self.links created in line 225. The list is based on the link_lengths variable which is passed into the function. This function is the __init__ of the ``RobotKineClass()`` class, meaning it is called when the robot object is created when the code is run. The list of link lengths is passed in as the only parameter when the ``RobotKineClass()`` is called.

.. code-block:: python
   :linenos:
   :emphasize-lines: 13-16

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

Derivation
----------------------------

Insert images showing derivation

.. image:: img/task_a_robot.png
   :width: 500
   :alt: Robot Diagram

*Diagram of robot with reference axes [1]*

*Table 1: D-H Table based on derivation*

+-------+--------+--------+--------+--------+
| link  | d      | theta  | a      | alpha  |
+=======+========+========+========+========+
| 1     | l0     | theta1 | 0      | 0      |
+-------+--------+--------+--------+--------+
| 2     | 0      | theta2 | 0      | pi/2   |
+-------+--------+--------+--------+--------+
| 3     | 0      | theta3 | l1     | 0      |
+-------+--------+--------+--------+--------+
| 4     | 0      | 0      | l2     | 0      |
+-------+--------+--------+--------+--------+

The function getFK(self,q) accesses the DH_tab a row at a time and copies the ith row to DH_params to compute the transformation matrix from one frame to the next.
A list q of current joint angles (theta) is passed in, and added to the base-state DH_params, in order to compute the current transformation matrix.
This is how the DH table is “updated” with changing theta.

.. code-block:: python
   :linenos:

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

Therefore, since theta in the D-H table is updated by iteration at each step rather than updating a variable, theta1, theta2, and theta3 are set to 0.
This is because the base configuration of the robot is such that the ith joints are in line with each other which results in the angles being set to 0.
These results are entered into the D-H table defined in the code.

.. code-block:: python
   :linenos:

   self.DH_tab = np.array([[self.links[0], 0., 0., 0.],
                        [0., 0., 0., pi/2.],
                        [0., 0., self.links[1], 0.],
                        [0., 0., self.links[2], 0.]])

----------------------------
Task B: Coding the D-H Table
----------------------------

The numpy array, DH_matrix, in line 211 represents the transformation matrix from frame i-1 to frame i.
As such, the transformation matrices of each joint can be multiplied in a chain to transform coordinates from the end effector frame to the base frame.
This is taken care of in the next section, this function DH_matrix() only creates the i-1 to i matrix.
The definition for this is shown below, and simply written in Python

.. image:: img/transform.png
   :width: 500
   :alt: i-1 to i transformation matrix

.. code-block:: python
   :linenos:
   :emphasize-lines: 9-12
   
   def DH_matrix(DH_params):
      d = DH_params[0]
      theta = DH_params[1]
      a = DH_params[2]
      alpha = DH_params[3]
      
      ################################################ TASK 2
      DH_matrix = np.array(
      [[cos(theta) , -sin(theta), 0, a],                     
      [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
      [sin(theta)*sin(alpha) ,cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
      [0., 0., 0., 1.]])

------------------------------------
Task C: Computing Forward Kinematics
------------------------------------

This function gets the forward kinematics, FK, of the robot as any given instance.
It involves multiplying each on the link transformation matrices together to make the compound transformation matrix.
This is done iteratively in a for loop, beginning line 4. The matrix is updated in line 17.

.. code-block:: python
   :linenos:
   :emphasize-lines: 4, 17

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
               # np.matmul operation here # #Pose of joint i wrt base

               T_0_i_1 = T_0_i
         T_0_n_1 = T_0_i
         DH_params = np.copy(self.DH_tab[self.nj, :])
         T_n_1_n = DH_matrix(DH_params)
         T_0_n = np.matmul(T_0_n_1, T_n_1_n)

The following equation shows how the pose of the end effector can be calculated with respect to the base frame; by multiplying together the transformation matrices of each joint.

.. image:: img/task_c_eq1.png
   :width: 414
   :alt: Task C equation 1

To do this iteratively, premultiply the transformation matrix of the ith frame by the matrix of the i-1th frame, for each frame.

.. image:: img/task_c_eq2.png
   :width: 286
   :alt: Task C equation 2

The following line of code implements this equation inside the loop that iterates over each joint.
Np.matmul is a numpy function that multiplies two matrices together. T_0_i is  the matrix giving the ith joint wrt the base frame and is updated at each iteration.
T_0_i_1 is the matrix from the base frame to the i-1th frame, and T_i_1_i is the matrix from the i-1th frame to the ith frame.
The code iteratively calls the previous function DH_matrix() to get the i-1 to i transformation matrix, T_i_1_i.

.. code-block:: python
   :linenos:
   :emphasize-lines: 17

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
            T_0_i = np.matmul(T_0_i_1, T_i_1_i) #Pose of joint i wrt base

            T_0_i_1 = T_0_i
        T_0_n_1 = T_0_i
        DH_params = np.copy(self.DH_tab[self.nj, :])
        T_n_1_n = DH_matrix(DH_params)
        T_0_n = np.matmul(T_0_n_1, T_n_1_n)

To test the code, open a new terminal and run

.. code-block:: python
   :linenos:

   cd Desktop/DE3Robotics/src/coursework_1/src
   python3 kinematics.py fk 

The FK model runs successfully if the following message appears:

.. code-block:: python
   :linenos:

   "Forward Kinematics calculations are correct, well done!"

==================
Inverse Kinematics
==================

-----------------------------------------------
Task D: Checking if a point is in the workspace
-----------------------------------------------

To convert coordinates from task space to joint space, the geometric inverse kinematics of the robot arm must be calculated.
The diagram demonstrates the home position of the robot arm.
In the home position, the first link (l0), is along the z0 axis of the frame, and the second and third link (l1, l2 respectively) are defined along the y0 plane, meaning joint angles (q0, q1 and q2) are measured relative to this.

Before computing the inverse kinematics of the arm, we must check if a given point is inside its reachable workspace. This is defined by a spherical shell centered on the end of its first link.

MATHEMATICAL EXPLANATION REQUIRED.

.. image:: img/reachable.png
   :width: 500
   :alt: Diagram of reachable space

The following code completes the calculation for the minimum and maximum sphere that defines the reachable workspace, with val, r_max and r_min needing to be entered:

.. code-block:: python
   :linenos:

    #Check if point is in WS. returns true or false
    def checkInWS(self, P):
        xP, yP, zP = P
        l0, l1, l2 = self.links
        
        ################################################ TASK 4
        val = 0.
        r_max = 0.
        r_min = 0.

        inWS = True

        if val > r_max**2. or val < r_min**2.:
            inWS = False

        return inWS

The r_max and r_min are defined as:

.. image:: img/task_d.png
   :width: 252

Therefore, r_max and r_min are defined in the code as shown below and the **2 is removed in the if statement to remove the terms being squared twice.

.. code-block:: python
   :linenos:

   #Check if point is in WS. returns true or false
   def checkInWS(self, P):
      xP, yP, zP = P
      l0, l1, l2 = self.links
      
      ################################################ TASK 4
      val = np.power(xP, 2) + np.power(yP, 2) + np.power((zP - l0), 2)
      r_max = np.power((l1 + l2), 2)
      r_min = np.power((l1 - l2), 2)

      inWS = True

      if val > r_max or val < r_min:
         inWS = False


      return inWS

--------------------------------------
Task E: Calculating Inverse Kinematics
--------------------------------------

Inverse kinematics are used to obtain the joint angles required to reach a desired end effector position.
For this, we need equations for q0, q1, q2 in terms of lengths and coordinates.

Derivation 1
------------

Derivation 2
------------


---------------------------------
Task F: Coding Inverse Kinematics
---------------------------------

Using the derivations from Task E, the code can be populated with the definitions for q.

Given any feasible point, there may be two configurations to reach it.
These two sets of configurations are stored in 3x1 vectors q_a and q_b, containing qa0, qa1, qa2 and qb0, qb1, qb2 respectively.

The robot can be treated as a planar two link robot, being rotated about z0 by q0.
Therefore, for any point there is only one q0 value which satisfies.
The two configurations are stored together as columns in matrix q.

Note: Due to the test point error at the time, three different derivations were used with additional custom test points, verified by the FK model.
This was  used to correct the test points.
Results with corrected test points shown.

Unsolved code to fill:

.. code-block:: python
   :linenos:

   q_a = np.zeros(3)
   q_b = np.zeros(3)

   q_a[0] = 0.
   q_b[0] = 0.

   r = 0.
   z = 0.

   q_a[2] = 0.
   q_b[2] = 0.

   q_a[1] = 0.
   q_b[1] = 0.
   
   q = [q_a, q_b]

Code from derivation 1:

.. code-block:: python
   :linenos:

   theta_1 = np.arctan2(yP, xP)
   r_1 = sqrt(xP**2 + yP**2)
   r_2 = zP - l1
   phi_2 = np.arctan2(r_2, r_1)
   r_3 = sqrt(r_1**2 + r_2**2)
   phi_1 = np.arccos((l3**2 - l2**2 - r_3**2) / (-2 * l2 * r_3))
   theta_2 = phi_2 - phi_1
   phi_3 = np.arccos((r_3**2 - l2**2 - l3**2) / (-2 * l2 * l3))
   theta_3 = pi - phi_3

   
   q_a[0] = theta_1 #np.arctan2(yP,xP)
   q_b[0] = theta_1 #np.arctan2(yP,xP)

   #r = np.power(np.power(xP,2)+np.power(yP,2),0.5)
   #z = zP-l1

   q_a[2] = pi-phi_3
   q_b[2] = -pi+phi_3        
   q_a[1] = phi_2-phi_1
   q_b[1] = phi_2+phi_1
   
   q = [q_a, q_b]

Code from derivation 2:

.. code-block:: python
   :linenos:

   theta_1 = np.arctan2(yP, xP)
   r_1 = sqrt(xP**2 + yP**2)
   r_2 = zP - l1
   
   D=(r_1**2+r_2**2-l2**2-l3**2)/(2*l2*l3)
   
   q_a[0] = theta_1 #np.arctan2(yP,xP)
   q_b[0] = theta_1 #np.arctan2(yP,xP)

   #r = np.power(np.power(xP,2)+np.power(yP,2),0.5)
   #z = zP-l1

   q_a[2] = np.arctan2(sqrt(1-D**2),D)
   q_b[2] = np.arctan2(-sqrt(1-D**2),D)
   
   q_a[1] = np.arctan2(r_2,r_1)-np.arctan2(l3*sin(q_a[2]),l2+l3*cos(q_a[2]))
   q_b[1] = np.arctan2(r_2,r_1)-np.arctan2(l3*sin(q_b[2]),l2+l3*cos(q_b[2]))
   
   q = [q_a, q_b]

Forward Kinematics Tester to validate solution (confirm error in test points)

.. code-block:: python
   :linenos:

   elif task=="test":
      point = [0.2, 0.5, 0.7]

      joint_angles = Robot.getIK(point)[0]
      test_point_1 = Robot.getFK(joint_angles[0])
      test_point_2 = Robot.getFK(joint_angles[1])

   print(point, test_point_1, test_point_2) 

.. code-block:: python
   :linenos:

   tasks = ['fk', 'ws', 'ik', 'dk', 'full', 'test']

Incorrect test points:

+-------+-------+-------+-------+-------+-------+-------+-------+-------+
| x     | y     | z     | q1    | q2    | q3    | q1    | q2    | q3    |
+=======+=======+=======+=======+=======+=======+=======+=======+=======+
| 2     | 0     | 1     | 0     | 0     | 0     | 0     | 0     | 0     |
+-------+-------+-------+-------+-------+-------+-------+-------+-------+
| 0     | 0     | 3     | 0     | 1.571 | 0     | 0     | 1.571 | 0     |
+-------+-------+-------+-------+-------+-------+-------+-------+-------+
| 1.648 | 0.9   | 0.521 | 0.5   | -0.5  | 0.5   | 0.5   | 0.5   | -0.5  |
+-------+-------+-------+-------+-------+-------+-------+-------+-------+
| 1.75  | 0     | 1     | 0     | 0     | 0     | 0     | 0     | 0     |
+-------+-------+-------+-------+-------+-------+-------+-------+-------+

Corrected test points:

+-------+-------+-------+-------+-------+-------+-------+-------+-------+
| x     | y     | z     | q1    | q2    | q3    | q1    | q2    | q3    |
+=======+=======+=======+=======+=======+=======+=======+=======+=======+
| 2     | 0     | 1     | 0     | 0     | 0     | 0     | 0     | 0     |
+-------+-------+-------+-------+-------+-------+-------+-------+-------+
| 0     | 0     | 3     | 0     | 1.571 | 0     | 0     | 1.571 | 0     |
+-------+-------+-------+-------+-------+-------+-------+-------+-------+
| 1.648 | 0.9   | 0.521 | 0.5   | -0.5  | 0.5   | 0.5   | 0     | -0.5  |
+-------+-------+-------+-------+-------+-------+-------+-------+-------+
| 1.75  | 0     | 1     | 0     | -0.505| 1.011 | 0     | 0.505 | -1.011|
+-------+-------+-------+-------+-------+-------+-------+-------+-------+

=======================
Differential Kinematics
=======================

-------------------------------
Task G: Computing the Jacobian
-------------------------------

The Jacobian is a matrix of first order partial derivatives, in this case of x, y and z (columns) with respect to q1, q2, and q3 (rows).
This allows task space velocity to be calculated from joint space velocities, as shown in the relationship below.

.. image:: img/jacobian.png
   :width: 299

With the provided equations, these can be expanded to isolate the different theta_dot multipliers (elements in the Jacobian)

.. image:: img/jacobian_equations.png
   :width: 500
   
These equations can be used to define the matrix in the code below, with red green and blue indicating the q0_dot, q1_dot and q2_dot components respectively.

.. code-block:: python
   :linenos:

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

=============
Robot Control
=============

-----------------------------------------------
Task H: Tuning Controller Gains
-----------------------------------------------

The robot arm uses a simple PID controller to reach desired joint positions.

SHORT EXPLANATION OF PID.

.. image:: img/pid.png
   :width: 497

The robot arm can be moved by running the following line in Terminator. 

.. code-block:: python
   :linenos:

   cd Desktop/DE3Robotics/src/coursework_1/src 
   python3 kinematics.py full

This in turn eventually calls the function sendCommands() which communicates with Gazebo using a .publish() method.

.. code-block:: python
   :linenos:

   def sendCommands(self,q):

      #print("SENDING JOINT VALUES ", q)
      rate = rospy.Rate(100) #Hz
      for i in range(3):

         n_conn = 0
         while not n_conn:
               self.ROSPublishers[i].publish(q[i])
               n_conn = self.ROSPublishers[i].get_num_connections()
               rate.sleep()

Each joint has its own PID control parameters. These can be tuned using *method maybe*

Table of results

Embedded video
 
To begin tuning the robotic arm, the starting parameters for the proportional, integral and derivative terms were 0. The outcome of the is 

Errors are huge as it cannot move at all because default controllers gains are very low. Errors changing as we send it different commands.

------------------------------
Task I: Adapting the Robot Arm
------------------------------

The end effector mass is controlled in line _ of the robot_model_gazebo.xacro file, which contains all the physical model parameters. Changing the end effector mass to 30Kg yields the following results.

Results

Embedded video

---------------------------------
Task J: Adapting Controller Gains
---------------------------------

Using the same methods as in Task H, the PID values are updated to account for the new response of the system.

Results.

Embedded video




