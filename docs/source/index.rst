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

=====
Setup
=====

Open Terminator to input commands to run different services or files.

Start by entering ``roscore`` - this is required to launch nodes and programs for the roscore system to start. It must be running in order for ROS nodes to communicate.

To start gazebo, in a new terminal, use ``rosrun gazebo_ros gazebo``, which generates an empty world.

- To close this instance use ``Ctrl+c``. 
- To close the Gazebo properly, in case multiple instances are running and it does not close, run ``killall gzserver gzclient``. The following response should occur if it has closed, ``gzserver: no process found and gzclient: no process found``.

Now it is possible to add the robotic arm to the simulation. In a new terminal run:

- ``cd Desktop/DE3Robotics``
- ``source devel/setup.bash``
- ``roslaunch coursework_1 coursework_1.launch``

The robotic arm should now be in Gazebo

.. note::
   Open Terminator to input commands to run different services or files
   
   Top Tip! The shortcut for opening new terminal instances:

   - ‘Ctrl+Shift+o’ - split horizontally
   - ‘Ctrl+Shift+e’ - split vertically

==================
Forward Kinematics
==================

-------------------------------
Task A: Computing the D-H Table
-------------------------------

This task initialises the Denavit-Hartenberg, D-H, table.
The table contains all the necessary information to orientate each link of the robot in a consistent manner so that the position of each link can be found relative to the other.
As the robot moves, the D-H table is updated.
The D-H table is a convenient way to store this information as the transformation matrix for each link can be evaluated using the corresponding row in the table.

Derivation
----------------------------

Insert images showing derivation

.. image:: img/diagram_with_frames.png
   :width: 500
   :alt: Robot Diagram

*Diagram of robot with D-H conventional reference axes*

*Table 1: D-H Table based on derivation*

+-------+--------+--------+--------+--------+
| link  | d      | θ      | a      | α      |
+=======+========+========+========+========+
| 1     | l0     | theta1 | 0      | 0      |
+=======+--------+--------+--------+--------+
| 2     | 0      | theta2 | 0      | pi/2   |
+-------+--------+--------+--------+--------+
| 3     | 0      | theta3 | l1     | 0      |
+-------+--------+--------+--------+--------+
| 4     | 0      | 0      | l2     | 0      |
+-------+--------+--------+--------+--------+

The numpy array, ``self.DH_tab``, line 13, is the live D-H table for the robot.
Its initial state is entered, with link lengths stored in ``self.links[n]``, where ``n`` is link number.
This refers to a list ``self.links`` created in line 8. The list is based on the ``link_lengths`` variable which is passed into the function.
This function is the ``__init__`` of the ``RobotKineClass()`` class, meaning it is called when the robot object is created when the code is run.
The list of link lengths is passed in as the only parameter when the ``RobotKineClass()`` is called.

.. warning::

   Line numbers in the follow code blocks do not correspond to the line numbers in kinematics.py or any other script mentioned

.. code-block:: python
   :linenos:

    class RobotKineClass():

    def __init__(self,link_lengths):

        self.ROSPublishers = set_joint_publisher()

        self.nj = 3    #number of joints
        self.links = link_lengths    # length of links

        ################################################ TASK 1
        #Define DH table for each link. DH_tab in R^njx4
        #d,theta,a,alpha
        self.DH_tab = np.array([0., 0., 0., 0.],
                                [0., 0., 0., 0.],
                                [0., 0., 0., 0.],
                                [0., 0., 0., 0.]])
        self.joint_types = 'rrr'	# three revolute joints

The function ``getFK(self,q)`` accesses the ``DH_tab`` a row at a time and copies the ith row to ``DH_params`` to compute the transformation matrix from one frame to the next.
A list ``q`` of current joint angles (theta) is passed in, and added to the base-state ``DH_params``, in order to compute the current transformation matrix.
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

.. note::
   
   Our edits and additions to the code are highlighted in the following code blocks

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

----------------------------
Task B: Coding the D-H Table
----------------------------

The numpy array, ``DH_matrix``, line 8, represents the transformation matrix from frame i-1 to frame i.
As such, the transformation matrices of each joint can be multiplied in a chain to transform coordinates from the end effector frame to the base frame.
This is taken care of in the next section, this function ``DH_matrix()`` only creates the i-1 to i matrix by copying in the relavent lines of the D-H table..
The definition for this is shown below

.. image:: img/transform.png
   :width: 500
   :alt: i-1 to i transformation matrix

*definition of i-1 to i th frame transformation matrix* [2]_

This definition is translated into the array in Python.

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

The function ``getFK(self,q)`` gets the forward kinematics, FK, of the robot as any given instance.
It involves multiplying each on the link transformation matrices together to make the compound transformation matrix.
This is done iteratively in a for loop, beginning line 4. The matrix is updated in line 17.

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
               
               T_i_1_i = DH_matrix(DH_params) #Pose of joint i wrt i-1
               
               ################################################ TASK 3 (replace np.eye(4) with the correct matrices)
               # np.matmul operation here # #Pose of joint i wrt base

               T_0_i_1 = T_0_i
         T_0_n_1 = T_0_i
         DH_params = np.copy(self.DH_tab[self.nj, :])
         T_n_1_n = DH_matrix(DH_params)
         T_0_n = np.matmul(T_0_n_1, T_n_1_n)

The following equation shows how the pose of the end effector can be calculated with respect to the base frame by multiplying together the transformation matrices of each joint. [3]_

.. image:: img/task_c_eq1.png
   :width: 414
   :alt: Task C equation 1

To do this iteratively, premultiply the transformation matrix of the ith frame by the matrix of the i-1th frame, for each frame. [4]_

.. image:: img/task_c_eq2.png
   :width: 286
   :alt: Task C equation 2

The following line of code implements this equation inside the loop that iterates over each joint.
``np.matmul`` is a numpy function that multiplies two matrices together. ``T_0_i`` is the matrix giving the ith joint with respect to the base frame and is updated at each iteration.
``T_0_i_1`` is the matrix from the base frame to the i-1th frame, and ``T_i_1_i`` is the matrix from the i-1th frame to the ith frame.
The code iteratively calls the previous function ``DH_matrix()`` to get the i-1 to i transformation matrix, ``T_i_1_i``.

To implement the above equations in the script, ``[T_0_i] = [T_0_i_1][T_i_1_1]``

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
In the home position, the first link **l0**, is along the **z0** axis of the frame, and the second and third link (**l1**, **l2** respectively) are defined along the **y0** plane, meaning joint angles (**q0**, **q1** and **q2**) are measured relative to this.

Before computing the inverse kinematics of the arm, we must check if a given point is inside its reachable workspace. This is defined by a spherical shell centered on the end of its first link.

MATHEMATICAL EXPLANATION REQUIRED. [5]_

.. image:: img/reachable.png
   :width: 500
   :alt: Diagram of reachable space

*diagram of reachable workspace*. [6]_

The following code completes the calculation for the minimum and maximum sphere that defines the reachable workspace, with ``val``, ``r_max`` and ``r_min`` needing to be entered in the following section of code, function ``checkInWS(self, P)``.

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

The ``r_max`` and ``r_min`` are defined as:

.. image:: img/task_d.png
   :width: 252

*definitions of variables* [7]_

Therefore, ``r_max`` and ``r_min`` are defined in the code as shown below and the ``**2`` is removed in the if statement to remove the terms being squared twice.

.. code-block:: python
   :linenos:
   :emphasize-lines: 8,9,13

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
For this, we need equations for ``q0``, ``q1``, ``q2`` in terms of lengths and coordinates.

.. Note::

   Due to the test point error at the time, two different derivations were used with additional custom test points, verified by the FK model.
   This was used to correct the test points.
   Results with corrected test points shown in **Task F**.

Derivation 1 [8]_ [9]_ [10]_
----------------------------

Derivation 2 [11]_
------------------


---------------------------------
Task F: Coding Inverse Kinematics
---------------------------------

Using the derivations from **Task E**, the code can be populated with the definitions for ``q``.

Given any feasible point, there may be two configurations to reach it.
These two sets of configurations are stored in 3x1 vectors ``q_a`` and ``q_b``, containing ``qa0``, ``qa1``, ``qa2`` and ``qb0``, ``qb1``, ``qb2`` respectively.

The robot can be treated as a planar two link robot, being rotated about ``z0`` by ``q0``.
Therefore, for any point there is only one ``q0`` value which satisfies the point.
The two configurations are stored together as columns in matrix ``q``.

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
   :emphasize-lines: 4-12, 14,15, 17,18, 20,21, 23

   q_a = np.zeros(3)
   q_b = np.zeros(3)

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

   q_a[2] = pi-phi_3
   q_b[2] = -pi+phi_3 

   q_a[1] = phi_2-phi_1
   q_b[1] = phi_2+phi_1
   
   q = [q_a, q_b]

Code from derivation 2:

.. code-block:: python
   :linenos:
   :emphasize-lines: 4-6, 8, 10,11, 13,14, 16,17, 19

   q_a = np.zeros(3)
   q_b = np.zeros(3)

   theta_1 = np.arctan2(yP, xP)
   r_1 = sqrt(xP**2 + yP**2)
   r_2 = zP - l1
   
   D=(r_1**2+r_2**2-l2**2-l3**2)/(2*l2*l3)
   
   q_a[0] = theta_1 #np.arctan2(yP,xP)
   q_b[0] = theta_1 #np.arctan2(yP,xP)

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

To validate the Inverse Kinematics model with corrected test points, run:

.. code-block:: python
   :linenos:

   cd Desktop/DE3Robotics/src/coursework_1/src
   python3 kinematics.py ik 

The IK model runs successfully if the following message appears:

.. code-block:: python
   :linenos:

   "Inverse Kinematics calculations are correct, well done!"

=======================
Differential Kinematics
=======================

-------------------------------
Task G: Computing the Jacobian
-------------------------------

The Jacobian is a matrix of first order partial derivatives, in this case of ``x``, ``y`` and ``z`` (columns) with respect to ``q1``, ``q2``, and ``q3`` (rows).
This allows task space velocity to be calculated from joint space velocities, as shown in the relationship below.
This can be simplified using ``J_11``, ``J_12``… ``J_33`` notation in the code provided.

.. code-block:: python
   :linenos:

   def getDK(self, q, q_dot):
      q0, q1, q2 = q
      l1, l2, l3 = self.links
      
      ################################################ TASK 7
      
      J_11=0
      J_21=0
      J_31=0
      
      J_12=0
      J_22=0
      J_32=0
      
      J_13=0
      J_23=0
      J_33=0
      
      self.Jacobian = np.array([[J_11, J_12, J_13],
                                 [J_21, J_22, J_23],
                                 [J_31, J_32, J_33]])
      x_dot = np.matmul(self.Jacobian, q_dot)
      return x_dot

To dervie the elements, we can compare the definition of the Jacobian to expansions of the kinematics equations provided.
The Jacobian, for this case, can be written as:

.. image:: img/jacobian.png
   :width: 299

*Jacobian definition* [12]_

With the provided equations, these can be expanded to isolate the different ``q_dot`` multipliers (elements in the Jacobian) as shown:

.. image:: img/jacobian_equations.png
   :width: 500

*Kinematics equations expanded* [13]_
   
Upon inspection, these equations reveal the definitions of J_11, J_12... and can be used to define the matrix in the code below, with red green and blue indicating the ``q0_dot``, ``q1_dot`` and ``q2_dot`` components respectively.

.. code-block:: python
   :linenos:
   :emphasize-lines: 7-9, 11-13, 15-17

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

To validate the jacobian, run:

.. code-block:: python
   :linenos:

   cd Desktop/DE3Robotics/src/coursework_1/src
   python3 kinematics.py dk

The IK model runs successfully if the following message appears:

.. code-block:: python
   :linenos:

   "Differential Kinematics calculations correct, well done!"

=============
Robot Control
=============

-----------------------------------------------
Task H: Tuning Controller Gains
-----------------------------------------------

The robot arm uses a simple PID controller to reach desired joint positions.

.. image:: img/pid.png
   :width: 497

*PID controller definition for torque output* [14]_

PID controller background [15]_ [16]_ [17]_ [18]_ [19]_ [20]_
-------------------------------------------------------------

The PID controller that is being used is broken up into 3 terms, a **proportional**, **derivative** and **integral** term.
PID gains are reactive and make up what is known as *‘feedback’* control, where the measured output of the system is used to enable better control such that it follows the desired trajectory.
The **proportional** term on its own is measuring the difference between the achieved joint angle and desired joint angle, defined as the error, and then multiplied by a scalar gain value ``Kp``.
It is a measure of *‘system stiffness’* and determines the amount of restoring force needed to overcome simple positional error, with the **proportional** aspect resulting from the ``Kp`` term multiplier.
This will determine how much correction should occur at the output.
The issue with only **P** being active, is that when ``Kp`` is too high, it will result in oscillations of the system that can be potentially destructive given the closed loop.
The video below demonstrates this in action for the robotic arm.

**P only video**

Given that the response is based on previous error, the moving average can also cause issues with oscillation.
Using this controller type on its own can also result in a major drawback of offset, this is when there is a sustained error built up over time that cannot be removed by the **P** on its own.
This is where the **integral** term comes in where it aims to increment or decrement the controller output and drive the error back to zero, rather than letting it persist.

Let’s consider **PI** control which brings the output back to set point, in this case the desired joint angle.
The **integral** term in the controller is determining the sum of error over time of the joint angle and multiplying it by the scalar gain ``Ki``.
This is ideal when the system is experiencing static loads and can be seen that when used, has little to no effect in improving a well defined **P** controller.
This is specific to this simulation scenario however as there is no static load.

**PI video**

Finally, the part of the system that will reduce the oscillations and overshoots caused by the **P** term, the **derivative** control.
This calculates the velocity of the error, which is the same as the difference between the desired joint velocity and achieved joint velocity.
The purpose of this term is to *‘guess’* where the error is going to go and respond accordingly, i.e. if the error is increasing, D will increment the response so that the increase is prevented, then the **PI** control will bring this back to the desired set point joint angle.
However if the **PI** control responds too aggressively, the **D** control will be a brake to the system and subdue to the **PI** response.
This is tuned by the scalar gain ``Kd``.
An example of this can be seen in the video below where a purposefully aggressive **PI** controller has been selected with clear overshoot and oscillation, and then the difference the **derivative** term makes in locking the **PI** response.

.. note::
   If ``Kp`` and ``Kd`` are increased, it will minimise the error more quickly and aggressively, which can be useful for handling against external disturbances, however too high and during movements it will overshoot and result in oscillation (a potentially destructive phenomenon for the robotic arm).
   In the case of this coursework simulation, it is likely ``Kd`` will have minimal effect.

**PID aggresive video**

Implementation
--------------

The robot arm can be moved by running the following line in Terminator:

.. code-block:: python
   :linenos:

   cd Desktop/DE3Robotics/src/coursework_1/src 
   python3 kinematics.py full

This in turn eventually calls the function ``sendCommands(self,q)`` which communicates with Gazebo using a ``.publish()`` method.

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

PID values are found in lines 11, 15, and 19 of the ``controller_settings.yaml`` file shown below:

.. code-block:: python
   :linenos:

   DESE3R:
   # Publish all joint states -----------------------------------
   joint_state_controller:
      type: joint_state_controller/JointStateController
      publish_rate: 100  
   
   # Position Controllers ---------------------------------------
   joint_0_position_controller:
      type: effort_controllers/JointPositionController
      joint: joint_0
      pid: {p: 0.1, i: 0, d: 0}
   joint_1_position_controller:
      type: effort_controllers/JointPositionController
      joint: joint_1
      pid: {p: 0.1, i: 0, d: 0}
   joint_2_position_controller:
      type: effort_controllers/JointPositionController
      joint: joint_2
      pid: {p: 0.1, i: 0, d: 0}

To run the robot with updated values, first close all instances of Gazebo, re-open Gazebo and run the ``kinematics.py full`` code again.

Tuning the PID controller
-------------------------

To effectively tune the gains of the robotic arm such that it follows the trajectory, the error graph for joint angles was used in the simulator.
This can be accessed by using the ``rqt`` command in the terminal and then selecting **Plugins → Visualisaiton →** then typing **/DES3RXXXXXXXXXXXXXXX**.
Then repeating this for each joint controller.

**Screenshots for rqt visualization**

There are a number of methods that can be used, with a common and simple heuristic-based method being *Ziegler-Nichols*.
This can be done by measuring the step-response of the system and first finding ``Kp``, then using that to find ``Kd`` and ``Ki``.
Alternatively, for the purpose of this report, it can be done through trial and error, visually determining results.
It is performed similarly to the order of explanation in this report.
The steps used are summarised below.

- Set all gains to zero
- Increase ``Kp`` until the controller output oscillates at a constant rate. It should continue to be increased such that the response should increase more quickly without the system becoming unstable.
- Once ``Kp`` response is fast enough, ``Ki`` is now increased with small steps to ensure the effects are seen. The goal of increasing ``Ki`` is to gradually reduce the oscillations and reduce the steady-state errors. Once the controller is stable, a **PI** controller has been formed.
- With ``Kp`` and ``Ki`` set with minimal steady state error, ``Kd`` can be used for dampening effects. Increasing ``Kd`` will decrease overshoot of the system. The ramp rate should be monitored and the key is to prevent overshooting. Once the overshoot has been dampened, ``Kd`` is set.

.. note::
   If ``Kd`` is set too high, then the output of the **PI** controller will be against the **D** term as they are both trying to achieve opposite effects.
   Therefore, **D** should be increased cautiously.

The table below summarises the process and outputs seen.
For further insight, please view the embedded simulation video that demonstrates each output.

Due to computing limits, identical **PID** values will be processed for each joint controller initially during the test methods.
Following this, hyper-parameter tuning can then be considered to optimise the result for each joint controller.

Table of results
----------------

Video results
-------------

Final Results
-------------

.. code-block:: python
   :linenos:
   :emphasize-lines: 11,15,19

   DESE3R:
   # Publish all joint states -----------------------------------
   joint_state_controller:
      type: joint_state_controller/JointStateController
      publish_rate: 100  
   
   # Position Controllers ---------------------------------------
   joint_0_position_controller:
      type: effort_controllers/JointPositionController
      joint: joint_0
      pid: {p: 0.1, i: 0, d: 0}
   joint_1_position_controller:
      type: effort_controllers/JointPositionController
      joint: joint_1
      pid: {p: 0.1, i: 0, d: 0}
   joint_2_position_controller:
      type: effort_controllers/JointPositionController
      joint: joint_2
      pid: {p: 0.1, i: 0, d: 0}

------------------------------
Task I: Adapting the Robot Arm
------------------------------

The end effector mass is controlled in line 16 of the ``robot_model_gazebo.xacro file``, which contains all the physical model parameters. Changing the end effector mass to 30Kg yields the following results.

.. code-block::
   :linenos:
   :emphasize-lines: 16

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

   <xacro:property name="ee_mass" value="30"/>

   <xacro:property name="link_length_0" value="${link_length_0_init-case_length}"/>
   <xacro:property name="link_length_1" value="${link_length_1_init-case_radius}"/>
   <xacro:property name="link_length_2" value="${link_length_2_init-case_radius}"/>

Results
-------

Embedded video
--------------

---------------------------------
Task J: Adapting Controller Gains
---------------------------------

Using the same methods as in Task H, the PID values are updated to account for the new response of the system.

Results
-------

Embedded video
--------------



==========
References
==========

.. [1] Course notes
.. [2] Course notes, adapted
.. [3] Course notes, adapted
.. [4] Course notes, adapted
.. [5] **Derviation to be added**
.. [6] **Image source to be added**
.. [7] Course notes
.. [8] Derivation 1
.. [9] Derivation 1
.. [10] Derivation 1
.. [11] Derivation 2
.. [12] Course notes, adapted
.. [13] Course notes, adapted
.. [14] Course notes, adapted
.. [15] PID information: https://www.motioncontroltips.com/faq-what-are-pid-gains-and-feed-forward-gains/
.. [16] PID information: https://epxx.co/artigos/feedback_en.html
.. [17] PID information: https://blog.opticontrols.com/archives/344
.. [18] PID information: https://pidexplained.com/how-to-tune-a-pid-controller/
.. [19] PID information: https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
.. [20] PID information: https://www.electronicshub.org/pid-controller-working-and-tuning-methods/#Trial_and_Error_Method