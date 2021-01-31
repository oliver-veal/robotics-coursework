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

Login for VM machine: deniro123
Open Terminator to input commands, shortcuts for opening new instances:
‘Ctrl+Shift+o’ - split horizontally
‘Ctrl+Shift+e’ - split vertically

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

----------------------------
Task B: Coding the D-H Table
----------------------------

text
.. code-block:: python

------------------------------------
Task C: Computing Forward Kinematics
------------------------------------

text

==================
Inverse Kinematics
==================

-----------------------------------------------
Task D: Checking if a point is in the workspace
-----------------------------------------------

text

--------------------------------------
Task E: Calculating Inverse Kinematics
--------------------------------------

text

---------------------------------
Task F: Coding Inverse Kinematics
---------------------------------

text

=======================
Differential Kinematics
=======================

-------------------------------
Task G: Computing the Jacobian
-------------------------------

=============
Robot Control
=============

-----------------------------------------------
Task H: Tuning Controller Gains
-----------------------------------------------

text

------------------------------
Task I: Adapting the Robot Arm
------------------------------

text

---------------------------------
Task J: Adapting Controller Gains
---------------------------------

text

test image

.. image:: img/diagram.png
   :height: 200
   :width: 200
   :alt: Robot Diagram

   