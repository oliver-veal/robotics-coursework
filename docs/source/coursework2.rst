.. DE3 Robotics Coursework documentation master file, created by
   sphinx-quickstart on Wed Jan 27 19:02:39 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

==========================================================
Welcome to DE3 Robotics Group 1 Coursework 2 Documentation
==========================================================

We (Shafae Ali, Oscar Jones, Oscar Leclercq, Oliver Veal) have decided to create a readthedocs page for our coursework submissions instead of a Google Drive or Word document. This way this documentation can be used for teaching ROS at any point in the future, as well as have integrated code blocks and interactive videos.
For the purpose of archiving the document, we will be submitting in PDF and HTML formats.

.. warning::

    For a more interactive experience, please use the web_ version instead of the PDF. Thanks!

.. _web: https://robotics-coursework-de3.readthedocs.io/en/latest/#

=====
Setup
=====

Open Terminator to input commands and run different services or files.

==================
C-Space Map
==================

----------------------------------------
Task Ai : C-Space Dilation, Square Mask
----------------------------------------

This task initialises the Denavit-Hartenberg, D-H, table.
The table contains all the necessary information to orientate each link of the robot in a consistent manner so that the position of each link can be found relative to the other.
As the robot moves, the D-H table is updated.
The D-H table is a convenient way to store this information as the transformation matrix for each link can be evaluated using the corresponding row in the table.

-------------------------------------------
Task Aii : C-Space Dilation, Circular Mask
-------------------------------------------

To make the D-H table, the reference

====================
Waypoint Navigation
====================

----------------------------------------------------
Task Bi: Adding Waypoints by Hand, Creating the Path
----------------------------------------------------

This task initialises the Denavit-Hartenberg, D-H, table.
The table contains all the necessary information to orientate each link of the robot in a consistent manner so that the position of each link can be found relative to the other.
As the robot moves, the D-H table is updated.
The D-H table is a convenient way to store this information as the transformation matrix for each link can be evaluated using the corresponding row in the table.


-------------------------------------------------------------
Task Bii: Adding Waypoints by Hand, Shortest Path Calculation
-------------------------------------------------------------

This task initialises the Denavit-Hartenberg, D-H, table.

==========================
Potential Field Algorithm
==========================

---------------------------------------------------------------------------
Task Ci: Implementing the Potential Field Algorithm, General Implementation
---------------------------------------------------------------------------

This task initialises the Denavit-Hartenberg, D-H, table.
The table contains all the necessary information to orientate each link of the robot in a consistent manner so that the position of each link can be found relative to the other.
As the robot moves, the D-H table is updated.
The D-H table is a convenient way to store this information as the transformation matrix for each link can be evaluated using the corresponding row in the table.

---------------------------------------------------------------------------
Task Cii: Implementing the Potential Field Algorithm, Custom Implementation
---------------------------------------------------------------------------

..
   Part ii
   As mentioned, sometimes we require more complex potential fields to achieve better performance from
   our motion planner. Design and implement your own potential functions (different from equations (1-3).
   Discuss your reasoning behind your proposed potential function, and how you implemented it in your report.
   On an image of the map, draw the path that DE NIRO takes, and include this in your report. Is
   this the optimal path? Comment on any interesting features of the path taken.
   Estimate the length of the path taken by DE NIRO - is it better or worse than the length of the
   path taken when following your own waypoints? If it is better, why do you think it is better? If it is worse,
   why do you think it is worse?

**A Better Potential Fields Algorithm**

Observations from using a linear falloff function:
 - In order to gain sufficient repulsion from nearby objects, a large Kn value is needed, which overwhelms any effect of the positive force. It is extremely difficult or impossible to balance the two forces.
 - The linear falloff means that distant objects that should have a small contribution to the negative force experienced in a given location actually contribute a much larger force than expected. This can result in pushed away from the centre of mass (the centre of the map in this roughly symmetrical case) rather than pushed away from local obstacles only.
 - Using a steeper falloff (e.g. quadratic or inverse square, like real gravitaional falloff) with a smaller overall coefficient means that strong forces are experienced close to objects, and in open space the predominant force is the positive attraction to the goal. This means the robots gets "stuck" in a gravitaion well more rarely and can make more confident progress. It also makes the robot much less likely to just "plough" straight into obstacles due to the unbalances positive/negative forces, and shows stable behaviour (i.e. finds a valid path) for a much greater combination of parameters.

If we could hypothesise a more ideal algorithm for this case (and hopefully considering more general cases), we would like it to have the following features:
 - Contributions from very distant objects are negligable or zero
 - We should never crash into obstacles as a priority, even if there is thereforce no valid path (it doesn't matter if the robot follows a path to the goal if it crashes on the way there)
 - The positive force should be able to be "felt" on all parts of the map in roughly the same magnitude 

As such, we propose the following modifications to the Potential Fields Algorithm*:

*While maintaining a linear force falloff

 - Normalise the "mass" of each object by only considering the edges of the obstacle to provide a repulsive force. (In effect: Run an edge detection algorithm over the expanded map to keep only the edges of obstacles. Or, equivalently, when creating the C-space map; dilate the obstacles by 1px less than required, then dilate using the full width mask, then take the XOR of the two maps. XOR will only keep 1 values where they exist on the fully expanded map, but not the less expanded map, i.e. the edges. Detailed below).
 - Take an average of the 40 largest force contributions by magnitude
 - Take the positive and negative scalar parameters to be equal

The idea behind taking the average of the top n largest force contributions comes from this (https://medium.com/@rymshasiddiqui/path-planning-using-potential-field-algorithm-a30ad12bdb08) article, in which the author describes their method as using a max function on the negative forces from each obstacle, essentially considering only the repulsive force from the most repulsive object. We improve on this idea by not just taking the largest value, but an average of the n largest values, where n is a tunable parameter. The idea behind taking the average can be seen when looking at a very narrow section. When only taking the max, the effect is to produce a force near the the central line between obstacles that "oscillates" very rapidly between pointing in one direction or the other, primarly because it can only consider repulsion from one object at a time. In this simple case, averaging the force from the objects only both sides product a much smoother path for the robot to follow, the the effects of being repelled by two close by but opposing forces "cancels out". The effects of changing this average parameter are discussed below:

**Effects of Tuning the Top N Average Parameter**

TODO


**Downsides and Considerations**

Some consideration would need to be given to some edge cases with this algorithm, although they haven't neccessarily been explored here as this particular obstacle map is geometrically simple (rectangular, convex shapes for obstacles etc).

Consider for example consider the case of a single point obstacle close to a "wall" (line) of obstacle pixels. It is possible in this case to encounter the same problem as when just taking a global average of all pixel forces, that is, the repulsive force from the single pixel does not contribute enough to the average to avoid the robot from crashing into it.

Another case would be that of non-convex obstacles (for example a "bowl" or "banana" type shape. ------- TODO Expand on this

==========================
Probabilistic Road Map
==========================

---------------------------------------------------
Task Di: Randomly Sampling from the Map
---------------------------------------------------

This task initialises the Denavit-Hartenberg, D-H, table.
The table contains all the necessary information to orientate each link of the robot in a consistent manner so that the position of each link can be found relative to the other.
As the robot moves, the D-H table is updated.
The D-H table is a convenient way to store this information as the transformation matrix for each link can be evaluated using the corresponding row in the table.

---------------------------------
Task Dii: Harris Corner Detection
---------------------------------

This task initialises the Denavit-Hartenberg, D-H, table.

-----------------------------------------------------------------
Task Ei: Creating the Graph, Tuning Distances for Creating Edges
-----------------------------------------------------------------

This task initialises the Denavit-Hartenberg, D-H, table.

---------------------------------------------------------
Task Eii: Creating the Graph, Tuning Edge Collision Check
---------------------------------------------------------

This task initialises the Denavit-Hartenberg, D-H, table.

-------------------------------------------------------------
Task Eiii: Creating the Graph, Completing an Incomplete Graph
-------------------------------------------------------------

This task initialises the Denavit-Hartenberg, D-H, table.

------------------------------------------------
Task Fi: Dijkstra's Algorithm, Creating the Path
------------------------------------------------

This task initialises the Denavit-Hartenberg, D-H, table.
The table contains all the necessary information to orientate each link of the robot in a consistent manner so that the position of each link can be found relative to the other.
As the robot moves, the D-H table is updated.
The D-H table is a convenient way to store this information as the transformation matrix for each link can be evaluated using the corresponding row in the table.

----------------------------------------------------
Task Fii: Dijkstra's Algorithm, Planning Algorithms
----------------------------------------------------

This task initialises the Denavit-Hartenberg, D-H, table.
The table contains all the necessary information to orientate each link of the robot in a consistent manner so that the position of each link can be found relative to the other.
As the robot moves, the D-H table is updated.
The D-H table is a convenient way to store this information as the transformation matrix for each link can be evaluated using the corresponding row in the table.