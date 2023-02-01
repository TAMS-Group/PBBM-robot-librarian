## Overview

The main ways to use this package is by calling the scripts in the *scripts*
folder. The main pipeline can be launched with the `grasping_controller.py`
file, while the rest are helper functions. In the *src* folder, two classes are
defined for controlling the gripper and shadowhand respectively. These are
mainly used in the pipeline. In the *launch* folder, the required launch files
to launch the main pipeline and some helper files are located, which are
described in more detail below. In the *config* file, the configs for the
Apriltag detection are located, but only the `tags.yaml` file should be
adjusted, if other Apriltags are used.

---

## Hardware setup

Because of the limited workspace of the manipulators of the PR2, both books
and the shelf need to be placed at certain spots for the pipeline to
execute. The shelf needs to be placed using the vision pipeline. When the
vision pipeline is launched, and the Apriltag of the shelf can be seen
by the robot, a tf frame with the name *shelf_tag* will be published. The
shelf needs to be moved manually, until pose of the tf frame in rviz
roughly corresponds to the following pose:

**position:**  
*x = 0.6192*  
*y = 0.3697*  
*z = 1.152*

**orientation:**  
*x = 0.5*  
*y = -0.5*  
*z = -0.5*  
*w = 0.5*

The books should then be placed in the upper shelf, preferably with the
largest to smallest from left to right. They should be placed in the
same general area as can be seen in the following image:

![Book setup in the shelf.](img/shelf_setup.jpg)

Afterwards, the setup should look similar to this image:

![Full setup with robot and shelf.](img/full_setup.jpg)

---


## Required repositories

Besides the three librarian repositories, `librarian_vision`,
`librarian_common` and `librarian_manipulation`, two additional repositories
are needed. The first one is required to be able to enable or disable the
collision detection between different objects and is a modified MoveIt
version, which can be found here:

[Modified MoveIt](https://github.com/bsygo/moveit/tree/pr-moveit-added-ACM-python-methods)

The second one is a modified version of the bio\_ik\_service, which can be
found here:

[Modified bio\_ik\_service](https://github.com/bsygo/bio_ik_service/tree/pr-bio-ik-service-request-planning-scene-state)

In the future, one or both of these modified repositories might get included
in the original repositories, which would make then not needed anymore.

---

## Launching the pipeline

Before launching the pipeline, it is advised to set up the robots arms using
the command:

`rosrun librarian_grasping_initial reset_arms.py`

This sets the arms to the side of the robot and improves the likelihood
for the planners ot find correct initial paths. It can also be used without
the rest of the pipeline ot just set the arms into save positions.
The easiest way to launch the pipeline is to call the launch file:

`roslaunch librarian_grasping_initial grasping.launch`

This starts the grasping pipeline, which awaits a *ManipulationService* message
on the *manipulation\_service* topic. However, this launches the pipeline
assuming to be run on the real robot. If the pipeline is ecpected to run in 
the MoveIt demo mode, please instead use the command:

`roslaunch librarian_grasping_initial grasping.launch demo:=True`

---

## Running manipulation individually

***DISCLAIMER: While the grasping pipeline can be run individually, that does not
mean the other packages are not required. At least librarian_resources needs
to exist, since some required resources are located there, even though nothing
from that package needs to be run manually.***

If the manipulation is to be run with the rest of the librarain project, the
launching section already covers everything necessary for it to run. However,
if for some reason the manipulation should be run individually, it can be done
by first launching the MoveIt scene:

`roslaunch tams_pr2_moevit_config demo.launch`

Next, the shelf and books need to be spawned. To spawn the shelf statically,
the required tf frames need to be published first by calling:

`roslaunch librarian_grasping_initial tf_shelf_broadcaster.launch`

If it is desired to spawn the shelf from an Apriltag detection, it can be
spawned using:

`roslaunch librarian_grasping_initial continuous_detection.launch`

The shelf, as well as the books, can then be spawned using the
*scene\_setup.py* script, either statically by calling:

`rosrun libraraian_grasping_initial scene_setup.py`

or using another Apriltag to spawn the books, by using:

`rosrun librarian_grasping_initial scene_setup.py -test`

The later one can also be called without having published the tf frames, which
results in only the books being spawned and not the shelf. For more details on
how to spawn the books using an Apriltag, refer to the documentation of the
`test_spawn_book` function in *scene\_setup.py*. If any other combination of
spawning the shelf and books is required, or other books, an other shelf pose 
or other book poses are desired, you will have to write another script using
the `create_shelf`, `create_book` and `test_spawn_book` function in 
`scene_setup.py`.

Afterwards, launch the pipeline as described in the launch section. Finally, a
manipulation message to grasp the books can be send by using:

`rosrun librarian_graspin_initial interaction_dummy_node.py`

Which books to grasp and where to place them needs to be changed in this node
manually or similar node need to be written.

---

## Step by step workflow

This list of steps should not be necessary to understand, if the pipeline is
only to be launched. However, if any bugs or errors occur, it might be a good
reference to better pinpoint the error. In the code, the corresponding
parts to each step are marked with comments.

1. Initiate controller and wait for message \(only once when launchfile is run\).
2. Instanciate an interface for both the gripper and the shadowhand \(when message was received\).
3. Pick the book.
   1. Move torso to the height corresponding to the shelf in which the book is.
   2. Tilt the book with the shadowhand.
      1. Move the fingers of the shadow hand in the desired configuration.
      2. Move the shadow hand in front of the book.
      3. Move the first finger above the book.
      4. Move the finger down to make contact with the book.
      5. Move the finger further down to apply pressure to the book.
      6. Move the shadow hand back in a straight line to tilt the book.
   3. Grasp the book with the gripper.
      1. Move the gripper before the book.
      2. Move the gripper in a straight line towards the book.
      3. Close the gripper to grasp the book.
   4. Extract the book.
      1. Move the shadowhand upwards to loose contact with the book.
      2. Move the shadowhand away from the book.
      3. Move back the gripper in a straight line a bit to pull out the book further.
      4. Open the gripper to let the book fall back straight with an exposed spine.
      5. Move the gripper straight forward again.
      6. Close the gripper.
      7. Move the gripper far back in a straight line to pull out the book completely.
4. Move the book before the camera and rotate it. Currently not usefull but in theory to identify book further.
5. Place the book.
   1. Move torso to the height corresponding to the shelf in which the book should be placed.
   2. Lean the book against the shelf or previous book.
      1. Move the gripper into position before the shelf.
      2. Move the gripper forward to have the book hovering in the shelf.
      3. Move the gripper to the left until a force threshold is met.
      4. Move the gripper downward until a force threshold is met.
      5. Open the gripper.
      6. Move the gripper straight backward to have the book leaning by itself.
      7. Close the gripper.
   3. Push the book upright.
      1. Move the gripper rotated in front of the shelf next to the book.
      2. Move the gripper forward in a straight line to be next to the book.
      3. Move the gripper to the left until a force threshold is met to push the book upright.
      4. Move the gripper to the right to remove it from the book.
      5. Move the gripper outside the shelf.
      6. Open the gripper.
6. Wait until next message is received and repeat from step 2.

