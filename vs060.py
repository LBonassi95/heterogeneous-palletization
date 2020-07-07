#!/usr/bin/env python


import sys
from random import random

import rospy
from std_msgs.msg import Int32
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import Data_Structures as ds
import XmlParser as xmlparser
import json

# Per lanciare
    # roslaunch denso_robot_bringup vs060_bringup.launch

class vs060Robot(object):
    def __init__(self):
        super(vs060Robot, self).__init__()

        robot = moveit_commander.RobotCommander(ns="vs060")
        # robot = moveit_commander.RobotCommander(ns="vs060")

        # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        # to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to one group of joints.  In this case the group is the joints in the Cobotta
        ## arm so we set ``group_name = arm``.
        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        group.set_max_velocity_scaling_factor(1)
        group.set_max_acceleration_scaling_factor(1)

        ## We create a `DisplayTrajectory`_ publisher which is used later to publish
        ## trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame
        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link
        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()
        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        self.origin = robot.get_current_state()
        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        ################ MOVE POSITIONS #####################
        self.home_joint_values = [0, 0, pi / 2, 0, pi / 2, 0]
        #self.home_joint_values = self.origin.joint_state
        #self.home_joint_values = [0.00015346623907763757, -1.407011836317686, -0.9765591968036391, 3.141215559515132, 0.7577111499402598, -3.1405790212738296]
        # [0,0,0,0,quello prima dell'end effector,0]

        self.pick_pose = geometry_msgs.msg.Pose()
        self.pick_pose.orientation.y = 1.0  # (0, 1, 0, 0)
        self.pick_pose.position.x = 0.5
        self.pick_pose.position.y = 0
        self.pick_pose.position.z = 0.3

        self.approach_pick_pose = geometry_msgs.msg.Pose()
        self.approach_pick_pose.orientation.y = 1.0
        self.approach_pick_pose.position.x = 0.5
        self.approach_pick_pose.position.y = 0
        self.approach_pick_pose.position.z = 0.4

        self.place_pose = geometry_msgs.msg.Pose()
        self.place_pose.orientation.y = 1.0
        self.place_pose.position.x = -0.4
        self.place_pose.position.y = 0.3
        self.place_pose.position.z = 0.5

        self.approach_place_pose = geometry_msgs.msg.Pose()
        self.approach_place_pose.orientation.y = 1.0
        self.approach_place_pose.position.x = -0.4
        self.approach_place_pose.position.y = 0.3
        self.approach_place_pose.position.z = 0.6

        self.origin_point = geometry_msgs.msg.Pose()
        self.origin_point.orientation.y = 1.0  # (0, 1, 0, 0)
        self.origin_point.position.x = -0.4
        self.origin_point.position.y = -0.3
        self.origin_point.position.z = 0

        self.mid_point = geometry_msgs.msg.Pose()
        self.mid_point.orientation.y = 1.0  # (0, 1, 0, 0)
        self.mid_point.position.x = -0.1
        self.mid_point.position.y = -0.2
        self.mid_point.position.z = 0.4

        self.box_counter = 0
        ################ MOVE POSITIONS #####################

    def attach_box(self, box_name, timeout=4):
        box_name = box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = 'arm'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, box_name, timeout=4):
        box_name = box_name
        scene = self.scene
        eef_link = self.eef_link
        scene.remove_attached_object(eef_link, name=box_name)
        #scene.remove_world_object(box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def go_to_joint_state(self, target_joint_values, execute=True):
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        if execute:
            plan = self.group.go(target_joint_values, wait=True)
            # Calling ``stop()`` ensures that there is no residual movement
            self.group.stop()
        else:
            self.group.set_joint_value_target(target_joint_values)
            plan = self.group.plan()

        #current_joints = self.group.get_current_joint_values()
        # print current_joints
        return plan

    def go_to_pose_goal(self, pose_goal, execute=True):

        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        self.group.set_pose_target(pose_goal)

        if execute:
            plan = self.group.go(wait=True)
            # Calling `stop()` ensures that there is no residual movement
            self.group.stop()
        else:
            plan = self.group.plan()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()

        #current_pose = self.group.get_current_pose().pose
        # print current_joints

        return plan

    def plan_cartesian_path(self, waypoints):  # scale=1):

        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through:

        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):

        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):

        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        self.group.execute(plan, wait=True)
        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


def place_boxes_planning(vs060, boxList, offset_x):
    # i = 0
    # for box in boxList:
    #     vs060.detach_box(str(i))
    #     i += 1
    scale_factor = 1
    eps = 0.005
    #offset_x = 0

    #boxList = sorted(boxList, key=lambda box: box.position.get_y(), reverse=False)  # check if it works properly
    #boxList = sorted(boxList, key=lambda box: box.get_end_y(), reverse=False)  # check if it works properly


    for box in boxList:


        vs060.approach_place_pose.position.x = box.position.get_x() + box.get_width() / 2 + vs060.origin_point.position.x
        vs060.approach_place_pose.position.y = box.position.get_z() + box.get_depth() / 2 + vs060.origin_point.position.y
        vs060.approach_place_pose.position.z = box.position.get_y() + box.get_height() + 0.1

        plan = vs060.go_to_pose_goal(vs060.approach_place_pose, execute=True)
        if plan:
            print('[OK] approach pose')

        vs060.place_pose.position.x = box.position.get_x() + box.get_width() / 2 + vs060.origin_point.position.x
        vs060.place_pose.position.y = box.position.get_z() + box.get_depth() / 2 + vs060.origin_point.position.y
        vs060.place_pose.position.z = box.position.get_y() + box.get_height()
        plan = vs060.go_to_pose_goal(vs060.place_pose, execute=True)
        if plan:
            print('[ok] pose ')
            #vs060.detach_box('scatolina' + str(index))
            #vs060.wait_for_state_update(box_is_known=True, timeout=5)


        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = vs060.planning_frame
        box_pose.pose.position.x = ((float(box.position.x) + (float(box.width) / 2)) / scale_factor) + vs060.origin_point.position.x
        box_pose.pose.position.y = ((float(box.position.z) + (float(box.depth) / 2)) / scale_factor) + vs060.origin_point.position.y
        box_pose.pose.position.z = ((float(box.position.y) + (float(box.height) / 2)) / scale_factor)

        vs060.scene.add_box(str(vs060.box_counter), box_pose,
                            size=((float(box.width) - eps) / scale_factor, (float(box.depth) - eps) / scale_factor,
                                  (float(box.height) - eps) / scale_factor))
        vs060.wait_for_state_update(box_is_known=True, timeout=5)

        plan = vs060.go_to_pose_goal(vs060.approach_place_pose, execute=True)
        if plan:
            print('[OK] approach pose')

        vs060.box_counter += 1
        print 'scatola piazzata' + str(vs060.box_counter)


def place_boxes(vs060, boxList, offset_x):
    # i = 0
    # for box in boxList:
    #     vs060.detach_box(str(i))
    #     i += 1
    scale_factor = 1
    eps = 0.0005
    # offset_x = 0

    for box in boxList:
        rospy.sleep(0.01)

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = vs060.planning_frame
        box_pose.pose.position.x = ((float(box.position.x) + (float(box.width) / 2)) / scale_factor) + vs060.origin_point.position.x + offset_x
        box_pose.pose.position.z = ((float(box.position.y) + (float(box.height) / 2)) / scale_factor)
        box_pose.pose.position.y = ((float(box.position.z) + (float(box.depth) / 2)) / scale_factor) + vs060.origin_point.position.y
        vs060.scene.add_box(str(random()) + str(vs060.box_counter), box_pose,
                            size=((float(box.width) - eps) / scale_factor, (float(box.depth) - eps) / scale_factor,
                                  (float(box.height) - eps) / scale_factor))
        vs060.box_counter += 1
        print 'scatola piazzata' + str(vs060.box_counter)


def multipleBinsManagement():
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('vs060_node', anonymous=True)
        vs060 = vs060Robot()

        # ROBOT GOES HOME
        plan = vs060.go_to_joint_state(vs060.pick_pose, execute=True)
        print('Sono andato in pick-pose')
        plan = vs060.go_to_joint_state(vs060.mid_point, execute=True)
        print('Sono andato in midpoint-pose')
        plan = vs060.go_to_joint_state(vs060.origin_point, execute=True)
        print('Sono andato in origin-pose')
        plan = vs060.go_to_joint_state(vs060.mid_point, execute=True)
        print('Sono tornato in midpoint-pose')
        print('Ora parte il pick and place....')

        # [START] placing grafico delle scatole per vedere come dovranno venire
        scale_factor = 25
        eps = 0.005
        offset_x = 2

        with open('./result.json') as json_file:
            data = json.load(json_file)
            for sol_bin in data:
                for box in data[sol_bin]['box_list']:
                    rospy.sleep(0.01)

                    # General structure of a box in json file
                    # [
                    #     "Item4",  --->type of box  index=0
                    #     6.0,  ---> width  index=1
                    #     2.0,  ---> height   index=2
                    #     3.0,  ---> depth  index=3
                    #     0,  ---> place position x  index=4
                    #     0,  ---> place position z  index=5
                    #     0  ---> place position y  index=6
                    # ]

                    box_pose = geometry_msgs.msg.PoseStamped()
                    box_pose.header.frame_id = vs060.planning_frame
                    box_pose.pose.position.x = ((float(box[4]) + (float(box[1]) / 2)) / scale_factor) + vs060.origin_point.position.x + offset_x
                    box_pose.pose.position.z = ((float(box[5]) + (float(box[2]) / 2)) / scale_factor)
                    box_pose.pose.position.y = ((float(box[6]) + (float(box[3]) / 2)) / scale_factor) + vs060.origin_point.position.y

                    vs060.scene.add_box(str(random()) + str(vs060.box_counter), box_pose,
                                        size=((float(box[1]) - eps) / scale_factor,
                                              (float(box[3]) - eps) / scale_factor,
                                              (float(box[2]) - eps) / scale_factor))
                    vs060.box_counter += 1
                    print 'scatola piazzata' + str(vs060.box_counter)

                offset_x += 1

        # [END] placing grafico delle scatole per vedere come dovranno venire


        # [START] placing effettivo delle scatole
        eps = 0.005
        scale_factor = 50

        with open('./result.json') as json_file:
            data = json.load(json_file)

            for sol_bin in data:
                # General structure of a box in json file
                # [
                #     "Item4",  --->type of box  index=0
                #     6.0,  ---> width  index=1
                #     2.0,  ---> height   index=2
                #     3.0,  ---> depth  index=3
                #     0,  ---> place position x  index=4
                #     0,  ---> place position z  index=5
                #     0  ---> place position y  index=6
                # ]
                index = 0
                boxList = []
                for box in data[sol_bin]['box_list']:
                    tmp_box = ds.Box(box[1]/scale_factor, box[2]/scale_factor, box[3]/scale_factor)
                    tmp_box.set_pos(box[4]/scale_factor, box[5]/scale_factor, box[6]/scale_factor)

                    boxList.append(tmp_box)

                index = 0
                boxList = sorted(boxList, key=lambda box: box.position.get_y(), reverse=False)
                boxList = sorted(boxList, key=lambda box: box.get_end_y(), reverse=False)

                for box in boxList:

                    box_pose = geometry_msgs.msg.PoseStamped()
                    box_pose.header.frame_id = vs060.planning_frame
                    box_pose.pose.position.x = 0.5
                    box_pose.pose.position.y = 0
                    box_pose.pose.position.z = box.get_height() / 2
                    vs060.scene.add_box('scatolina' + str(index), box_pose,
                                        size=((box.get_width() - eps), (box.get_depth() - eps),
                                              (box.get_height() - eps)))

                    vs060.wait_for_state_update(box_is_known=True, timeout=5)

                    vs060.approach_pick_pose.position.z = box.get_height() + 0.1   # modifica in qualcosa percentuale
                    plan = vs060.go_to_pose_goal(vs060.approach_pick_pose, execute=True)
                    if plan:
                        print('[OK] approach pre-pick')

                    vs060.pick_pose.position.z = box.get_height()
                    plan = vs060.go_to_pose_goal(vs060.pick_pose, execute=True)
                    if plan:
                        print('[OK] pick')

                    vs060.attach_box('scatolina' + str(index))

                    plan = vs060.go_to_pose_goal(vs060.approach_pick_pose, execute=True)
                    if plan:
                        print('[OK] approach after-pick ')

                    plan = vs060.go_to_pose_goal(vs060.mid_point, execute=True)
                    if plan:
                        print('[OK] mid point')

                    vs060.approach_place_pose.position.x = box.position.get_x() + box.get_width() / 2 + vs060.origin_point.position.x
                    vs060.approach_place_pose.position.y = box.position.get_z() + box.get_depth() / 2 + vs060.origin_point.position.y
                    vs060.approach_place_pose.position.z = box.position.get_y() + box.get_height() + 0.1

                    plan = vs060.go_to_pose_goal(vs060.approach_place_pose, execute=True)
                    if plan:
                        print('[OK] approach pose')

                    vs060.place_pose.position.x = box.position.get_x() + box.get_width() / 2 + vs060.origin_point.position.x
                    vs060.place_pose.position.y = box.position.get_z() + box.get_depth() / 2 + vs060.origin_point.position.y
                    vs060.place_pose.position.z = box.position.get_y() + box.get_height()
                    plan = vs060.go_to_pose_goal(vs060.place_pose, execute=True)
                    if plan:
                        print('[ok] pose ')
                        vs060.detach_box('scatolina' + str(index))
                        vs060.wait_for_state_update(box_is_known=True, timeout=5)

                    else:
                        # torno alla posizione iniziale
                        # metto giu il pezzo e lo prendo da un altro lato
                        # riprovo a fare il panning
                        print('aia')
                        vs060.detach_box('scatolina' + str(index))
                        vs060.wait_for_state_update(box_is_known=True, timeout=5)
                        vs060.scene.remove_world_object('scatolina' + str(index))
                        vs060.wait_for_state_update(box_is_known=True, timeout=5)

                    plan = vs060.go_to_pose_goal(vs060.approach_place_pose, execute=True)
                    if plan:
                        print('[OK] approach after-pose')

                    plan = vs060.go_to_pose_goal(vs060.mid_point, execute=True)
                    if plan:
                        print('[OK] mid point after pose')

                    print('==================================================\n')
                    index += 1


                # Attendo 5 secondi e rimuovo tutte le scatole appena piazzate e passo al prossimo bin
                rospy.sleep(5)
                print('Ho terminato il piazzamento di un bin!!!\n')
                print('ora cancello tutto e passo al prossimo bin se c\'e')

                for i in range(0, index):
                    vs060.scene.remove_world_object('scatolina' + str(i))
                    vs060.wait_for_state_update(box_is_known=True, timeout=2)

                index = 0
                print('=================================================================')

        # [END] placing effettivo delle scatole

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':

    multipleBinsManagement()

