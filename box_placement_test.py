#!/usr/bin/env python

## Cobotta robot demo for UNIBS class

import sys
import rospy
from std_msgs.msg import Int32
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import Data_Structures as ds
import test_palletizationModel


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

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.box_counter = 0

        ################ MOVE POSITIONS #####################
        self.home_joint_values = [0, 0, pi / 2, 0, pi / 2, 0]
        # [0,0,0,0,quello prima dell'end effector,0]
        self.object_spawning_config = [0, 0, -pi / 2, 0, 0, 0]

        self.pick_pose = geometry_msgs.msg.Pose()
        self.pick_pose.orientation.y = 1.0  # (0, 1, 0, 0)
        self.pick_pose.position.x = 0.2
        self.pick_pose.position.y = -0.13
        self.pick_pose.position.z = 0.05

        self.approach_pick_pose = geometry_msgs.msg.Pose()
        self.approach_pick_pose.orientation.y = 1.0
        self.approach_pick_pose.position.x = 0.2
        self.approach_pick_pose.position.y = -0.13
        self.approach_pick_pose.position.z = 0.15

        self.place_pose = geometry_msgs.msg.Pose()
        self.place_pose.orientation.y = 1.0
        self.place_pose.position.x = 0.26
        self.place_pose.position.y = 0.1
        self.place_pose.position.z = 0.1

        self.approach_place_pose = geometry_msgs.msg.Pose()
        self.approach_place_pose.orientation.y = 1.0
        self.approach_place_pose.position.x = 0.26
        self.approach_place_pose.position.y = 0.1
        self.approach_place_pose.position.z = 0.2

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

        current_joints = self.group.get_current_joint_values()
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

        current_pose = self.group.get_current_pose().pose
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
        self.display_trajectory_publisher.publish(display_trajectory);

    def execute_plan(self, plan):

        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        self.group.execute(plan, wait=True)
        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


def place_boxes(vs060, boxList, offset_x):
    # i = 0
    # for box in boxList:
    #     vs060.detach_box(str(i))
    #     i += 1
    scale_factor = 20
    eps = 0.1

    for box in boxList:
        rospy.sleep(0.01)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = vs060.planning_frame
        box_pose.pose.position.x = ((float(box.position.x) + (float(box.width)/2)) / scale_factor) + offset_x
        box_pose.pose.position.z = ((float(box.position.y) + (float(box.height)/2)) / scale_factor)
        box_pose.pose.position.y = ((float(box.position.z) + (float(box.depth)/2)) / scale_factor)
        vs060.scene.add_box(str(vs060.box_counter), box_pose,
                            size=((float(box.width)-eps) / scale_factor, (float(box.depth)-eps) / scale_factor, (float(box.height)-eps) / scale_factor))
        vs060.box_counter += 1
        print 'scatola piazzata' + str(vs060.box_counter)


if __name__ == '__main__':

    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('vs060_node', anonymous=True)
        vs060 = vs060Robot()

        #CONFIGURAZIONE CON BUG
        bin = ds.Bin(10, 50, 10)
        box_list = test_palletizationModel.get_random_box_list_with_weight(10)
        sb = ds.SingleBinProblem(bin)
        sb.boxList = box_list
        res = sb.branch_and_bound_filling_iter()
        place_boxes(vs060, sb.placement_best_solution2, 1)

        # bin = ds.Bin(30, 50, 20)
        # box_list1 = [ds.Box(2, 3, 4) for i in range(15)]
        # for box in box_list1:
        #     box.itemName = "item1"
        # box_list2 = [ds.Box(4, 5, 2) for i in range(15)]
        # for box in box_list2:
        #     box.itemName = "item2"
        # box_list3 = [ds.Box(6, 2, 3) for i in range(15)]
        # for box in box_list3:
        #     box.itemName = "item3"
        # box_list = box_list1 + box_list2 + box_list3
        # sb = ds.SingleBinProblem(bin)
        # sb.boxList = box_list
        # res = sb.branch_and_bound_filling_iter()
        # place_boxes(vs060, sb.placement_best_solution2, 0.1)


    except rospy.ROSInterruptException:
        pass
