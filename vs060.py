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
        

class vs060Robot(object):
    """CobottaRobot"""
    def __init__(self):
        super(vs060Robot, self).__init__()

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander(ns="vs060")
        # robot = moveit_commander.RobotCommander(ns="vs060")

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the Cobotta
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

        ################ MOVE POSITIONS #####################
        self.home_joint_values = [0, 0, pi/2, 0, pi/2, 0]
        
        self.pick_pose = geometry_msgs.msg.Pose()
        self.pick_pose.orientation.y = 1.0 # (0, 1, 0, 0)
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


    def go_to_joint_state(self, target_joint_values, execute = True):
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

    def go_to_pose_goal(self, pose_goal, execute = True):

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


    def plan_cartesian_path(self, waypoints): #scale=1):

        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through:

        (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold

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


def place_boxes(vs060):
    single_bin = ds.SingleBinProblem(ds.Bin(10, 10, 10))
    boxList = [ds.Box(1, 2, 3) for i in range(103)]
    single_bin.boxList = boxList
    res, boxList = single_bin.fillBin()
    i = 0
    for box in boxList:
        rospy.sleep(0.01)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = vs060.planning_frame
        box_pose.pose.position.x = float(box.position.x) / 10 + 2
        box_pose.pose.position.y = float(box.position.y) / 10 + 2
        box_pose.pose.position.z = float(box.position.z) / 10 + 0
        vs060.scene.add_box('castelnovo' + str(i), box_pose,
                            size=(float(box.width) / 10, float(box.height) / 10, float(box.depth) / 10))
        i += 1
        print 'scatola piazzata' + str(i)


if __name__ == '__main__':

    try:    
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('vs060_node', anonymous=True)
        vs060 = vs060Robot()
        
        # ROBOT GOES HOME      
        execution = True
        plan = vs060.go_to_joint_state(vs060.home_joint_values, execution)
#        cr.display_trajectory(plan)
        
        rate = rospy.Rate(10) # 10hz

        place_boxes(vs060)


#       PICK POSITION
        print "Moving Robot to pick position ..."
        plan = vs060.go_to_pose_goal(vs060.approach_pick_pose, execution)
#            cr.display_trajectory(plan)
#            rospy.sleep(1)
        plan = vs060.go_to_pose_goal(vs060.pick_pose, execution)
        rospy.sleep(1)
        plan = vs060.go_to_pose_goal(vs060.approach_pick_pose, execution)
#            rospy.sleep(1)
        print "Moving Robot to place position ..."
        plan = vs060.go_to_pose_goal(vs060.approach_place_pose, execution)
#            cr.display_trajectory(plan)
#            rospy.sleep(1)
        plan = vs060.go_to_pose_goal(vs060.place_pose, execution)
#            cr.display_trajectory(plan)
        rospy.sleep(1)
        plan = vs060.go_to_pose_goal(vs060.approach_place_pose, execution)
#            cr.display_trajectory(plan)
#            rospy.sleep(1)
        print "Task done, going back home ..."
            # ROBOT GOES HOME
        plan = vs060.go_to_joint_state(vs060.home_joint_values, execution)
#            cr.display_trajectory(plan)
        rate.sleep()

    except rospy.ROSInterruptException:
        pass
