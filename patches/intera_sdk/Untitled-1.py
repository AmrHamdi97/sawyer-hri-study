
#!/usr/bin/env python

import rospy
from intera_interface import Limb
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions,
    InteractionOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
import copy
import math
from intera_interface import RobotParams

iteration = 10
distance = []

def move_to_home(limb, timeout=15.0, speed=0.1):
    try:
        neutral_pose = rospy.get_param("named_poses/{0}/poses/neutral".format(limb.name))
    except KeyError:
        rospy.logerr(("Get neutral pose failed, arm: \"{0}\".").format(limb.name))
        return
    angles = dict(list(zip(limb.joint_names(), neutral_pose)))
    angles.update(right_j0 = 165/180*math.pi,
    right_j5 = angles['right_j5'] - math.pi/2,
    right_j6 = angles['right_j6'] - math.pi )
    limb.set_joint_position_speed(speed)
    return limb.move_to_joint_positions(angles, timeout)


def interpolate_poses(start_pose, end_pose, steps):
    """Linear interpolation between two poses in Cartesian space (XYZ only)."""
    interpolated = []
    for i in range(1, steps + 1):
        pose = copy.deepcopy(start_pose)
        alpha = float(i) / steps
        pose.position.x = (1 - alpha) * start_pose.position.x + alpha * end_pose.position.x
        pose.position.y = (1 - alpha) * start_pose.position.y + alpha * end_pose.position.y
        pose.position.z = (1 - alpha) * start_pose.position.z + alpha * end_pose.position.z
        # Orientation remains fixed
        interpolated.append(pose)
    return interpolated

def run():
    """
    Main loop for the controller.
    """
    # Publisher for interaction control
    pub = rospy.Publisher('/robot/limb/right/interaction_control_command',
                                InteractionControlCommand, queue_size=10)

    # Publisher for collision avoidance suppression
    collision_suppress_pub = rospy.Publisher('/robot/limb/right/suppress_collision_avoidance',
                                                    Empty, queue_size=10)

    rospy.loginfo("Running Cartesian Impedance Controller.")

    # Start collision avoidance suppression in a separate thread
    import threading
    collision_thread = threading.Thread(target=self.suppress_collision_avoidance)
    collision_thread.daemon = True  # Ensure thread exits with the main program
    collision_thread.start()

    publish_control()

def main():
    rospy.init_node('x')

    limb = Limb()
    tip_name = "right_hand"

    # --- Impedance Control Setup ---
    interaction_options = InteractionOptions()
    interaction_options.set_interaction_control_active(True) # Enable interaction controller

    # Soft compliance mostly in Z direction (downward)
    interaction_options.set_K_impedance([1000.0, 1000.0, 1000.0, 40.0, 40.0, 40.0]) # [N/m, Nm/rad]

    # Optional: allow free movement in certain axes (disable max impedance)
    interaction_options.set_max_impedance([True, True, True, True, True, True])

    # Mode: 1 = Impedance (or try 3 = Impedance + force limit)
    interaction_options.set_interaction_control_mode([1, 1, 3, 1, 1, 1])
    # interaction_options.set_max_force_threshold([20.0]*6)
    # interaction_options.set_max_torque_threshold([10.0]*6)

    # interaction_options.set_interaction_control_mode([3, 3, 3, 1, 1, 1])
    interaction_options.set_force_command([100.0, 100.0, 100.0, 20.0, 20.0, 20.0])

    # # Optional: set endpoint frame (if needed)
    # interaction_options.set_endpoint_name("right_hand")
    # interaction_options.set_in_endpoint_frame(True)

    # Optional: prevent sudden jerk when applying new settings
    interaction_options.set_disable_reference_resetting(True)  
    interaction_options.set_disable_damping_in_force_control(True)

    # limb.set_interaction_options(interaction_options)

    rospy.loginfo("Moving to neutral...")
    move_to_home(limb=limb)
    rospy.sleep(1.0)

    # Get current pose
    endpoint_state = limb.tip_state(tip_name)
    start_pose = endpoint_state.pose

    # Define end pose
    end_pose = copy.deepcopy(start_pose)
    end_pose.position.z -= 0.30 # distance

    # Interpolate 5 Cartesian waypoints down
    waypoints_down = interpolate_poses(start_pose, end_pose, steps=40)

    # Cartesian trajectory
    traj_options = TrajectoryOptions()
    traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
    traj_options.interaction_params = interaction_options.to_msg()
    traj = MotionTrajectory(limb=limb)
    traj.set_trajectory_options(traj_options)

    # Waypoint options
    wpt_opts = MotionWaypointOptions(
    max_linear_speed=0.1,
    max_linear_accel=0.1,
    max_rotational_speed=0.1,
    max_rotational_accel=0.1,
    max_joint_speed_ratio=0.1
    )

    rospy.loginfo("Descending linearly...")
    for pose in waypoints_down:
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        joint_angles = limb.joint_ordered_angles() # Use current joints as nullspace seed
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)
        waypoint.set_cartesian_pose(pose_stamped, tip_name, joint_angles)
        traj.append_waypoint(waypoint.to_msg())

    result = traj.send_trajectory(timeout=60.0)
    if result and result.result:
        rospy.loginfo("Descent complete.")
    else:
        rospy.logwarn("Descent failed or timed out.")

    rospy.sleep(10)

    rospy.loginfo("Moving to neutral...")
    move_to_home(limb=limb)
    rospy.sleep(10)
    i = 1
    for i in range(1, len(iteration)):
        for distance in distance:
            # Get current pose
            endpoint_state = limb.tip_state(tip_name)
            start_pose = endpoint_state.pose

    # Define end pose (20 cm downward in Z)
    end_pose = copy.deepcopy(start_pose)
    end_pose.position.z -= 0.2775 # distance

    # Interpolate 5 Cartesian waypoints down
    waypoints_down = interpolate_poses(start_pose, end_pose, steps=20)

    rospy.loginfo("Descending linearly...")
    for pose in waypoints_down:
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        joint_angles = limb.joint_ordered_angles() # Use current joints as nullspace seed
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)
        waypoint.set_cartesian_pose(pose_stamped, tip_name, joint_angles)
        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory(timeout=15.0)
        if result and result.result:
            rospy.loginfo("Descent complete.")
        else:
            rospy.logwarn("Descent failed or timed out.")

    rospy.sleep(10)

    rospy.loginfo("Moving to neutral...")
    move_to_home(limb=limb)
    rospy.sleep(10)
    rospy.loginfo("")

    rospy.loginfo("Script finished.")


if __name__ == "__main__":
    main()
    run()
