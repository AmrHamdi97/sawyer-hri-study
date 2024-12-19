#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from intera_core_msgs.msg import InteractionControlCommand
from intera_motion_interface import InteractionOptions, InteractionPublisher
from std_msgs.msg import Bool  # For receiving error state

def setup_cartesian_impedance_control(stiffness, damping, max_stiffness, interaction_frame, control_mode, nullspace_stiffness):
    interaction_options = InteractionOptions()

    # Activate interaction control
    interaction_options.set_interaction_control_active(True)
    interaction_options.set_K_impedance(stiffness)
    interaction_options.set_K_nullspace(nullspace_stiffness)
    interaction_options.set_max_impedance(max_stiffness)
    interaction_options.set_interaction_control_mode(control_mode)
    interaction_options.set_in_endpoint_frame(True)
    interaction_options.set_interaction_frame(interaction_frame)
    interaction_options.set_disable_damping_in_force_control(False)
    interaction_options.set_disable_reference_resetting(False)
    interaction_options.set_rotations_for_constrained_zeroG(False)

    msg = interaction_options.to_msg()
    msg.D_impedance = damping

    return msg

def shutdown_handler(ic_pub):
    """
    Handler to switch back to position control mode upon shutdown.
    """
    rospy.loginfo("Shutting down and switching to position control mode.")
    ic_pub.send_position_mode_cmd()

class CartesianImpedanceController:
    def __init__(self):
        rospy.init_node('cartesian_impedance_controller')

        # Default stiffness and error stiffness
        self.default_stiffness = [500.0, 30.0, 30.0, 30.0, 30.0, 30.0]  # Updated based on requirements
        self.error_stiffness = [500.0, 300.0, 300.0, 30.0, 30.0, 30.0]  # High stiffness for error correction
        self.current_stiffness = self.default_stiffness
        self.error_detected = False

        # Initialize other parameters
        self.damping = rospy.get_param("~damping", [8.0, 8.0, 8.0, 2.0, 2.0, 2.0])
        self.nullspace_stiffness = rospy.get_param("~nullspace_stiffness", [10.0] * 7)
        self.max_stiffness = rospy.get_param("~max_stiffness", [False, False, False, False, False, False])
        self.control_mode = rospy.get_param("~control_mode", [1, 1, 1, 1, 1, 1])

        # Define a neutral interaction frame (identity matrix)
        self.interaction_frame = Pose()
        self.interaction_frame.position.x = 0.0
        self.interaction_frame.position.y = 0.0
        self.interaction_frame.position.z = 0.0
        self.interaction_frame.orientation.w = 1.0
        self.interaction_frame.orientation.x = 0.0
        self.interaction_frame.orientation.y = 0.0
        self.interaction_frame.orientation.z = 0.0

        # Initialize the interaction publisher
        self.ic_pub = InteractionPublisher()

        # Subscribe to the error state topic
        rospy.Subscriber('/error_state', Bool, self.error_state_callback)

        # Register the shutdown handler
        rospy.on_shutdown(lambda: shutdown_handler(self.ic_pub))

    def error_state_callback(self, msg):
        """
        Callback to handle error state changes.
        """
        if msg.data and not self.error_detected:
            self.error_detected = True
            self.current_stiffness = self.error_stiffness
            rospy.loginfo("Error detected: Stiffness adjusted to %s", self.current_stiffness)
        elif not msg.data and self.error_detected:
            self.error_detected = False
            self.current_stiffness = self.default_stiffness
            rospy.loginfo("Error resolved: Stiffness reverted to %s", self.current_stiffness)

    def publish_control(self):
        """
        Continuously publish Cartesian Impedance Control settings.
        """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Generate the interaction message
            interaction_msg = setup_cartesian_impedance_control(
                self.current_stiffness,
                self.damping,
                self.max_stiffness,
                self.interaction_frame,
                self.control_mode,
                self.nullspace_stiffness,
            )
            rospy.loginfo("Publishing stiffness: %s", self.current_stiffness)

            # Publish the interaction command
            self.ic_pub.send_command(interaction_msg, 10)
            rate.sleep()

    def run(self):
        """
        Main loop for the controller.
        """
        self.publish_control()

if __name__ == '__main__':
    try:
        controller = CartesianImpedanceController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupt detected, exiting...")

