import sys
import os
import time
from Pangolin_Kinematic import PangolinKinematic
from Pangolin_ControlCmd import PangolinControl

class HeadShakeTail:
    def __init__(self):
        # Initialize the kinematics object
        self.kinematic = PangolinKinematic()
        self.controlcmd = PangolinControl()

        # Properly initialize attributes required for `spine_controller`
        self.kinematic.gait_name = 'move_linear'  # Set a default gait name
        self.kinematic.is_walking = True          # Simulate walking state
        self.kinematic.req_vel.angular.z = 0.0    # Neutral angular velocity
        self.kinematic.req_vel.linear.x = 1.0     # Forward linear velocity

    def walk_and_shake_tail(self, iterations=20, delay=0.1):
        """Walk forward and shake the tail simultaneously."""
        spine_shake_state = 1  # 1 for one direction, -1 for the opposite
        spine_shake_amplitude = 16  # Maximum spine angle amplitude
        spine_shake_step = 4  # Increment for the shaking motion

        for i in range(iterations):
            # Walk forward (linear velocity is set to 1.0 in initialization)
            leg_angle, head_angle, spine_angle = self.kinematic.calculate_joint(
                self.kinematic.gait_name,
                self.kinematic.leg_position,
                self.kinematic.req_vel,
                self.kinematic.is_walking
            )

            # Generate dynamic spine angle for tail shaking
            spine_shake_value = spine_shake_state * spine_shake_amplitude
            spine_shake_amplitude -= spine_shake_step * spine_shake_state
            if spine_shake_amplitude <= -16 or spine_shake_amplitude >= 16:
                spine_shake_state *= -1  # Reverse direction

            # Apply the shaking value to the spine angle
            spine_angle = [spine_shake_value, 0]

            # Simulate motor control (output angles)
            print(f"Iteration {i + 1}: Walking Leg Angles: {leg_angle}, Head Angles: {head_angle}, Spine Angles: {spine_angle}")

            # Simulate delay for smooth movement
            time.sleep(delay)


if __name__ == "__main__":
    # Ensure the script works regardless of module paths
    sys.path.append(os.path.expanduser('~/pangolin_ws/src/quadruped_robot_4_DOF/pangolin_base'))

    # Initialize and run the walking and tail-shaking functionality
    walker = HeadShakeTail()
    walker.walk_and_shake_tail(iterations=2000, delay=0.1)  # Walk and shake tail 20 times with 0.1s delay
