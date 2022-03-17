"""Move Shadow hand to specified pose.

Based on:
https://dexterous-hand.readthedocs.io/en/latest/user_guide/3_software_description.html
"""

import argparse

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder


def main():
    args = parse_args()

    # Initialize our ROS node
    rospy.init_node("sr_hand", anonymous=True)

    # Find the connected Shadow Hand
    hand_finder = HandFinder()
    hand_parameters = hand_finder.get_hand_parameters()
    hand_serial = next(iter(hand_parameters.mapping))

    # Initialize hand connection
    hand_commander = SrHandCommander(
        args.robot_name,
        hand_parameters=hand_parameters,
        hand_serial=hand_serial
    )

    # Print some debug information
    print("Robot name: ", hand_commander.get_robot_name())
    print("Group name: ", hand_commander.get_group_name())
    print("Planning frame: ", hand_commander.get_planning_frame())

    # Refresh targets first if they have recently changed
    hand_commander.refresh_named_targets()
    print("Named targets: ", hand_commander.get_named_targets())

    # Change velocity & acceleration scaling
    hand_commander.set_max_velocity_scaling_factor(args.scaling)
    hand_commander.set_max_acceleration_scaling_factor(args.scaling)

    # Move to specified pose
    # wait=True (default) indicates that function blocks until completed
    for pose in  args.poses:
        print('Moving to:', pose)
        hand_commander.move_to_named_target(pose)
    print('Movements completed.')



def parse_args():
    parser = argparse.ArgumentParser('Connects to Shadow Hand and specifies pose.')
    parser.add_argument(
        '-p',
        '--poses',
        nargs='+',
        help='Pose to move the hand to.',
    )
    parser.add_argument(
        '-n',
        '--robot-name',
        default='right_hand',
        help='robot name',
    )
    parser.add_argument(
        '-s',
        '--scaling',
        type=float,
        default=1,
        help='velocity and acceleration scaling (0, 1]',
    )

    args = parser.parse_args()
    return args


if __name__ == "__main__":
    main()
