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

    # Find the connected Shadow Hand
    hand_finder = HandFinder()
    hand_parameters = hand_finder.get_hand_parameters()
    hand_serial = hand_parameters.mapping.keys()[0]

    # Initialize our ROS node
    rospy.init_node("sr_hand_commander", anonymous=True)
    # Initialize hand connection
    hand_commander = SrHandCommander(
        args.group_name,
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

    # Move to specified pose
    # wait=True (default) indicates that function blocks until completed
    print('Moving to:', args.pose)
    hand_commander.move_to_named_target(args.pose)
    print('Movement completed.')



def parse_args():
    parser = argparse.ArgumentParser('Connects to Shadow Hand and specifies pose.')
    parser.add_argument(
        '-p',
        '--pose',
        required=True,
        help='date string, such as 20210201',
    )
    parser.add_argument(
        '-n',
        '--robot-name',
        default='right_hand',
        help='robot group name',
    )

    args = parser.parse_args()
    return args


if __name__ == "__main__":
    main()
