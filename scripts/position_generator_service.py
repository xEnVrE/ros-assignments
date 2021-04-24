#! /usr/bin/env python

import random
import rospy
from ros_assignments.srv import PositionGenerator, PositionGeneratorResponse


def position_generator_callback(request):
    """Return a random position in [-8.0, 8.0] x [-8.0, 8.0]."""

    response = PositionGeneratorResponse()

    response.x = random.uniform(-8.0, 8.0)
    response.y = random.uniform(-8.0, 8.0)

    return response


def main():
    """Main."""

    # Initialize node and associated service
    rospy.init_node('position_generation_service')
    srv = rospy.Service('position_generation_service', PositionGenerator, position_generator_callback)

    # Spin
    rospy.spin()


if __name__ == '__main__':
    main()
