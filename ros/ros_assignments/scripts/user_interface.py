#! /usr/bin/env python

import curses
import rospy
from curses import wrapper
from ros_assignments.srv import UserCommand, UserCommandRequest


def draw_status(screen, status):
    """Draw application status."""

    screen.addstr(5, 0, 'Status:          ')
    screen.refresh()
    screen.addstr(5, 0, 'Status: ' + status)


def draw_robot(screen, status):
    """Draw robot."""

    if status == 'up':
        screen.addstr(7 , 4, '     ^    ')
        screen.addstr(8 , 4, ' _   |   _')
    elif status == 'idle':
        screen.addstr(7 , 4, '          ')
        screen.addstr(8 , 4, ' _       _')

    if status == 'right':
        screen.addstr(9 , 4, '/_/_____/_/')
    elif status == 'left':
        screen.addstr(9 , 4, '\\_\\_____\\_\\')
    else:
        screen.addstr(9 , 4, '|_|_____|_|')
    screen.addstr(10, 4, ' |       |')
    screen.addstr(11, 4, ' |       |')
    screen.addstr(12, 4, ' |       |')
    screen.addstr(13, 4, ' | robot |')
    screen.addstr(14, 4, ' |       |')
    screen.addstr(15, 4, '|_|_____|_|')

    if status == 'down':
        screen.addstr(17 , 4, '     |    ')
        screen.addstr(18 , 4, '     V    ')
    elif status == 'idle':
        screen.addstr(17 , 4, '          ')
        screen.addstr(18 , 4, '          ')


def close(screen):
    """Close the interface."""

    curses.nocbreak()
    screen.keypad(False)
    curses.echo()
    curses.endwin()


def main(screen):
    """Main."""

    status = 'idle'
    robot_status = 'idle'
    last_sent = ''

    # init ros node
    rospy.init_node('user_interface')

    # service client
    srv_client_user_command = rospy.ServiceProxy('/user_command_service', UserCommand)

    # useful curses options
    curses.cbreak()
    curses.curs_set(False)
    curses.noecho()
    screen.nodelay(True)

    # welcome screen
    screen.addstr(0, 0, '*** User interface ***')
    screen.addstr(2, 0, 'Available commands: q - close, i - idle, r - random')
    screen.addstr(3, 0, '                    w - go forward, a - turn left, s - go backward, d - turn right')
    draw_status(screen, status)
    draw_robot(screen, robot_status)
    screen.refresh()

    # robot velocity
    v_forward = 0.0
    w_yaw = 0.0

    elapsed = 0.0

    period = 20
    rate = rospy.Rate(period)

    # handle user input periodically
    while not rospy.is_shutdown():

        c = screen.getch()
        if c == ord('q'):
            srv_client_user_command(mode = 'idle')

            close(screen)
            exit(0)

        elif c == ord('r'):
            status = 'random'

        elif c == ord('i'):
            status = 'idle'

        elif c == ord('w'):
            status = 'joystick'
            v_forward = 0.5
            w_yaw = 0.0
            robot_status = 'up'

        elif c == ord('a'):
            status = 'joystick'
            v_forward = 0.0
            w_yaw = 1.0
            robot_status = 'left'

        elif c == ord('s'):
            status = 'joystick'
            v_forward = -0.5
            w_yaw = 0.0
            robot_status = 'down'

        elif c == ord('d'):
            status = 'joystick'
            v_forward = 0.0
            w_yaw = -1.0
            robot_status = 'right'


        # avoid spiky commands with a buffer of 0.5 seconds
        if status == 'joystick':
            elapsed += period / 1000.0

            if last_sent != 'joystick':
                srv_client_user_command(mode = 'joystick', v_forward = v_forward, w_yaw = w_yaw)
                last_sent = 'joystick'

            if elapsed > 0.3:
                if (c != ord('w')) and\
                   (c != ord('a')) and\
                   (c != ord('s')) and\
                   (c != ord('d')):
                    elapsed = 0
                    status = 'idle'
                    robot_status = 'idle'

        if status == 'idle':
            if last_sent != 'idle':
                srv_client_user_command(mode = 'idle')
                last_sent = 'idle'
        elif status == 'random':
            if last_sent != 'random':
                srv_client_user_command(mode = 'random')
                last_sent = 'random'

        draw_status(screen, status)
        draw_robot(screen, robot_status)

        screen.refresh()

        rate.sleep()


if __name__ == '__main__':
    wrapper(main)
