#! /usr/bin/env python

# Disclaimer: this script is based based on https://github.com/CarmineD8/robot_description/blob/main/scripts/bug.py

import rospy
import time
from threading import Lock
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *
from geometry_msgs.msg import Twist
from ros_assignments.srv import PositionGenerator
from ros_assignments.srv import UserCommand, UserCommandResponse

import math

pub = None
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
srv_client_user_interface_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
position_ = Point()
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'wall following', 'new random point requested', 'joystick mode', 'idle']
state_ = 0
mode_ = 'idle'
random_mode_requested_ = False
desired_v_ = 0.0
desired_w_ = 0.0
mutex_ = Lock()
# 0 - go to point
# 1 - wall following

# callbacks


def clbk_odom(msg):
    global position_, yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }


def clbk_user_command(request):
    global mode_
    global desired_v_
    global desired_w_
    global random_mode_requested_

    mode = ''
    outcome = 'Success.'

    available_modes = ['idle', 'joystick', 'random']
    if request.mode in available_modes:
        mode = request.mode

        if mode == 'idle':
            change_state(4)

        elif mode == 'joystick':
            desired_v_ = request.v_forward
            desired_w_ = request.w_yaw

            change_state(3)

        elif mode == 'random':
            # change_state(2) should be called but it will cause calling a ROS2 service
            # before returning the answer to this service call, hence it is deferred to the main loop
            random_mode_requested_ = True

    else:
        outcome = 'Failed. Available modes are ' + str(available_modes) + '.'

    return outcome


def send_velocity(v_forward, w_yaw):
    global pub

    twist_msg = Twist()
    twist_msg.linear.x = v_forward
    twist_msg.angular.z = w_yaw

    pub.publish(twist_msg)


def get_state():
    global state_
    global mutex_
    global random_mode_requested_

    state = None
    random_mode_requested = None

    mutex_.acquire()
    state = state_
    random_mode_requested = random_mode_requested_
    mutex_.release()

    return state, random_mode_requested


def get_desired_position():
    global mutex_
    global desired_position_

    desired_position = None

    mutex_.acquire()
    desired_position = desired_position_
    mutex_.release()

    return desired_position


def get_desired_velocity():
    global mutex_
    global desired_v_, desired_w_

    desired_v = None
    desired_w = None

    mutex_.acquire()
    desired_v = desired_v_
    desired_w = desired_w_
    mutex_.release()

    return desired_v, desired_w


def clear_random_mode_requested():
    global random_mode_requested_

    mutex_.acquire()
    random_mode_requested_ = False
    mutex_.release()


def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_, srv_client_position_generator_
    global desired_position_, mutex_

    mutex_.acquire()

    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)

    if state_ == 0:
        # go to point
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)

    if state_ == 1:
        # wall follower
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)

    if state_ == 2:

        # stop any ongoing motion
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
        send_velocity(0.0, 0.0)

        # get new random point
        resp = srv_client_position_generator_()
        desired_position_.x = resp.x
        desired_position_.y = resp.y

        # set desired position on the parameter server for pre-existing modules
        rospy.set_param("des_pos_x", desired_position_.x)
        rospy.set_param("des_pos_y", desired_position_.y)

        # log desired position
        rospy.loginfo('new random position')
        rospy.loginfo(desired_position_.x)
        rospy.loginfo(desired_position_.y)

    if state_ == 3:
        # command requested velocity

        # before commanding in the while loop, stop any ongoing motion
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
        send_velocity(0.0, 0.0)

    if state_ == 4:
        # stop any ongoing motion
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
        send_velocity(0.0, 0.0)

    mutex_.release()


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def main():
    time.sleep(2)
    global regions_, position_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, srv_client_user_interface_, srv_client_position_generator_, pub

    rospy.init_node('bug0')

    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    srv_client_user_interface_ = rospy.ServiceProxy('/user_interface', Empty)
    srv_client_position_generator_ = rospy.ServiceProxy('/position_generation_service', PositionGenerator)

    srv_server_user_command_ = rospy.Service('user_command_service', UserCommand, clbk_user_command)

    # initialize as idle state
    change_state(4)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue

        # rospy.loginfo(state_)

        state, random_mode_requested = get_state()
        desired_position = get_desired_position()
        desired_v, desired_w = get_desired_velocity()

        # Check if random mode has been requested
        if random_mode_requested:
            change_state(2)
            clear_random_mode_requested()

        if state == 0:
            err_pos = math.sqrt(pow(desired_position.y - position_.y,
                                    2) + pow(desired_position.x - position_.x, 2))
            if(err_pos < 0.3):
                change_state(2)

            elif regions_['front'] < 0.2:
                change_state(1)

        elif state == 1:
            desired_yaw = math.atan2(
                desired_position.y - position_.y, desired_position.x - position_.x)
            err_yaw = normalize_angle(desired_yaw - yaw_)
            err_pos = math.sqrt(pow(desired_position.y - position_.y,
                                    2) + pow(desired_position.x - position_.x, 2))

            if(err_pos < 0.3):
                change_state(2)
            if regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
                change_state(0)

        elif state == 2:
            err_pos = math.sqrt(pow(desired_position_.y - position_.y,
                                    2) + pow(desired_position_.x - position_.x, 2))
            if(err_pos > 0.35):
                change_state(0)

        elif state == 3:
            send_velocity(desired_v, desired_w)

        rate.sleep()


if __name__ == "__main__":
    main()
