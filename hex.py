#!python3

import maestro
import transformations as tf
import numpy as np
import time
from operator import add
from math import *
import matplotlib
from matplotlib.pyplot import *
import sys
import select
import tty, sys

pod_position = [0, 0, 80]       # coordinates in mm
pod_angle = [0, 0, 0]           # angles in radians
pod_position_offset = [0, 0, 0] # coordinates in mm
pod_angle_offset = [0, 0, .5]    # angles in radians

class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


getch = _Getch()

# servo target difference of 700 ~= 90 degrees = pi/2, center at 1500
# servo library adds in precision multiplier of 4
def angle_to_steps(degrees):
    return int((1500+degrees*700/(pi/2))*4)

def string_point(point):
    plot(point[0], point[1], 'o')
    return '({}, {}, {})'.format(point[0], point[1], point[2])

def line(points):
    x = []
    y = []
    for point in points:
        x.append(point[0])
        y.append(point[1])
    plot(x, y)

def normalize(avector):
    value = np.linalg.norm(avector)
    result = []
    for val in avector:
        result.append(val*1.0/value)
    return np.array(result)
       
class Leg:
    def __init__(self, rel_xyz, xyz, servos):
        """set x, y, z in mm relative to body center and absolute leg position"""
        """servos is indexable of servo indices (e.g., (0, 2, 3))"""
        self.rel_x, self.rel_y, self.rel_z = rel_xyz
        self.x, self.y, self.z = xyz
        self.servos = servos
        self.coxa_length = 61
        self.femur_length = 100
        self.tibia_length = 145
    def get_coxa_position(self, body_xyz, body_angle):
        """find x, y, z of coxa joint in mm given body position"""
        a,b,c = body_angle
        u,v,w = body_xyz
        rotation = tf.euler_matrix(a, b, c, 'rxyz')
        translation = tf.translation_matrix([u, v, w])
        transform = tf.concatenate_matrices(rotation, translation)
        position = transform.dot([self.rel_x, self.rel_y, self.rel_z, 1])
        reference1 = transform.dot([100, 100, 0, 1]) # another vector on the body plane
        reference2 = transform.dot([-100, 100, 0, 1]) # another vector on the body plane
        #print("\tBody position xyz: "+string_point([u,v,w]))
        #print("\tCoxa joint xyz: "+string_point(position))
        #print("\tLeg xyz: "+string_point([self.x, self.y, self.z]))
        return (position, reference1, reference2) # have extra indices
    def get_coxa_angle(self, body_xyz, body_angle):
        """find angle in radians of coxa joint relative to (body center, coxa joint) vector"""
        bx,by,bz = body_xyz
        (u,v,w,_),(u2,v2,w2,_),(u3,v3,w3,_) = self.get_coxa_position(body_xyz, body_angle) # location of coxa joint
        body_vec = np.array([bx-u, by-v, bz-w]) # vector from joint to body
        #print("\tCoxa joint to body: "+string_point(body_vec))
        leg_vec = np.array([self.x-u, self.y-v, self.z-w]) # vector from joint to leg end
        #print("\tCoxa joint to leg: "+string_point(leg_vec))
        body_vec1 = np.array([u2-bx, v2-by, w2-bz]) # vector from reference 1 to body
        body_vec2 = np.array([u3-bx, v3-by, w3-bz]) # vector from reference 2 to body
        # project onto body plane
        leg_vec_flat = np.add(float(np.dot(leg_vec, body_vec1))/float(np.dot(leg_vec,leg_vec))*body_vec1,
                              float(np.dot(leg_vec, body_vec2))/float(np.dot(leg_vec,leg_vec))*body_vec2)
        #print("\tCoxa joint to leg, flat to body: "+string_point(leg_vec_flat))
        #print("\tCoxa normalized flat: "+string_point(normalize(leg_vec_flat)))
        #print("\tCoxa normalized to body: "+string_point(normalize(body_vec)))
        
        angle = (normalize(leg_vec_flat).dot(normalize(body_vec)))
        print("\tCoxa joint angle: {}".format(angle))
        cross = np.cross(leg_vec_flat, body_vec);
        if np.cross(body_vec1, body_vec2).dot(cross) < 0:
            angle = -angle;
        return 0
        return angle
    def get_femur_position(self, body_xyz, body_angle):
        """find xyz position of femur joint under given conditions"""
        pass
    def get_femur_tibia_angles(self, body_xyz, body_angle):
        """find angles in radians of femur and tibia joints under given conditions"""
        return (0, 0)
    def update(self, body_xyz, body_angle):
        coxa_angle = self.get_coxa_angle(body_xyz, body_angle)
        femur_angle, tibia_angle = self.get_femur_tibia_angles(body_xyz, body_angle)
        pod.setTarget(self.servos[0], angle_to_steps(coxa_angle))
        pod.setTarget(self.servos[1], angle_to_steps(femur_angle))
        pod.setTarget(self.servos[2], angle_to_steps(tibia_angle))
    def initialize(self):
        pod.setTarget(self.servos[0], angle_to_steps(0))
        pod.setTarget(self.servos[1], angle_to_steps(0))
        pod.setTarget(self.servos[2], angle_to_steps(0))

##### MAIN #####

pod = maestro.Controller()
legs = []
legs.append(Leg( (40,70,0), (75,130,0), (0,6,12) ))
legs.append(Leg( (65,0,0), (150,0,0), (1, 7, 13) ))
legs.append(Leg( (40,-70,0), (75,-130,0), (2, 8, 14) ))
legs.append(Leg( (-40,70,0), (-75,130,0), (3, 9, 15) ))
legs.append(Leg( (-65,0,0), (-150,0,0), (4, 10, 16) ))
legs.append(Leg( (-40,-70,0), (-75,-130,0), (5, 11, 17) ))


enable = True
if enable:
    for leg in legs:
        leg.initialize()
        time.sleep(0.5)

dance = False
walk = True
if (not dance and not walk) and False:
    body_xyz = np.add(pod_position,pod_position_offset)
    body_abc = np.add(pod_angle,pod_angle_offset)
    print("Body XYZ: {}".format(body_xyz))
    print("Body ABC: {}".format(body_abc))
    for index, leg in enumerate(legs):
        print("Leg {}:".format(index))
        #print("\tLeg position: " + string_point([leg.x, leg.y, leg.z]))
        leg.get_coxa_angle(body_xyz, body_abc)
        #leg.update(body_xyz, body_abc)

delay = 0.1
forward = 0.5
height = 0.9

def right_up():
    # right up
    pod.setTarget(6, angle_to_steps(height))
    pod.setTarget(8, angle_to_steps(height))
    pod.setTarget(10, angle_to_steps(-height))
    time.sleep(delay)
def right_forward(d):
    # right forward
    pod.setTarget(0, angle_to_steps(-forward*d))
    pod.setTarget(2, angle_to_steps(-forward*d))
    pod.setTarget(4, angle_to_steps(forward*d))
    time.sleep(delay)
def right_forward_rotate(d):
    pod.setTarget(0, angle_to_steps(-forward*d))
    pod.setTarget(2, angle_to_steps(-forward*d))
    pod.setTarget(4, angle_to_steps(-forward*d))
    time.sleep(delay)
def left_back_rotate(d):
    pod.setTarget(1, angle_to_steps(-forward*d))
    pod.setTarget(3, angle_to_steps(-forward*d))
    pod.setTarget(5, angle_to_steps(-forward*d))
    time.sleep(delay)
def left_back(d):
    pod.setTarget(1, angle_to_steps(forward*d))
    pod.setTarget(3, angle_to_steps(-forward*d))
    pod.setTarget(5, angle_to_steps(-forward*d))
    time.sleep(delay)
def right_down():
    pod.setTarget(6, angle_to_steps(0))
    pod.setTarget(8, angle_to_steps(0))
    pod.setTarget(10, angle_to_steps(0))
    time.sleep(delay)
def left_up():
    pod.setTarget(7, angle_to_steps(height))
    pod.setTarget(9, angle_to_steps(-height))
    pod.setTarget(11, angle_to_steps(-height))
    time.sleep(delay)
def left_forward(d):
    pod.setTarget(1, angle_to_steps(-forward*d))
    pod.setTarget(3, angle_to_steps(forward*d))
    pod.setTarget(5, angle_to_steps(forward*d))
    time.sleep(delay)
def right_back(d):
    pod.setTarget(0, angle_to_steps(forward*d))
    pod.setTarget(2, angle_to_steps(forward*d))
    pod.setTarget(4, angle_to_steps(-forward*d))
    time.sleep(delay)
def left_down():
    pod.setTarget(7, angle_to_steps(0))
    pod.setTarget(9, angle_to_steps(0))
    pod.setTarget(11, angle_to_steps(0))
    time.sleep(delay)

def dowalk(d):
    right_up()
    right_forward(d)
    left_back(d)
    right_down()

    left_up()
    left_forward(d)
    right_back(d)
    left_down()

def turn(d):
    right_up()
    right_forward_rotate(d)
    left_back_rotate(-d)
    right_down()

    left_up()
    left_back_rotate(d)
    right_forward_rotate(-d)
    left_down()

if walk:
    # initialize
    for servo in range(12):
        pod.setTarget(servo, angle_to_steps(0))
        time.sleep(0.1)
    for servo in [12, 13, 14]:
        pod.setTarget(servo, angle_to_steps(1.2))
        time.sleep(0.1)
    for servo in [15, 16, 17]:
        pod.setTarget(servo, angle_to_steps(-1.2))
        time.sleep(0.1)

    x = ' '
    while True:
        while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if line:
                if str(line)[0] in ['j', 'k', 'h', 'l', ' ', 'q', 'S', 's', 'h', 'H', 'f', 'F']:
                    x = str(line)[0]
            else: # an empty line means stdin has been closed
                print('eof')
                exit(0)
        else:
            if x == 'k':
                dowalk(1)
            if x == 'j':
                dowalk(-1)
            if x == 'h':
                turn(1)
            if x == 'l':
                turn(-1)
            if x == 's':
                delay /= 1.2
                x = ' '
            if x == 'S':
                delay *= 1.2
                x = ' '
            if x == 'f':
                forward /= 1.2
                x = ' '
            if x == 'F':
                forward *= 1.2
                x = ' '
            if x == 'h':
                height /= 1.2
                x = ' '
            if x == 'H':
                height *= 1.2
                x = ' '
            if x == 'q':
                pod.close()
                exit(0)
        time.sleep(0.001)

if dance:
    angle = 0
    while True:
        angle += 0.01
        for servo in range(18):
            pod.setTarget(servo, angle_to_steps(.5*sin(angle)))
            time.sleep(0.0001)

def something(line):
  print('read input:', line, end='')

def something_else():
  print('no input')

# If there's input ready, do something, else do something
# else. Note timeout is zero so select won't block at all.

pod.close()

