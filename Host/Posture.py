from vpython import *

def posture_init():

    scene = canvas(width=800, height=600, center=vector(0, 0, 0), background=color.white)


    arrow(pos=vector(0, 0, 0), axis=vector(5, 0, 0), color=color.red, shaftwidth=0.1)  # x轴
    arrow(pos=vector(0, 0, 0), axis=vector(0, 5, 0), color=color.green, shaftwidth=0.1)  # y轴
    arrow(pos=vector(0, 0, 0), axis=vector(0, 0, 5), color=color.blue, shaftwidth=0.1)  # z轴


    length = 8  
    width = 3  
    height = 0.5  
    cuboid = box(pos=vector(0, 0, 0), length=length, width=width, height=height, color=color.gray(0.7))

    return cuboid, scene


def posture(cuboid, scene, pitch_in, roll_in, yaw_in):

    pitch = radians(pitch_in)
    roll = radians(roll_in)
    yaw = radians(yaw_in)

    cuboid.up = vector(0, 1, 0)
    cuboid.axis = vector(0, 0, 1)

    cuboid.rotate(angle=yaw, axis=vector(0, 1, 0), origin=vector(0, 0, 0))


    cuboid.rotate(angle=-pitch, axis=vector(0, 0, 1), origin=vector(0, 0, 0))


    cuboid.rotate(angle=-roll, axis=vector(1, 0, 0), origin=vector(0, 0, 0))


