import get_skew_data
import hand_feature_generation
from get_skew_data import *


def sampleCone(obj_transform, hand_transform, sample_rate, radiusA, radiusB, angle): #T_reference_source
    transforms = []
    roll_transform = rotation_matrix_from_rpy(-angle, 0, 0)
    hand_transform = numpy.dot(hand_transform, roll_transform)
    T_between_init = getTransformBetweenLinks(hand_transform, obj_transform) #T_obj_hand
    for theta in range(0, sample_rate):
        x = radiusA * math.cos(theta * math.pi/sample_rate)
        y = radiusB * math.sin(theta * math.pi/sample_rate)
        T_R = rotation_matrix_from_rpy(0,theta * math.pi/sample_rate,0) #rotation to apply to hand
        T_rotation = numpy.eye(4)
        T_rotation[0:3,0:3] = T_R
        T_rotation[0][3] = x
        T_rotation[2][3] = y
        T_between = numpy.dot(T_between_init, T_rotation) #T_obj_hand
        T_world_hand = numpy.dot(T_obj, T_between) #T_world_hand = T_world_obj * T_obj_hand
        transforms.append(T_world_hand)
    return transforms


def sampleEllipsoid(obj_transform, hand_transform, sample_rate, radiusA, radiusB, radiusC):
    
