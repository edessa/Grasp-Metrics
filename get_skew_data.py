import openravepy
import numpy
from scipy import spatial
from points_bounding_box import *
import math
from hand_feature_generation import *

def bounding_box(item):
	obj = item.ComputeAABB()
	max_xyz =  obj.pos()+obj.extents()
	min_xyz = obj.pos()-obj.extents()
	return min_xyz, max_xyz

def getMeshTree(filename):
	b = numpy.loadtxt(filename, dtype=float)
	kd_tree_list = list(numpy.reshape(b, (-1,3)))
	tree = spatial.cKDTree(kd_tree_list)
	return tree

def BoundingBox_param(item):
	bbox = item.ComputeAABB()
	orig = bbox.pos()
	extents = bbox.extents()
	return orig, extents


def quat_from_rpy(roll, pitch, yaw):
    cr, cp, cy = math.cos(roll / 2), math.cos(pitch / 2), math.cos(yaw / 2)
    sr, sp, sy = math.sin(roll / 2), math.sin(pitch / 2), math.sin(yaw / 2)
    return numpy.array([
        cr * cp * cy + sr * sp * sy,
        -cr * sp * sy + cp * cy * sr,
        cr * cy * sp + sr * cp * sy,
        cr * cp * sy - sr * cy * sp])


def rotation_matrix_from_rpy(roll, pitch, yaw):
	return openravepy.rotationMatrixFromQuat(quat_from_rpy(roll, pitch, yaw))

def get_positions_wrt_obj(item, filename, distance, skew, step):
	#kd_mesh = getMeshTree(filename)
	#point = kd_mesh.query(point_from_bounding_box)

	orig, extents = BoundingBox_param(item)
	#principal_axes = ['x', 'z', 'y'][extents.index(max(extents))]
	#faces = draw_BoundingBox(orig, extents, principal_axes, step)
	faceSet1 = draw_BoundingBox(orig, extents, 'z', step)
	faceSet2 = draw_BoundingBox(orig, extents, 'x', step)

	faces = [faceSet1[0], faceSet1[2], faceSet2[0],faceSet2[2]]
	rotation = [[math.pi,0,0], [0, 0, 0], [0, -math.pi/2, 0],[0, math.pi/2, 0]]
	distance_offset = [[0, 0, distance], [0, 0, -distance], [distance, 0, 0], [-distance, 0, 0]]
	T_hand = numpy.eye(4)
	R_hand = T_hand[0:3,0:3]
	transforms_aligned = transforms_skewed = []

	for i in range(0, 4):
		for j in range(0, len(faces[i])):
			T_hand = openravepy.matrixFromAxisAngle(rotation[i])
			T_hand[0][3] = faces[i][j][0] + distance_offset[i][0]
			T_hand[1][3] = faces[i][j][1] + distance_offset[i][1]
			T_hand[2][3] = faces[i][j][2] + distance_offset[i][2]
			transforms_aligned.append(T_hand)

			for s in range(0, 3):
				skewForward = skewBackward = [0, 0, 0]
				skewForward[s] = skew
				skewBackward[s] = -skew
				T_forward = numpy.matmul(T_hand, openravepy.matrixFromAxisAngle(skewForward))
				T_backward = numpy.matmul(T_hand, openravepy.matrixFromAxisAngle(skewBackward))
				transforms_skewed.append(T_forward)
				transforms_skewed.append(T_backward)

	return transforms_aligned, transforms_skewed

def collectData():
	env = openravepy.Environment()
	robot = loadRobot(env)
	item = loadObject(env)
	transforms_aligned, transforms_skewed = get_positions_wrt_obj(item, 'SprayBottle.out', 0.13, 0.1, 10)
	for i in range(0, len(transforms_aligned)):
		robot.SetTransform(transforms_aligned[i])
		generateHandFeatures('./aligned_data/Transforms_Aligned' + str(i) + '.out', robot, item)

	for j in range(0, len(transforms_skewed)):
		robot.SetTransform(transforms_skewed[j])
		generateHandFeatures('./skewed_data/Transforms_Skewed' + str(j) + '.out', robot, item)
