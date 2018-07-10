import openravepy
import numpy
from scipy import spatial
from points_bounding_box import *
import math
from hand_feature_generation import *
import random
from subprocess import call
from stl2obj import convert
import stl
from stl import mesh


def getUniformQuatDistribution(N):
	rotations = []
	for i in range(0, N):
		s = random.random()
		phi1 = (1-s)**0.5
		phi2 = s**0.5
		theta1 = 2 * math.pi * random.random()
		theta2 = 2 * math.pi * random.random()
		w = math.cos(theta2) * phi2
		x = math.sin(theta1) * phi1
		y = math.cos(theta1) * phi1
		z = math.sin(theta2) * phi2
		rotations.append([w, x, y, z])
	return rotations

def getTranslations(N, size):
	points = []
	for x in pylab.frange(-size/2.0, size/2.0, size/float(N)):
		for z in pylab.frange(-size/2.0, size/2.0, size/float(N)):
				points.append([x, 0, z])
	return points


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

def sample_obj_transforms():
	transforms = []
	translations = getTranslations(3, .1)

	for trans in translations:
		uniformly_sampled_rotations = getUniformQuatDistribution(1000)
		for quat in uniformly_sampled_rotations:
			R = openravepy.rotationMatrixFromQuat(quat)
			T_obj = numpy.eye(4)
			T_obj[0:3,0:3] = R
			T_obj[0][3] = trans[0]
			T_obj[1][3] = trans[1]
			T_obj[2][3] = -0.3 + trans[2] #depth offset
			transforms.append(T_obj)
	return transforms

def generateParameters(type, N):
	parameters = []
	shapeResolutions = {'cube':10, 'ellipse':100, 'cylinder':25, 'cone':25, 'vase':25}
	resolution = shapeResolutions[type]
	if type == 'cone':
		n = int(N**(1./2.))
		for width in pylab.frange(0.05, 0.15, 0.1/n):
			for height in pylab.frange(0.05, 0.15, 0.1/n):
				parameters.append([width, height, 0.1, 0, 0, resolution])
	else:
		n = int(N**(1./2.))
		for height in pylab.frange(0.05, 0.15, 0.1/n):
			for alpha in pylab.frange(10, 40, 30/n):
				parameters.append([0.1, height, 0.1, alpha, 0, resolution])
	return parameters

def generateSDF(filename, grid_size, padding):
	convert(filename + '.stl', filename + '.obj')
	call(["./SDFGen", str(filename) + '.obj', str(grid_size), str(padding)])

def collectData():
	shapes = ['cube', 'ellipse', 'cylinder', 'cone']
	eng = matlab.engine.start_matlab()
	eng.cd(r'/home/eadom/NearContactStudy/ShapeGenerator/',nargout=0)
	env = openravepy.Environment()
	robot = loadRobot(env)

	for shape in shapes:
		parameterShape = generateParameters(shape, 50)
		for parameters in parameterShape:
			print parameters
			item, filename = loadObject(env, eng, shape, parameters)
			transforms = sample_obj_transforms()
			for transform in transforms:
				print transform
				item.SetTransform(transform)
				meshUntransformed = mesh.Mesh.from_file(filename + '.stl')
				meshUntransformed.transform(transform)
				meshUntransformed.save(filename + '.stl', mode=stl.Mode.ASCII)
				generateSDF(filename, 0.001, 0)
				generateHandFeatures(filename + '.out', robot, item, filename)
				env.Save(filename + '.dae')
			env.Remove(env.GetBodies()[1])
