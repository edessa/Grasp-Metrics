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
import os

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

def getRPYRotations(nR, nY, nP):
	rotations = []
	for roll in pylab.frange(-0.4, 0.4, 0.8/nR):
		for pitch in pylab.frange(-0.4, 0.4, 0.8/nP):
			for yaw in pylab.frange(-0.4, 0.4, 0.8/nY):
				print roll, pitch, yaw
				rotations.append(rotation_matrix_from_rpy(roll, pitch, yaw))
	return rotations

def getTranslations(Nx, sizeX, Nz, sizeZ):
	points = []
	for x in pylab.frange(-sizeX/2.0, sizeX/2.0, sizeX/float(Nx)):
		for z in pylab.frange(0, sizeZ, sizeZ/float(Nz)):
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

def sample_obj_transforms(env, item):
	transforms = []
	translations = getTranslations(5, .1, 3, .05)

	for trans in translations:
		uniformly_sampled_rotations = getRPYRotations(3, 3, 3)
		for R in uniformly_sampled_rotations:
			T_obj = numpy.eye(4)
			T_obj[0:3,0:3] = R
			item.SetTransform(T_obj)
			while env.CheckCollision(item):
				T_obj = adjustTransform(env, item, T_obj)
			T_obj[0][3] = trans[0]
			T_obj[1][3] = trans[1]
			T_obj[2][3] += trans[2] #depth offset
			transforms.append(T_obj)
	return transforms

#def generateParameters(type, N):
#	parameters = []
#	shapeResolutions = {'cube':10, 'ellipse':100, 'cylinder':25, 'cone':25, 'vase':25}
#	resolution = shapeResolutions[type]
#	if type == 'cone':
#		n = int(N**(1./2.))
#		for width in pylab.frange(0.05, 0.15, 0.1/n):
#			for height in pylab.frange(0.05, 0.15, 0.1/n):
#				parameters.append([width, height, 0.1, 10, 0, resolution])
#	else:
#		n = int(N**(1./2.))
#		for height in pylab.frange(0.05, 0.15, 0.1/n):
#			for alpha in pylab.frange(10, 40, 30/n):
#				parameters.append([0.1, height, 0.1, alpha, 0, resolution])
#	return parameters

def generateParameters(type):
	shapeResolutions = {'cube':10, 'ellipse':100, 'cylinder':25, 'cone':25, 'vase':25}
	resolution = shapeResolutions[type]
	parameters = []
	if type == 'ellipse':
		parameters.append([0.1, 0.1, 0.1, 0.1, 0, resolution])
		parameters.append([0.2, 0.2, 0.2, 0.1, 0, resolution])
		parameters.append([0.2, 0.2, 0.3, 0.1, 0, resolution])
	if type == 'cone':
		parameters.append([0.07, 0.07, 0.07, 30, 0, resolution])
		parameters.append([0.15, 0.15, 0.15, 30, 0, resolution])
		parameters.append([0.2, 0.2, 0.2, 30, 0, resolution])
	if type == 'cube':
		parameters.append([0.05, 0.05, 0.05, 0.05, 0, resolution])
		parameters.append([0.13, 0.13, 0.13, 0.1, 0, resolution])
		parameters.append([0.13, 0.13, 0.2, 0.1, 0, resolution])
	if type == 'cylinder':
		parameters.append([0.1, 0.2, 0.1, 0.05, 0, resolution])
		parameters.append([0.2, 0.2, 0.2, 0.05, 0, resolution])
	return parameters


def generateSDF(filename, grid_size, padding):
	convert('./Shapes/' + filename + '.stl', './Shapes/' + filename + '.obj')
##	if os.path.isfile('./Shapes/' + str(filename) + '.vti'):
#		os.remove('./Shapes/' + str(filename) + '.vti')
#		time.sleep(0.4)
	file = '/home/eadom/Grasp-Metrics/Shapes/' + str(filename) + '.obj'
	print file
	call(["./SDFGen", file, str(grid_size), str(padding)])

def STLtoPLY(filename):
	reader = vtk.vtkSTLReader()
	writer = vtk.vtkPLYWriter()
	reader.SetFileName('./Shapes/' + filename + '.stl')
	writer.SetFileName('./Shapes/' + filename + '.ply')
	writer.SetInputConnection(reader.GetOutputPort())
	writer.Write()

def createPointCloud(filename):
	STLtoPLY(filename)
	filePly = './Shapes/' + filename
	command = "python3 point_sampler.py " + filePly + " obj"
	print command
	call(command.split()) #need to use ply file for point_sampler.py, find a way to save obj/stl as ply
	pcd = numpy.loadtxt('./Shapes/' + filename + '.out', dtype=float)
	pcd = list(numpy.reshape(pcd, (-1,3)))
	return pcd

def adjustTransform(env, item, transform):
	while env.CheckCollision(item):
		transform[2][3] += 0.01
		item.SetTransform(transform)
	transform[2][3] += 0.02
	return transform


def collectData():
	directory = './' + str(int(time.time())) + '/'
	os.mkdir(directory, 0755)
	shapes = ['cube', 'ellipse', 'cylinder', 'cone']
	eng = matlab.engine.start_matlab()
	eng.cd(r'/home/eadom/NearContactStudy/ShapeGenerator/',nargout=0)
	env = openravepy.Environment()
	env.SetViewer('qtcoin')
	robot = loadRobot(env)
	iter = 0
	viewer = env.GetViewer()
	for shape in shapes:
		parameterShape = generateParameters(shape)
		for parameters in parameterShape:
			item, filename = loadObject(env, eng, shape, parameters)
			item.SetVisible(0)
			pointCloud = createPointCloud(filename)
			print filename
			generateSDF(filename, 0.005, 70)
			transforms = sample_obj_transforms(env, item)
			for transform in transforms:
				viewer.SetTitle("Iteration" + str(iter))
				item.SetTransform(transform)
				generateHandFeatures(env, directory + filename + str(iter), robot, item, filename, transform, pointCloud)
				env.Save(directory + filename + '_' + str(iter) + '.dae')
				iter += 1
			env.Remove(env.GetBodies()[1])
