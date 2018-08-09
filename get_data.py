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
from graspit_commander import GraspitCommander
from geometry_msgs.msg import Pose


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

def sample_obj_transforms(env, gc, item):
	transforms = []
	translations = getTranslations(5, .1, 3, .05)

	for trans in translations:
		uniformly_sampled_rotations = getRPYRotations(3, 3, 3)
		for R in uniformly_sampled_rotations:
			T_obj = numpy.eye(4)
			T_obj[0:3,0:3] = R
			T_obj[2][3] = 0.05
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
		#parameters.append([0.13, 0.13, 0.2, 0.1, 0, resolution])
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

def getPoseMsg(transform):
	robotPose = Pose()
	q = openravepy.quatFromRotationMatrix(transform[0:3,0:3])
	robotPose.position.x = transform[0][3]
	robotPose.position.y = transform[1][3]
	robotPose.position.z = transform[2][3]
	robotPose.orientation.x = q[0]
	robotPose.orientation.y = q[1]
	robotPose.orientation.z = q[2]
	robotPose.orientation.w = q[3]
	return robotPose

def getRobotPose(transform):
	robotPose = Pose()
	T_init = numpy.eye(4)
	T_init[0:3,0:3] = rotation_matrix_from_rpy(math.pi/2, math.pi, 0)
	T_init[2][3] = 0.0835
	return numpy.matmul(T_init, transform)

def generateGaussianNoise(configuration, num_uncertainties):
    mean = configuration
    #cov = numpy.diag(numpy.array([0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])/10.0)
    distr = numpy.random.normal(configuration, [0.005, 0.005, 0.005, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1], [num_uncertainties,10])
    return distr

def get_link_transforms(robot, links):
	transforms = []
	for link in links:
		transforms.append(robot.GetLink(link).GetTransform())
	return transforms

def transform_norms(original_link_transforms, noise_induced_link_transforms, surfaceNorms):
	transformed_norms = []
	for i in range(0, len(surfaceNorms)):
		transformed_norms.append(transformNormal(numpy.dot(numpy.linalg.inv(noise_induced_link_transforms[i]),original_link_transforms[i]), surfaceNorms[i])[0:3])
	return transformed_norms

def transform_centers(original_link_transforms, noise_induced_link_transforms, centerRet):
	transformed_pts = []
	for i in range(0, len(centerRet)):
		T = numpy.dot(numpy.linalg.inv(noise_induced_link_transforms[i]),original_link_transforms[i])
		print T
		transformed_pts.append(transformPoint(T, centerRet[i])[0:3])
	return transformed_pts

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
	gc = GraspitCommander()
	for shape in shapes:
		parameterShape = generateParameters(shape)
		for parameters in parameterShape:
			item, filename = loadObject(env, eng, shape, parameters)
			gc.clearWorld()
			gc.importRobot('Barrett')
			gc.importGraspableBody('/home/eadom/Grasp-Metrics/Shapes/' + filename + '.ply')
			robot_pose = getPoseMsg(getRobotPose(numpy.eye(4)))
			gc.setRobotPose(robot_pose)
			item.SetVisible(1)
			pointCloud = createPointCloud(filename)
			print filename
			generateSDF(filename, 0.005, 70)
			transforms = sample_obj_transforms(env, gc, item)
			graspit_transform = getRobotPose(transforms)
			for transform in transforms:
				configuration = [0]*10
				noise_distr = generateGaussianNoise(configuration, 150)
				noise_induced_transform = numpy.eye(4)
				hand_points = getManuallyLabelledPoints()
				robot.SetTransform(numpy.eye(4))
				robot.SetDOFValues([0,0,0,0], [0, 1, 2, 3])
				original_link_transforms = get_link_transforms(robot, hand_points.keys())
				centerRet = numpy.array([[0.128, 0.025, 0.115], [0.0882787, 0.0254203, 0.104], [0.0, 0.0, 0.09496], [0.128, -0.0246, 0.116], [0.0889559, -0.0254208,  0.104], [-0.0894092, 0.00154199, 0.1048],[-0.128, 0,  0.117]])
				surface_norms = numpy.array([[-0.707,  0.00799337, 0.707], [0, 0., 1], [0, 0, 1], [-0.707,  0.00799337, 0.707], [0, 0, 1], [0, 0, 1], [ 0.707, -0.00799337, 0.707]])
				for noise in noise_distr:
					viewer.SetTitle("Iteration" + str(iter))
					noise_induced_transform[0:3,3] = numpy.array(noise[0:3])
					noise_induced_transform[0:3,0:3] = rotation_matrix_from_rpy(noise[3:6][0], noise[3:6][1], noise[3:6][2])
					robot.SetTransform(noise_induced_transform)
					print "Noise transform:"
					print noise_induced_transform
					for i in range(6, 10):
						if noise[i] < 0:
							noise[i] = 0
					robot.SetDOFValues(noise[6:10], [0, 1, 2, 3])
					if not env.CheckCollision(robot):
						noise_induced_link_transforms = get_link_transforms(robot, hand_points.keys())
						item.SetTransform(transform)
						robot_pose = getPoseMsg(getRobotPose(noise_induced_transform))
						gc.setRobotPose(robot_pose)
						gc.setGraspableBodyPose(0, getPoseMsg(transform))
						point_verts = []
						for i in range(0, len(hand_points)):
							point_verts.append(ast.literal_eval(hand_points[hand_points.keys()[i]]))
						transformed_center_ret = transform_centers(numpy.linalg.inv(original_link_transforms), numpy.linalg.inv(noise_induced_link_transforms), centerRet)
						transformed_surface_norms = transform_norms(numpy.linalg.inv(original_link_transforms), numpy.linalg.inv(noise_induced_link_transforms),  surface_norms)
						points_in_hand_plane = getGridOnHand(robot, hand_points.keys(), transformed_center_ret, transformed_surface_norms)
						generateHandFeatures(env, gc, directory + filename + str(iter), robot, item, filename, transform, pointCloud, points_in_hand_plane)
						#env.Save(directory + filename + '_' + str(iter) + '.dae')
						iter += 1
			env.Remove(env.GetBodies()[1])
