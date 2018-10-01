import openravepy
import numpy
from scipy import spatial
import math
from hand_feature_generation import *
import random
from subprocess import call
from stl2obj import convert
import os
from graspit_commander import GraspitCommander
from graspit_commander.graspit_exceptions import *
from geometry_msgs.msg import Pose
import pdb
import time

#Creates a uniformly sampled set of quaternions
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


#Iterates over roll, pitch and yaw rotations of target object
def getRPYRotations(nR, nY, nP):
	rotations = []
	for roll in pylab.frange(-0.4, 0.4, 0.8/nR):
		for pitch in pylab.frange(-0.4, 0.4, 0.8/nP):
			for yaw in pylab.frange(-0.4, 0.4, 0.8/nY):
				rotations.append(rotation_matrix_from_rpy(roll, pitch, yaw))
	return rotations

#Iterates over translations of target objecct
def getTranslations(Nx, sizeX, Ny, sizeY, Nz, sizeZ):
	points = []
	for x in pylab.frange(-sizeX/2.0, sizeX/2.0, sizeX/float(Nx)):
		for y in pylab.frange(-sizeY/2.0, sizeY/2.0, sizeY+0.000001/float(Ny)):
			for z in pylab.frange(0, sizeZ, sizeZ+0.0000001/float(Nz)):
				points.append([x, y, z])
	return points

def bounding_box(item):
	obj = item.ComputeAABB()
	max_xyz =  obj.pos()+obj.extents()
	min_xyz = obj.pos()-obj.extents()
	return min_xyz, max_xyz

#Creates KD-tree out of pointcloud
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

#Converts roll, pitch and yaw to quaternion
def quat_from_rpy(roll, pitch, yaw):
    cr, cp, cy = math.cos(roll / 2), math.cos(pitch / 2), math.cos(yaw / 2)
    sr, sp, sy = math.sin(roll / 2), math.sin(pitch / 2), math.sin(yaw / 2)
    return numpy.array([
        cr * cp * cy + sr * sp * sy,
        -cr * sp * sy + cp * cy * sr,
        cr * cy * sp + sr * cp * sy,
        cr * cp * sy - sr * cy * sp])

#Roll, pitch and yaw rotation to rotation matrix
def rotation_matrix_from_rpy(roll, pitch, yaw):
	return openravepy.rotationMatrixFromQuat(quat_from_rpy(roll, pitch, yaw))

#Creates combination of object translations and rotations for experiments.
def sample_obj_transforms(env, gc, item, handType):
	transforms = []
	if handType == 'pr2_gripper': #Change object transforms based on hand type chosen
		translations = getTranslations(5, .05, 1, 0, 3, .05)
	if handType == 'ShadowHand':
		translations = getTranslations(5, .03, 3, 0.05, 1, 0)
	if handType == 'Barrett':
		translations = getTranslations(5, .1, 1, 0, 3, .05)
	print translations
	for trans in translations:
		uniformly_sampled_rotations = getRPYRotations(3, 3, 3)
		for R in uniformly_sampled_rotations:
			T_obj = numpy.eye(4)
			T_obj[0:3,0:3] = R
			T_obj[0][3] = trans[0]
			T_obj[1][3] = trans[1]
			if handType == 'Barrett':
				T_obj[2][3] = trans[2] + 0.1
			elif handType == 'pr2_gripper':
				T_obj[2][3] = trans[2] + 0.06
			else:
				T_obj[2][3] = trans[2] + 0.04
			item.SetTransform(T_obj)
			while env.CheckCollision(item):
				T_obj = adjustTransform(env, item, T_obj)
			#T_obj[2][3] += trans[2] #depth offset
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

#def generateParameters(type):
#	shapeResolutions = {'cube':10, 'ellipse':100, 'cylinder':25, 'cone':25, 'vase':25}
#	resolution = shapeResolutions[type]
#	parameters = []
#	if type == 'ellipse':
#		parameters.append([0.05, 0.05, 0.05, 0.05, 0, resolution])
#		parameters.append([0.1, 0.1, 0.1, 0.1, 0, resolution])
#		#parameters.append([0.2, 0.2, 0.3, 0.1, 0, resolution])
#	if type == 'cone':
#		parameters.append([0.06, 0.06, 0.06, 30, 0, resolution])
#		parameters.append([0.1, 0.1, 0.1, 30, 0, resolution])
#		#parameters.append([0.2, 0.2, 0.2, 30, 0, resolution])
#	if type == 'cube':
#		parameters.append([0.05, 0.05, 0.05, 0.05, 0, resolution])
#		parameters.append([0.09, 0.09, 0.09, 0.09, 0, resolution])
#		#parameters.append([0.13, 0.13, 0.2, 0.1, 0, resolution])
#	if type == 'cylinder':
#		parameters.append([0.06, 0.2, 0.06, 0.05, 0, resolution])
#		parameters.append([0.1, 0.2, 0.1, 0.05, 0, resolution])
#	return parameters

def generateParameters(type): #Calls shape generating code, just creates a list of parametesr based on the target shape
	shapeResolutions = {'cube':10, 'ellipse':100, 'cylinder':25, 'cone':25, 'vase':25}
	resolution = shapeResolutions[type]
	parameters = []
	if type == 'ellipse':
		parameters.append([0.0775, 0.0775, 0.0775, 0.0775, 0, resolution])
	#	parameters.append([0.2, 0.2, 0.2, 0.1, 0, resolution])
		#parameters.append([0.2, 0.2, 0.3, 0.1, 0, resolution])
	if type == 'cone':
		parameters.append([0.08, 0.08, 0.08, 27, 0, resolution])
		parameters.append([0.12, 0.12, 0.12, 27, 0, resolution])
		#parameters.append([0.2, 0.2, 0.2, 30, 0, resolution])
	if type == 'cube':
		parameters.append([0.059, 0.059, 0.059, 0.059, 0, resolution])
		parameters.append([0.08, 0.08, 0.08, 0.08, 0, resolution])
		#parameters.append([0.13, 0.13, 0.2, 0.1, 0, resolution])
	if type == 'cylinder':
		parameters.append([0.06, 0.08, 0.06, 0.06, 0, resolution])
		parameters.append([0.08, 0.097, 0.08, 0.008, 0, resolution])
	return parameters

def generateSDF(filename, grid_size, padding): #Creates a signed distance field out of object mesh
	convert('./Shapes/' + filename + '.stl', './Shapes/' + filename + '.obj')
##	if os.path.isfile('./Shapes/' + str(filename) + '.vti'):
#		os.remove('./Shapes/' + str(filename) + '.vti')
#		time.sleep(0.4)
	file = '/home/irongiant/Grasp-Metrics/Shapes/' + str(filename) + '.obj'
	print file
	call(["./SDFGen", file, str(grid_size), str(padding)])

def STLtoPLY(filename): #Converts STL to PLY
	reader = vtk.vtkSTLReader()
	writer = vtk.vtkPLYWriter()
	reader.SetFileName('./Shapes/' + filename + '.stl')
	writer.SetFileName('./Shapes/' + filename + '.ply')
	writer.SetInputConnection(reader.GetOutputPort())
	writer.Write()

def createPointCloud(filename): #Create point cloud out of mesh (only used for debugging)
	STLtoPLY(filename)
	filePly = './Shapes/' + filename
	command = "python3 point_sampler.py " + filePly + " obj"
	print command
	call(command.split()) #need to use ply file for point_sampler.py, find a way to save obj/stl as ply
	pcd = numpy.loadtxt('./Shapes/' + filename + '.out', dtype=float)
	pcd = list(numpy.reshape(pcd, (-1,3)))
	return pcd

def adjustTransform(env, item, transform): #Used to create collision-free object transforms
	item.SetTransform(transform)
	while env.CheckCollision(item):
		transform[2][3] += 0.01
		item.SetTransform(transform)
	return transform

def getPoseMsg(transform): #Convert transform into ROS poseStamped message
	robotPose = Pose()
	q = openravepy.quatFromRotationMatrix(transform[0:3,0:3])
	robotPose.position.x = transform[0][3]
	robotPose.position.y = transform[1][3]
	robotPose.position.z = transform[2][3]
	robotPose.orientation.x = q[1]
	robotPose.orientation.y = q[2]
	robotPose.orientation.z = q[3]
	robotPose.orientation.w = q[0]
	return robotPose

def getRobotPose(transform, handType): #Creates the tranfsorm of the hand for loading into GraspIt!
	robotPose = Pose()
	if handType == 'Barrett':
		T_init = numpy.eye(4)
	#	T_init[0:3,0:3] = rotation_matrix_from_rpy(math.pi/2, math.pi, 0)
		T_init[2][3] = 0.0835
	elif handType == 'ShadowHand':
		T_init = numpy.eye(4)
		T_init[0:3,0:3] = rotation_matrix_from_rpy(0,3.14,0)
		T_init[0:3,0:3] = numpy.matmul(T_init[0:3,0:3], rotation_matrix_from_rpy(3.14,0,0))
		#T_init[0][3] -= 0.05
		T_init[2][3] = 0.0025
		T_offset = numpy.eye(4)
		T_offset[0][3] = -0.08
		T_init = numpy.matmul(T_init, T_offset)
	else:
		T_init = numpy.eye(4)
		T_init[0:3,0:3] = rotation_matrix_from_rpy(0,-math.pi/2,0)
		T_offset = numpy.eye(4)
		#T_offset[0][3] = 0.025
		T_init = numpy.matmul(T_init, T_offset)
	return numpy.matmul(T_init, transform)

def generateGaussianNoise(handType, configuration, noise_std, num_uncertainties):
	mean = configuration
#cov = numpy.diag(numpy.array([0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])/10.0)
#[0.005, 0.005, 0.005, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
	distr = numpy.random.normal(configuration, noise_std, [num_uncertainties, len(noise_std)])
	return distr

def getHandPoints(handType): #Seed points for grid generation (point location and normal)
	if handType == 'Barrett':
		centerRet = numpy.array([[0.128, 0.025, 0.115], [0.0882787, 0.0254203, 0.104], [0.0, 0.0, 0.09496], [0.128, -0.0246, 0.116], [0.0889559, -0.0254208,  0.104], [-0.0894092, 0.00154199, 0.1048],[-0.128, 0,  0.117]])
		surface_norms = numpy.array([[-0.707,  0.00799337, 0.707], [0, 0., 1], [0, 0, 1], [-0.707,  0.00799337, 0.707], [0, 0, 1], [0, 0, 1], [ 0.707, -0.00799337, 0.707]])
	elif handType == 'ShadowHand':
		centerRet = numpy.array([[0.044, 0.1185, 0.01468], [0.0449, 0.1529, 0.01392], [0.04507, 0.18452, 0.01145], [0, 0.18252, 0.01297], [0.10283, 0.11211, 0.01335], [0.077, 0.08721, 0.01332], [0.0006, 0.11832, 0.01510], [0.001, 0.15268, 0.01476], [0.01036,0.04447,0.01607], [-0.0199, 0.17334, 0.01229], [-0.01986, 0.14325, 0.01448], [-0.02045, 0.10678, 0.01476], [0.02288, 0.15658, 0.01412], [0.02184, 0.12047, 0.01482], [0.02351, 0.18685, 0.01168]])
		surface_norms = numpy.array([[0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1], [-0.55, 0.83, 0], [-0.55, 0.83, 0], [0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1]])
	else:
		centerRet = numpy.array([[0.04590, 0, 0.16547], [-0.04590, 0, 0.16547]])
		surface_norms = numpy.array([[-1, 0, 0], [1, 0, 0]])
	return centerRet, surface_norms

def collectData(handType, num): #Workhorse of this file, handType is the chosen hand, num is the type of shape
	directory = './' + str(handType) + '_' + str(int(time.time())) + '/' #Directory for saving metrics and labels
	os.mkdir(directory, 0755)
	shapes = ['cube', 'ellipse', 'cylinder', 'cone']
	shapes = [shapes[num % 4]]
	eng = matlab.engine.start_matlab()
	eng.cd(r'/home/irongiant/NearContactStudy/ShapeGenerator/',nargout=0)
	env = openravepy.Environment()
	collisionChecker = openravepy.RaveCreateCollisionChecker(env,'ode')
	env.SetCollisionChecker(collisionChecker)
	#env.SetViewer('qtcoin') #Set viewer (optional)
	robot, init_transform = loadRobot(env, handType) #load robot into Openrave and get transform of the hand in OpenRAVE
	iter = 0
	#viewer = env.GetViewer()
	gc = GraspitCommander()
	GraspitCommander.GRASPIT_NODE_NAME = "/graspit" + str(num+1) + "/"
	hand_points = getManuallyLabelledPoints(handType)
	centerRet, surface_norms = getHandPoints(handType)
	if handType == 'Barrett': #The 'trianglesTransformed' variable is a hack used to handle the
		trianglesTransformed = 0 # inconsistencies between the meshes of the hands. I'll explain more in a writeup
	elif handType == 'pr2_gripper':
		trianglesTransformed = 2
	else:
		trianglesTransformed = 1
	points_in_hand_plane = getGridOnHand(robot, hand_points.keys(), centerRet, surface_norms) #Generates the points and normals of the grids on the hand
	points_in_hand_plane = morphPoints(robot, points_in_hand_plane, trianglesTransformed) #Morphs the points from the grid plane onto the mesh of the hand
	if handType == 'pr2_gripper':
		numJoints = 0
	else:
		numJoints = len(robot.GetActiveDOFValues())
	T_rotation = numpy.eye(4)
	T_rotation[0:3,0:3] = rotation_matrix_from_rpy(0, 0, math.pi/2)
	for shape in shapes: #Only iterates over 1 shape, see line 264
		parameterShape = generateParameters(shape)
		for parameters in parameterShape:
			item, filename = loadObject(env, eng, shape, parameters)
			STLtoPLY(filename)
			gc.clearWorld()
			gc.importRobot(handType)
			gc.importGraspableBody('/home/irongiant/Grasp-Metrics/Shapes/' + filename + '.ply')
			robot_pose = getPoseMsg(getRobotPose(numpy.eye(4), handType))
			gc.setRobotPose(robot_pose)
			gc.setGraspableBodyPose(0, getPoseMsg(numpy.matmul(T_rotation, numpy.eye(4))))
			item.SetVisible(1) #Can make item visible or invisible for viewing
			print filename
			generateSDF(filename, 0.0025, 70) #Signed distance field needs grid resolution parameters
			transforms = sample_obj_transforms(env, gc, item, handType)
			graspit_transform = getRobotPose(transforms, handType) #Get the tarnsform for loading the hand into graspit
			for transform in transforms:
				item.SetTransform(transform)
				#transform[0:3,0:3] = numpy.matmul(transform[0:3,0:3], rotation_matrix_from_rpy(math.pi/2, math.pi, 0))
				gc.setGraspableBodyPose(0, getPoseMsg(numpy.matmul(T_rotation, transform)))
				print transform
				print getPoseMsg(transform)
				generateHandFeatures(env, gc, directory + filename + str(iter), robot, item, filename, transform, points_in_hand_plane, trianglesTransformed, math.pi/6, 5) #Saves signed distance metric and wedge metric
						#env.Save(directory + filename + '_' + str(iter) + '.dae')
				configuration = [0]*(6 + numJoints)
				noise_std = [0.005, 0.005, 0.005, 0.1, 0.1, 0.1] + [0.1] * numJoints  #Covariance matrix parameters for gaussian noise injection
				noise_distr = generateGaussianNoise(handType, configuration, noise_std, 100) #Samples noise from multivariate gaussian
		#		volume = epsilon = []
				epsilon = []
				contacts = []

				for noise in noise_distr:
					noise_induced_transform = numpy.eye(4)
					noise_induced_transform[0:3,3] = numpy.array(noise[0:3])
					noise_induced_transform[0:3,0:3] = rotation_matrix_from_rpy(noise[3:6][0], noise[3:6][1], noise[3:6][2])
					#if handType != "pr2_gripper":
					#	robot.SetTransform(numpy.matmul(init_transform, noise_induced_transform))
					#else:
					robot.SetTransform(numpy.matmul(noise_induced_transform, init_transform)) #Inject noise into transform of hand
					for i in range(6, 6 + numJoints):
						if noise[i] < 0:
							noise[i] = 0
					if handType != "pr2_gripper":
						robot.SetDOFValues(noise[6:6 + numJoints], numpy.arange(numJoints)) #Inject noise into joint angles of hand
					collExcept = 0
					try:
						robot_pose = getPoseMsg(getRobotPose(noise_induced_transform, handType))
						gc.setRobotPose(robot_pose) #Sets teh same transform set in OpenRAVE into GraspIt!
						result = gc.computeQuality() #Computes grasp quality
					except (RobotCollisionException, InvalidRobotPoseException):
						collExcept = 1
					if not env.CheckCollision(robot) and collExcept == 0:
						if handType == "Barrett":
							gc.forceRobotDof(noise[6: 6 + numJoints])
						if handType == 'ShadowHand':
							gc.approachToContact()
						gc.autoGrasp()
						g_contacts = []
						try:
							r = gc.getRobot(0)
							contact = r.robot.contacts
							for c in contact:
								point = c.ps.pose.position
								pose = c.ps.pose.orientation
								g_contacts.append([point.x, point.y, point.z, pose.x, pose.y, pose.z, pose.w])
							result = gc.computeQuality()
						#	volume += [result.volume]
							epsilon += [result.epsilon] #List of grasp qualities as a result of noise
							contacts.append(g_contacts)  # list of contacts made between hand and object during grasp as a result of noise
						except InvalidRobotPoseException:
						#	volume += [0]
							epsilon += [0]
						if handType != 'ShadowHand':
							dofs = [0] * numJoints
						else:
							dofs = [0] * 17
						if handType != 'pr2_gripper':
							gc.forceRobotDof(dofs)
						gc.autoOpen()
				robot.SetTransform(init_transform)
				robot_pose = getPoseMsg(getRobotPose(numpy.eye(4), handType))
				gc.setRobotPose(robot_pose)
				if handType != "pr2_gripper":
					robot.SetDOFValues([0]*len(robot.GetActiveDOFValues()), numpy.arange(0, len(robot.GetActiveDOFValues())))
				numpy.savetxt(directory + filename + str(iter) + '_epsi_labels' + '.out', epsilon, delimiter=',', fmt='%s') #Saves the grasp qualities
				numpy.savetxt(directory + filename + str(iter) + '_cont_labels' + '.out', contacts, delimiter=',', fmt='%s') #Saves the contacts made

				iter += 1

			env.Remove(env.GetBodies()[1])
