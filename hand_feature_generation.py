import openravepy
import numpy
import math
import time
import ast
import pylab
import get_data
import vtk
import matlab.engine
from pathlib import Path
import itertools
from scipy import spatial
from graspit_commander.graspit_exceptions import InvalidRobotPoseException

handles = []

def generateGeometry(eng, type, parameters):
	print parameters
	print type
	filename = eng.generateShapes(type, [float(parameters[0]), float(parameters[1]), float(parameters[2]), float(parameters[3]), float(parameters[4]), float(parameters[5])], nargout=1) #width, height, extent, alpha, min_d, resolution
	return filename

def loadRobot(env):
	env.Load('./barretthand.robot.xml')
	return env.GetRobots()[0]

def loadObject(env, eng, obj_type, parameters):
    filename = generateGeometry(eng, obj_type, parameters)
    env.Load(filename + '.stl')
    return env.GetBodies()[1], Path(filename).name

def transform(points, localT):
	for i in range(0, len(points)):
		points[i][0] += localT[0][3]
		points[i][1] += localT[1][3]
		points[i][2] += localT[2][3]
	return points


def getTransformBetweenLinks(link1, link2):
    trns1 = link1.GetTransform()
    trns2 = link2.GetTransform()
    return numpy.dot(numpy.linalg.inv(trns2),trns1)

def getVerts(surface):
    link_geom = surface.GetGeometries()
    verts = []
    for i in range(0, len(link_geom)):
	info = link_geom[i].GetInfo()
	localT = info._t
    	link_tris = link_geom[i].GetCollisionMesh()
    	verts += transform(link_tris.vertices.tolist(), localT)
    return verts

def surfaces():
    return [robot.GetLinks()[7], robot.GetLinks()[6],  robot.GetLinks()[12],  robot.GetLinks()[13],  robot.GetLinks()[14], item.GetLinks()[0]]

def getRobotVerts(robot):
	b = numpy.loadtxt('./RobotHand.out', dtype=float)
	b = list(numpy.reshape(b, (-1,6)))
	transformed_points = []
	for point in b:
		transformed_pt = transformPoint(robot.GetTransform(), point[0:3])
		transformed_nm = transformNormal(robot.GetTransform(), point[3:6])
		transformed_points.append([transformed_pt[0], transformed_pt[1], transformed_pt[2], transformed_nm[0], transformed_nm[1], transformed_nm[2]])
	return transformed_points

def getDistance(point, triangle): #Barrycentric coordinates are 0.5,0.5,0.5 so we can just compute distance between center of triangle and point
	#center = [(triangle[0] + triangle[3] + triangle[6])/3, (triangle[1] + triangle[4] + triangle[7])/3, (triangle[2] + triangle[5] + triangle[8])/3]
	center = [triangle[0], triangle[1], triangle[2]]
	distance = math.sqrt((point[0]-center[0])**2 + (point[1] - center[1])**2 + (point[2] - center[2])**2)
	return distance, center

def centerItem(sdfCenter, itemCenter):
	T = numpy.eye(4)
	T[0][3] = sdfCenter[0]
	T[1][3] = sdfCenter[1]
	T[2][3] = sdfCenter[2]
	return T

def getManuallyLabelledPoints():
	hand_points = {'handbase': '[0.0, 0.0, 0.09496]', 'Finger2-1': '[-0.0894092, 0.00154199, 0.1048]' , 'Finger2-2': '[-0.132307, 0.003609,  0.117]', 'Finger1-1': '[0.0882787, 0.0254203, 0.104]', 'Finger1-2':'[0.128, 0.025, 0.115]', 'Finger0-1' : '[0.0889559, -0.0254208,  0.104]', 'Finger0-2' : '[0.128, -0.0246, 0.116]'}
	return hand_points

def processVTI(obj_name):
	filename = './Shapes/' + obj_name + '.vti'
	r = vtk.vtkXMLImageDataReader()

	r.SetFileName(filename)
	r.Update()
	data = r.GetOutput()

	dim = numpy.asarray(data.GetDimensions())
	dim = [dim[0], dim[1], dim[2]]
	spacing = data.GetSpacing()
	spacing = [spacing[0], spacing[1], spacing[2]]
	center = data.GetCenter()
	#x = numpy.zeros(data.GetNumberOfPoints())
	#y = x.copy()
	#z = x.copy()
	#Can do data.SetOrigin(data.GetOrigin() blah blah blah)
	bounds = data.GetBounds()
	bounds = numpy.array(bounds)
	extent = data.GetExtent()
	pd = data.GetPointData()
	a = pd.GetArray(0)
	xshape = numpy.array((extent[1],extent[3],extent[5])) - numpy.array((extent[0],extent[2],extent[4])) + 1
	xshape_rev = xshape[-1::-1]
	x = numpy.zeros(xshape_rev,numpy.float32)
	for zi in range(xshape[2]):
	    for yi in range(xshape[1]):
	        for xi in range(xshape[0]):
	            idx = zi*xshape[0]*xshape[1] + yi*xshape[0] + xi
		    x[zi,yi,xi] = a.GetValue(idx)
	return x, bounds, extent, spacing, center


#N is the number of points in 1 direction
def getPlane(point, normal, radiusX, radiusY, Nx, Ny): #equation of plane is a*x+b*y+c*z+d=0  [a,b,c] is the normal. Thus, we have to calculate d and we're set
	d = -1 * numpy.dot(point, normal)
	minIndex = normal.index(min(normal))
	n1 = -normal[(minIndex+1)%3]
	n2 = normal[(minIndex+2)%3]
	#u = ([0, n2, n1])/numpy.linalg.norm([0,n1,n2]) #This is the vector inside the plane, perp. to the normal vector
	if normal[0] == 0 and normal[1] == 0:
		u = [0, 1, 0]
	else:
		u = [-normal[1], normal[0], 0]
	v = numpy.cross(u, normal)

	u = numpy.array(u) / numpy.linalg.norm(u)
	v = numpy.array(v) / numpy.linalg.norm(v)

	points_in_plane = []

	deltaX = radiusX / Nx
	epsilonX = deltaX * 0.5
	deltaY = radiusY / Ny
	epsilonY = deltaY * 0.5

	for y in pylab.frange(-radiusY, radiusY, deltaY): #Epsilon makes sure point count is symmetric and we don't miss points on extremes
		for x in pylab.frange(-radiusX, radiusX, deltaX):
			points_in_plane.append(point + x*u + y*v)

	return points_in_plane

def transformPoint(T, point):
	return numpy.matmul(T, (list(point) + [1]))

def transformNormal(T, normal):
	return numpy.matmul(numpy.transpose(numpy.linalg.inv(T)), list(normal) + [0])

def getGridOnHand(robot, hand_links, centerRet, surface_norms):
	all_hand_points = []
	#print centerRet
	for i in range(0, len(hand_links)):
		Tlocal = numpy.linalg.inv(robot.GetLink(hand_links[i]).GetTransform())
		point_transformed = list(transformPoint(Tlocal, centerRet[i])[0:3])
		#print surface_norms[i][0:3]
		normal_transformed = list(transformNormal(Tlocal, surface_norms[i])[0:3])

		if hand_links[i] == 'handbase':
			points = getPlane(point_transformed, normal_transformed, 0.02,0.0065, 1, 1)
		else:
			points = getPlane(point_transformed, normal_transformed, 0.01, 0.003, 1, 1)
		#print robot.GetLink(hand_links[i])
		link_points = []
		for j in range(0, len(points)):
			point_hand_frame = [points[j][0], points[j][1], points[j][2]]
			#print transformPoint(numpy.linalg.inv(Tlocal), point_hand_frame)[0:3]
			link_points.append([transformPoint(numpy.linalg.inv(Tlocal), point_hand_frame)[0:3], transformNormal(numpy.linalg.inv(Tlocal), normal_transformed)[0:3]])
		all_hand_points.append(link_points)

	#print len(all_hand_points)
	return all_hand_points

def getBarryPoints(handVerts, linkVerts):#call this for each point, or if loading multiple points through linkVerts
	norms_barry_triangles = surface_norms = barry_coords = triangleBarys = centerRet  = []						 #then change return to append
	#print len(handVerts)
	for i in range(0, len(linkVerts)):
		distances = centers = normals = []
		point = linkVerts[i]
		distances = []   #list of distances between point specified and entire hand
		minDistance = minCenter = minNormal = 1000
		for j in range(0, len(handVerts)):
			hand_point = handVerts[j][0:3]

			distance, center = getDistance(point, hand_point)
			if distance < minDistance:
				minDistance = distance
				minCenter = hand_point
				minNormal = handVerts[j][3:6]
			#	print minCenter
		centerRet = numpy.concatenate((centerRet, minCenter))
		surface_norms = numpy.concatenate((surface_norms, minNormal))
		#print point
		#print minNormal
		#print '---'
	#	triangleBary = handTriangles[minDistanceIndex]
		#barry_coord = getBarryCoordinates(point, triangleBary)
		#surface_norm = (getSurfaceNormal(triangleBary)) / numpy.array(numpy.linalg.norm((getSurfaceNormal(triangleBary)))).astype(float)
		#barry_coords.append(barry_coord)
		#triangleBarys.append(triangleBary)
	surface_norms = list(numpy.reshape(surface_norms, (-1,3)))
	centerRet = list(numpy.reshape(centerRet, (-1,3)))
	return list(surface_norms), list(centerRet)

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / numpy.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return numpy.arccos(numpy.clip(numpy.dot(v1_u, v2_u), -1.0, 1.0))

def bounding_box(item):
	obj = item.ComputeAABB()
	return obj.pos()

def interPolateNeighbors(field, index1, index2, index3):
	deltaCell = [0, 1]
	neighbors = [p for p in itertools.product(deltaCell, repeat=3)]
	distance = 0

	for neighbor in neighbors:
		distance += field[index3 + neighbor[2]][index2 + neighbor[1]][index1 + neighbor[0]]
	return distance / 8.0 #8 neighbors in grid

def transformPointCloud(transform, pointCloudObj):
	transformedPointCloud = []
	for point in pointCloudObj:
		transformedPoint = transformPoint(transform, point)[0:3]
		transformedPointCloud.append(transformedPoint)
	return transformedPointCloud

def plotPoints(env, grid):
    handles = []
    pointGrid = []
    for i in range(0, len(grid)):
	    for j in range(0, len(grid[0])):
		    pointGrid.append(grid[i][j][0])

    handles.append(env.plot3(points=numpy.array(pointGrid), pointsize=0.001, colors=numpy.array(((0,1,0))), drawstyle=1))

    #for i in range(0, len(obj_pts)):
    #    handles.append(env.drawarrow(p1=pointGrid[i], p2 = obj_pts[i], linewidth=0.001, color=[1.0, 0.0, 0.0]))

    return handles

def plotObject(env, obj_pts):
	handles.append(env.plot3(points=numpy.array(obj_pts), pointsize=0.0005, colors=numpy.array(((1,0,0))), drawstyle=1))

def getColl(env, links):
	for link in links:
		if env.CheckCollision(link):
			return True
	return False

def alignmentJoints(env, robot, item):
	a = [0] * 4
	aInd = numpy.arange(0, 4, 1)
	leftSideColl = rightSideColl = False
	iterContactLeft = iterContactRight = 0
	for i in range(0, 100):
		if leftSideColl == False:
			a[2] += 0.01
			robot.SetDOFValues(a, aInd)
			leftSideColl = getColl(env, [robot.GetLinks()[8]])
			if not leftSideColl:
				iterContactLeft = i
		if rightSideColl == False:
			a[0] += 0.01
			a[1] += 0.01
			robot.SetDOFValues(a, aInd)
			rightSideColl = getColl(env, [robot.GetLinks()[6]] + [robot.GetLinks()[3]])
			if not rightSideColl:
				iterContactRight = i
		if leftSideColl == True and rightSideColl == True:
			break

	return iterContactLeft, iterContactRight

def alignmentDistance(env, robot, item):
	a = [0] * 4
	aInd = numpy.arange(0, 4, 1)
	leftSideColl = rightSideColl = False
	iterContactLeft = iterContactRight = 0
	iter = 0

	while a[0] < 2 and a[1] < 2 and a[2] < 2:
		iter += 1
		fing1RDist = fing2RDist = fing1LDist = 0
		curFing1R = robot.GetLinks()[6].GetTransform()[0:3, 3:4]
		curFing2R = robot.GetLinks()[3].GetTransform()[0:3, 3:4]
		curFing1L = robot.GetLinks()[8].GetTransform()[0:3, 3:4]


		if rightSideColl == False:
			while fing1RDist < 0.005:
				a[0] += 0.0001
				robot.SetDOFValues(a, aInd)
				fing1RDist = getDistance(curFing1R, robot.GetLinks()[6].GetTransform()[0:3, 3:4])
			if getColl(env, [robot.GetLinks()[6]]):
				rightSideColl = True
				iterContactRight = iter

			while fing2RDist < 0.005:
				a[1] += 0.0001
				robot.SetDOFValues(a, aInd)
				fing2RDist = getDistance(curFing2R, robot.GetLinks()[3].GetTransform()[0:3, 3:4])
			if getColl(env, [robot.GetLinks()[3]]):
				rightSideColl = True
				iterContactRight = iter

		if leftSideColl == False:
			while fing1LDist < 0.005:
				a[2] += 0.0001
				robot.SetDOFValues(a, aInd)
				fing1LDist = getDistance(curFing1L, robot.GetLinks()[8].GetTransform()[0:3, 3:4])
			if getColl(env, [robot.GetLinks()[8]]):
				leftSideColl = True
				iterContactLeft = iter

		if leftSideColl == True and rightSideColl == True:
			break

	return iterContactLeft, iterContactRight

def generateHandFeatures(env, gc, filename, robot, item, item_name, transformObj, pointCloudObj, points_in_hand_plane):
	global handles
	transformedPointCloud = transformPointCloud(transformObj, pointCloudObj)
	print transformObj
	tree = spatial.cKDTree(transformedPointCloud)
	handles = plotPoints(env, points_in_hand_plane)

	#surface_norms, centerRet = getBarryPoints(robot_hand_verts, point_verts)

#	print centerRet
#	print surface_norms
#	print hand_points.keys()

#	print points_in_hand_plane
	center = bounding_box(item)
	field, bounds, extent, spacing, sdfCenter = processVTI(item_name)
	lower_bound = [bounds[0], bounds[2], bounds[4]]
	upper_bound = [bounds[1], bounds[3], bounds[5]]

	#print bounds
	gx, gy, gz = numpy.gradient(field)
	#print "sdfCenter"
	#rint sdfCenter
	#print "centerItem"
	#print center
	offset_transform = centerItem(sdfCenter, center)
	#print offset_transform

	distances = [[]]
	errors = []
	obj_pts = []

	for i in range(0, len(points_in_hand_plane)):
		distance_link = []
		for j in range(0, len(points_in_hand_plane[i])):
			point = points_in_hand_plane[i][j][0]
			normal = points_in_hand_plane[i][j][1]
			#point = [0,0, 0.2]
			point_world_to_obj = transformPoint(numpy.linalg.inv(transformObj), point)[0:3]
			point_transformed = transformPoint(offset_transform, point_world_to_obj)[0:3]

			index1 = int(extent[1] * (point_transformed[0] - lower_bound[0])/(upper_bound[0] - lower_bound[0]))
			index2 = int(extent[3] * (point_transformed[1] - lower_bound[1])/(upper_bound[1] - lower_bound[1]))
			index3 = int(extent[5] * (point_transformed[2] - lower_bound[2])/(upper_bound[2] - lower_bound[2]))

	#		print
			if index1 > extent[1]:
				print "failed"
				index1 = extent[1]
			if index1 < 0:
				print "failed"
				index1 = 0
			if index2 > extent[3]:
				print "failed"
				index2 = extent[3]
			if index2 < 0:
				print "failed"
				index2 = 0
			if index3 > extent[5]:
				print "failed"
				index3 = extent[5]
			if index3 < 0:
				print "failed"
				index3 = 0

			ground_truth_pt = tree.query(point)[0]
	#		print "debugging distance"
	#		print ground_truth_pt
	#		print "debugging point"
			debug_pt = tree.data[tree.query(point)[1]]
			obj_pts.append(debug_pt)

	#		print "field distance"
	#		print field[index3][index2][index1]
			signed_distance_function_distance = math.fabs(interPolateNeighbors(field, index1, index2, index3))
	#		print signed_distance_function_distance
			errors.append(ground_truth_pt - signed_distance_function_distance)

			normal_grid_vector = [gz[index3][index2][index1], gy[index3][index2][index1], gx[index3][index2][index1]] #Ex: gz is really field[x,y,z+1]-field[x,y,z-1]/ 2*step
			angle_between_norms = angle_between(normal, normal_grid_vector)
			distance_link.append([signed_distance_function_distance, angle_between_norms])

		distances.append(distance_link)


	#plotObject(env, transformedPointCloud)


	print "Saved grasp metric matrice"
	numpy.savetxt(filename + '.out', distances, delimiter=',', fmt='%s')
	numpy.savetxt(filename + '_error' + '.out', errors, delimiter=',', fmt='%s')

	#alignment_label_joints = alignmentJoints(env, robot, item)
	#robot.SetDOFValues([0]*4, [0, 1, 2, 3])
	#alignment_label_dist = alignmentDistance(env, robot, item)
	#robot.SetDOFValues([0]*4, [0, 1, 2, 3])

	gc.autoGrasp()
	try:
		result = gc.computeQuality()
		volume = [result.volume]
		epsilon = [result.epsilon]
	except InvalidRobotPoseException:
		volume = [0]
		epsilon = [0]

	dofs = [0, 0, 0, 0]
	gc.forceRobotDof(dofs)
	gc.autoOpen()
	numpy.savetxt(filename + '_labels' + '.out', volume + epsilon, delimiter=',', fmt='%s')





#x, y, z = offset_SDF(x, y, z, offset1)

#and_coords = getBarryPoints(getRobotVerts(), getVerts(robot.GetLinks()[1])


#env.SetViewer('qtcoin')
