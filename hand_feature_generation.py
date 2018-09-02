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
from decimal import Decimal
import fpectl
from numpy.core.umath_tests import inner1d
import inspect
from CGAL.CGAL_Kernel import Point_3
from CGAL.CGAL_Kernel import Triangle_3
from CGAL.CGAL_Kernel import Ray_3
from CGAL.CGAL_AABB_tree import AABB_tree_Triangle_3_soup

handles = []

def generateGeometry(eng, type, parameters):
	print parameters
	print type
	filename = eng.generateShapes(type, [float(parameters[0]), float(parameters[1]), float(parameters[2]), float(parameters[3]), float(parameters[4]), float(parameters[5])], nargout=1) #width, height, extent, alpha, min_d, resolution
	return filename

def loadRobot(env, handType):
	if handType == 'Barrett':
		env.Load('./barretthand.robot.xml')
		T = numpy.eye(4)
	elif handType == 'pr2_gripper':
		env.Load('/home/eadom/Grasp-Metrics/pr2-gripper.stl')
		T = numpy.eye(4)
		T[0:3,0:3] = get_data.rotation_matrix_from_rpy(math.pi/2, -math.pi/2, 0)
	else:
		env.Load('./shadow-hand.dae')
		T = numpy.array([[0,0,-1,0],[1,0,0,-0.08],[0,-1,0,0],[0,0,0,1]])
	env.GetBodies()[0].SetTransform(T)
	return env.GetBodies()[0], T

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

def getManuallyLabelledPoints(handType):
	if handType == 'Barrett':
		hand_points = {'handbase': '[0.0, 0.0, 0.09496]', 'Finger2-1': '[-0.0894092, 0.00154199, 0.1048]' , 'Finger2-2': '[-0.132307, 0.003609,  0.117]', 'Finger1-1': '[0.0882787, 0.0254203, 0.104]', 'Finger1-2':'[0.128, 0.025, 0.115]', 'Finger0-1' : '[0.0889559, -0.0254208,  0.104]', 'Finger0-2' : '[0.128, -0.0246, 0.116]'}
	elif handType == 'ShadowHand':
		hand_points = {'palm': '[0.01036,0.04447,0.01607]', 'Finger0-2': '[-0.0199, 0.17334, 0.01229]', 'Finger0-1': '[-0.01986, 0.14325, 0.01448]', 'Finger0-0': '[-0.02045, 0.10678, 0.01476]', 'Finger2-1': '[0.02288, 0.15658, 0.01412]', 'Finger2-0': '[0.02184, 0.12047, 0.01482]', 'Finger2-2': '[0.02351, 0.18685, 0.01168]', 'Finger3-0': '[0.044, 0.1185, 0.01468]', 'Finger3-1': '[0.0449, 0.1529, 0.01392]', 'Finger3-2': '[0.04507, 0.18452, 0.01145]', 'Finger1-2': '[0, 0.18252, 0.01297]', 'Finger4-2': '[0.10283, 0.11211, 0.01335]', 'Finger4-1': '[0.077, 0.08721, 0.01332]', 'Finger1-0': '[0.0006, 0.11832, 0.01510]', 'Finger1-1': '[0.001, 0.15268, 0.01476]'}
	else:
		hand_points =  {'None':'0', 'None2':'0'}
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

def getRobotTriangles(robot):
	triangles = []
	for i in range(0, len(robot.GetLinks())):
	    triangles += getTris(robot.GetLinks()[i], robot.GetTransform())
	return verts

def verticesToTriangles(verts, triangleIndices, transforms):
    triangles = []
    for i in range(0, len(triangleIndices)):
        ax, ay, az = transformPoint(transforms[1], transformPoint(transforms[0], verts[triangleIndices[i][0]])[0:3])[0:3]
        bx, by, bz = transformPoint(transforms[1], transformPoint(transforms[0], verts[triangleIndices[i][1]])[0:3])[0:3]
        cx, cy, cz = transformPoint(transforms[1], transformPoint(transforms[0], verts[triangleIndices[i][2]])[0:3])[0:3]
        triangles.append(Triangle_3(Point_3(ax, ay, az), Point_3(bx, by, bz), Point_3(cx, cy, cz)))
    return numpy.array(triangles)

def getTris(robot, trianglesTransformed): #ShadowHand is 1, barrett is 0
    triangles = []
    print trianglesTransformed
    print robot.GetTransform()
    for i in range(0, len(robot.GetLinks())):
        link_geom = robot.GetLinks()[i].GetGeometries()
	if trianglesTransformed == 1:
		T = robot.GetTransform()
		T[0:3,3] = [0,0,0]
	    	transforms = [numpy.eye(4), T]
	elif trianglesTransformed == 0:
	    transforms = [robot.GetLinks()[i].GetTransform(), robot.GetTransform()]
	else:
		transforms = [numpy.eye(4), robot.GetTransform()]
        for j in range(0, len(link_geom)):
            link_tris = link_geom[j].GetCollisionMesh()
	    link_verts = link_tris.vertices.tolist()
	    for k in range(0, len(link_verts)):
	        link_verts[k] = transformPoint((link_geom[j].GetTransform()), link_verts[k])[0:3]
            if len(triangles) != 0:
                triangles = numpy.concatenate((triangles, verticesToTriangles(link_verts, link_tris.indices.tolist(), transforms)), axis=0)
            else:
                triangles = verticesToTriangles(link_verts, link_tris.indices.tolist(), transforms)
    if trianglesTransformed != 1:
	return triangles
    return triangles[3000:len(triangles)]

def getGridOnHand(robot, hand_links, centerRet, surface_norms):
	all_hand_points = []
	#print centerRet
	for i in range(0, len(hand_links)):
		link_points = []
		if 'None' not in hand_links:
			Tlocal = numpy.linalg.inv(robot.GetLink(hand_links[i]).GetTransform())
			point_transformed = list(transformPoint(Tlocal, centerRet[i])[0:3])
			normal_transformed = list(transformNormal(Tlocal, surface_norms[i])[0:3])

			points = getPlane(point_transformed, normal_transformed, 0.01, 0.005, 10, 10)
			#print robot.GetLink(hand_links[i])
			for j in range(0, len(points)):
				point_hand_frame = [points[j][0], points[j][1], points[j][2]]
				#print transformPoint(numpy.linalg.inv(Tlocal), point_hand_frame)[0:3]
				link_points.append([transformPoint(numpy.linalg.inv(Tlocal), point_hand_frame)[0:3], transformNormal(numpy.linalg.inv(Tlocal), normal_transformed)[0:3]])
		else:
			points = getPlane(list(centerRet[i]), list(surface_norms[i]), 0.01, 0.01, 10, 10)
			for j in range(0, len(points)):
				link_points.append([points[j][0:3], list(surface_norms[i])])
		all_hand_points.append(link_points)
	#print len(all_hand_points)
	return all_hand_points

def unit_vector(vector):
    return vector / numpy.linalg.norm(vector)

def angle_between(v1, v2):
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


def morphPoints(robot, grid, trianglesTransformed):
	tris = getTris(robot, trianglesTransformed)
	tree = AABB_tree_Triangle_3_soup(tris)
	print len(tris)
	print trianglesTransformed
	for i in range(0, len(grid)):
		for j in range(0, len(grid[i])):
			#if j != 98 and j != 119 and j != 120 and j != 140 and j != 141 and j != 161 and j != 162 and j != 163:
			if trianglesTransformed != 1 or ((i != 11 and i != 13) or (i == 11 and (j < 95 or j > 300)) or (i == 13 and j < 243)):
				pt = grid[i][j][0]
				point_query = Point_3(pt[0], pt[1], pt[2])
				point_morphed = tree.closest_point(point_query)
				if trianglesTransformed == 1:
					grid[i][j][0] = [point_morphed.x(), point_morphed.y()-0.08, point_morphed.z()]
				else:
					grid[i][j][0] = [point_morphed.x(), point_morphed.y(), point_morphed.z()]
			else:
				grid[i][j][0] = [grid[i][j][0][0], grid[i][j][0][1]-0.08, grid[i][j][0][2]]
	print "finished morphing"
	return grid

def generateHandFeatures(env, gc, filename, robot, item, item_name, transformObj, pointCloudObj, points_in_hand_plane, trianglesTransformed):
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







#x, y, z = offset_SDF(x, y, z, offset1)

#and_coords = getBarryPoints(getRobotVerts(), getVerts(robot.GetLinks()[1])


#env.SetViewer('qtcoin')
