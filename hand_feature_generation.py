import openravepy
import numpy
import math
import time
import ast
import pylab
import vtk_process
import get_skew_data



def loadRobot(env):
	env.Load('/home/eadom/Grasp-Metrics/barretthand.robot.xml')
	return env.GetRobots()[0]

def loadObject(env):
    env.Load('/home/eadom/Grasp-Metrics/SprayBottle.STL', {'scalegeometry':'0.001'})
    return env.GetBodies()[1]

def transform(points, localT):
	for i in range(0, len(points)):
		points[i][0] += localT[0][3]
		points[i][1] += localT[1][3]
		points[i][2] += localT[2][3]
	return points

def getVerts(surface):
    link_geom = surface.GetGeometries()
    verts = []
    for i in range(0, len(link_geom)):
	info = link_geom[i].GetInfo()
	localT = info._t
    	link_tris = link_geom[i].GetCollisionMesh()
    	verts += transform(link_tris.vertices.tolist(), localT)
    return verts

#def getVerts(surface):
#	return surface.GetCollisionData().vertices.tolist()

def surfaces():
    return [robot.GetLinks()[7], robot.GetLinks()[6],  robot.GetLinks()[12],  robot.GetLinks()[13],  robot.GetLinks()[14], item.GetLinks()[0]]

#def getRobotVerts(robot):
#	verts = []
#	for i in range(0, len(robot.GetLinks())):
#		print robot.GetLinks()[i]
#		print len(getVerts(robot.GetLinks()[i]))
#		verts += getVerts(robot.GetLinks()[i])
#	print len(verts)
#	return verts

def getRobotVerts(robot):
	b = numpy.loadtxt('RobotHand.out', dtype=float)
	return list(numpy.reshape(b, (-1,6)))

def getDistance(point, triangle): #Barrycentric coordinates are 0.5,0.5,0.5 so we can just compute distance between center of triangle and point
	#center = [(triangle[0] + triangle[3] + triangle[6])/3, (triangle[1] + triangle[4] + triangle[7])/3, (triangle[2] + triangle[5] + triangle[8])/3]
	center = [triangle[0], triangle[1], triangle[2]]
	distance = math.sqrt((point[0]-center[0])**2 + (point[1] - center[1])**2 + (point[2] - center[2])**2)
	return distance, center

def getBarryCoordinates(point, triangle):
	point1 = triangle[0:3]
	point2 = triangle[3:6]
	point3 = triangle[6:9]
	v0 = point2 - point1
	v1 = point3 - point1
	v2 = point - point1
	d00 = numpy.dot(v0, v0)
	d01 = numpy.dot(v0, v1)
	d11 = numpy.dot(v1, v1)
	d20 = numpy.dot(v2, v0)
	d21 = numpy.dot(v2, v1)
	denom = d00 * d11 - d01 * d01
	B1 = (d11 * d20 - d01 * d21) / denom
	B2 = (d00 * d21 - d01 * d20) / denom
	B3 = 1.0 - B1 - B2
	return [B1, B2, B3]

def getSurfaceNormal(triangle):
	point1 = triangle[0:3]
	point2 = triangle[3:6]
	point3 = triangle[6:9]
	U = point2 - point1
	V = point3 - point1
	Nx = U[1]*V[2] - U[2]*V[1]
	Ny = U[2]*V[0] - U[0]*V[2]
	Nz = U[0]*V[1] - U[1]*V[0]
	return [Nx, Ny, Nz]

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

def offset_SDF(x,y,z,offset):
	for i in range(0, len(x)):
		x[i] -= offset[0]
		y[i] -= offset[1]
		z[i] -= offset[2]
	return x,y,z

def offsetTransform(item, offset):
	T = item.GetTransform()
	T[0][3] -= offset[0]
	T[1][3] -= offset[1]
	T[2][3] -= offset[2]
	item.SetTransform(T)

def centerItem(item, bounds, bounding_item):
	lower_bound = [bounds[0], bounds[2], bounds[4]] #bounds of bounding box/signed distance field (min)
	upper_bound = [bounds[1], bounds[3], bounds[5]] #bounds of bounding box/signed distance field (max)
	offset1 = bounding_item[0] - lower_bound
	offset2 = bounding_item[1] - upper_bound

	offset = (offset1 + offset2) / 2.0
	offsetTransform(item, offset)
	return lower_bound, upper_bound

def getManuallyLabelledPoints():
	hand_points = {'handbase': '[0.00432584, 0.000212848, 0.09496]', 'Finger2-1': '[-0.0894092, 0.00154199, 0.1048]' , 'Finger2-2': '[-0.132307, 0.003609,  0.117]', 'Finger1-1': '[0.0882787, 0.0254203, 0.104]', 'Finger1-2':'[0.128, 0.025, 0.115]', 'Finger0-1' : '[0.0889559, -0.0254208,  0.104]', 'Finger0-2' : '[0.128, -0.0246, 0.116]'}
	return hand_points

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

	for y in pylab.frange(-radiusY, radiusY+epsilonY, deltaY): #Epsilon makes sure point count is symmetric and we don't miss points on extremes
		for x in pylab.frange(-radiusX, radiusX+epsilonX, deltaX):
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
		Tlocal = robot.GetLink(hand_links[i]).GetTransform()
		point_transformed = list(transformPoint(Tlocal, centerRet[i])[0:3])
		#print surface_norms[i][0:3]
		normal_transformed = list(transformNormal(Tlocal, surface_norms[i])[0:3])
		if hand_links[i] == 'handbase':
			points = getPlane(point_transformed, normal_transformed, 0.04,0.013, 1, 1)
		else:
			points = getPlane(point_transformed, normal_transformed, 0.03, 0.006, 1, 1)
		#print robot.GetLink(hand_links[i])
		link_points = []
		for j in range(0, len(points)):
			point_hand_frame = [points[j][0], points[j][1], points[j][2]]
			#print transformPoint(numpy.linalg.inv(Tlocal), point_hand_frame)[0:3]
			link_points.append([transformPoint(numpy.linalg.inv(Tlocal), point_hand_frame)[0:3], transformNormal(numpy.linalg.inv(Tlocal), normal_transformed)[0:3]])
		all_hand_points.append(link_points)

	#print len(all_hand_points)
	return all_hand_points


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

def generateHandFeatures(filename, robot, item):
	robot_hand_verts = getRobotVerts(robot)
	hand_points = getManuallyLabelledPoints()

	point_verts = []
	for i in range(0, len(hand_points)):
		point_verts.append(ast.literal_eval(hand_points[hand_points.keys()[i]]))

	surface_norms, centerRet = getBarryPoints(robot_hand_verts, point_verts)

	points_in_hand_plane = getGridOnHand(robot, hand_points.keys(), centerRet, surface_norms)
	bounding_item = bounding_box(item)
	field, bounds, extent, spacing = vtk_process.processVTI()
	gx, gy, gz = numpy.gradient(field)
	lower_bound, upper_bound = centerItem(item, bounds, bounding_item)

	distances = [[]]

	for i in range(0, len(points_in_hand_plane)):
		distance_link = []
		for j in range(0, len(points_in_hand_plane[i])):
			point = points_in_hand_plane[i][j][0]
			normal = points_in_hand_plane[i][j][1]
			index1 = extent[1] * (point[0] - lower_bound[0])/(upper_bound[0] - lower_bound[0])
			index2 = extent[3] * (point[1] - lower_bound[1])/(upper_bound[1] - lower_bound[1])
			index3 = extent[5] * (point[2] - lower_bound[2])/(upper_bound[2] - lower_bound[2])

			if index1 > extent[1]:
				index1 = extent[1]
			if index1 < 0:
				index1 = 0
			if index2 > extent[3]:
				index2 = extent[3]
			if index2 < 0:
				index2 = 0
			if index3 > extent[5]:
				index3 = extent[5]
			if index3 < 0:
				index3 = 0

			xGridCoord = lower_bound[0] + spacing[0] * index1
			yGridCoord = lower_bound[1] + spacing[1] * index2
			zGridCoord = lower_bound[2] + spacing[2] * index3

			signed_distance_function_distance = field[index3, index2, index1]
			normal_grid_vector = [gz[index3][index2][index1], gy[index3][index2][index1], gx[index3][index2][index1]] #Ex: gz is really field[x,y,z+1]-field[x,y,z-1]/ 2*step
			point_to_grid_distance = math.sqrt((point[0] - xGridCoord)**2 + (point[1] - yGridCoord)**2 + (point[2] - zGridCoord)**2)
			point_to_mesh_distance = math.sqrt(signed_distance_function_distance**2 + point_to_grid_distance**2)
			angle_between_norms = angle_between(normal, normal_grid_vector)
			distance_link.append([point_to_mesh_distance, angle_between_norms])

		distances.append(distance_link)
	numpy.savetxt(filename, distances, delimiter=',', fmt='%s')




env = openravepy.Environment()
robot = loadRobot(env)
item = loadObject(env)
#generateHandFeatures('Robot.out', robot, item)


#x, y, z = offset_SDF(x, y, z, offset1)

#and_coords = getBarryPoints(getRobotVerts(), getVerts(robot.GetLinks()[1])


#env.SetViewer('qtcoin')
