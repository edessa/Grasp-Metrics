import openravepy
from openravepy import *
import numpy
import math
import time
import sys
import ast
import vtk


def processVTI():
	filename = '/home/eadom/NearContactStudy/InterpolateGrasps/models/stl_files/SprayBottle.vti'
	r = vtk.vtkXMLImageDataReader()
	r.SetFileName(filename)
	r.Update()
	data = r.GetOutput()
	dim = numpy.asarray(data.GetDimensions())
	dim = [dim[0]/1000.0, dim[1]/1000.0, dim[2]/1000.0]
	spacing = data.GetSpacing()
	spacing = [spacing[0]/1000.0, spacing[1]/1000.0, spacing[2]/1000.0]
	#x = numpy.zeros(data.GetNumberOfPoints())
	#y = x.copy()
	#z = x.copy()
	#Can do data.SetOrigin(data.GetOrigin() blah blah blah)
	bounds = data.GetBounds()
	bounds = numpy.array(bounds)/1000

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
		    x[zi,yi,xi] = a.GetValue(idx) / 1000.0
	return x, bounds, extent, spacing

def bounding_box(item):
	obj = item.ComputeAABB()
	max_xyz =  obj.pos()+obj.extents()
	min_xyz = obj.pos()-obj.extents()
	return min_xyz, max_xyz

def loadRobot(env):
	env.Load('/home/eadom/openrave/src/robots/barretthand.robot.xml')
	return env.GetRobots()[0]

def loadObject(env):
    env.Load('/home/eadom/NearContactStudy/InterpolateGrasps/models/stl_files/SprayBottle.STL', {'scalegeometry':'0.001'})
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
	print len(handVerts)
	for i in range(0, len(linkVerts)):
		distances = centers = normals = []
		point = linkVerts[i]
		distances = []   #list of distances between point specified and entire hand
		for j in range(0, len(handVerts)):
			distance, center = getDistance(point, handVerts[j][0:3])
			distances.append(distance) #list of distances
			centers.append(center)
			normals.append(handVerts[j][3:6])
		minDistance = min(list(distances))
		print minDistance
		minDistanceIndex = distances.index(minDistance)
		minCenter = centers[minDistanceIndex]
		minNormal = normals[minDistanceIndex]
		print minCenter
		print point
		print minNormal
		print '---'
	#	triangleBary = handTriangles[minDistanceIndex]
		#barry_coord = getBarryCoordinates(point, triangleBary)
		#surface_norm = (getSurfaceNormal(triangleBary)) / numpy.array(numpy.linalg.norm((getSurfaceNormal(triangleBary)))).astype(float)
		surface_norms.append(minNormal)
		#barry_coords.append(barry_coord)
		#triangleBarys.append(triangleBary)
		centerRet.append(minCenter)
	return surface_norms, centerRet

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

def centerItem(item, bounds):
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
def getPlane(point, normal, radiusX, radiusY, N): #equation of plane is a*x+b*y+c*z+d=0  [a,b,c] is the normal. Thus, we have to calculate d and we're set
	d = -1 * numpy.dot(point, normal)
	min = normal.index(min(normal))
	n1 = normal[(min+1)%3]
	n2 = normal[(min+2)%3]
	u = ([0, n1, n2])/numpy.linalg.norm([0,n1,n2]) #This is the vector inside the plane, perp. to the normal vector
	v = numpy.cross(u, normal)

	points_in_plane = []

	deltaX = radiusX / N
	epsilonX = deltaX * 0.5
	deltaY = radiusY / N
	epsilonY = deltaY * 0.5

	for y in range(-radiusY, radiusY+epsilonY, deltaY): #Epsilon makes sure point count is symmetric and we don't miss points on extremes
		for x in range(-radiusX, radiusX+epsilonX, deltaX):
			if x*x + y*y < radiusX*radiusY:
				points_in_plane.append(point + x*u + y*v)

	return points_in_plane



env = openravepy.Environment()
robot = loadRobot(env)
item = loadObject(env)

robot_hand_verts = getRobotVerts(robot)
hand_points = getManuallyLabelledPoints()



point_verts = []
for i in range(0, len(hand_points)):
	point_verts.append(ast.literal_eval(hand_points[hand_points.keys()[i]]))

min = 1000
for i in range(0, len(robot_hand_verts)):#
	dist = math.sqrt((robot_hand_verts[i][0]-point_verts[0][0])**2 + (robot_hand_verts[i][1] - point_verts[0][1])**2 + (robot_hand_verts[i][2] - point_verts[0][2])**2)#
	if dist < min:#
		min = dist

surface_norms, centerRet = getBarryPoints(robot_hand_verts, point_verts)

#print centerRet
#print point_verts
points_in_plane = getPlane(centerRet[0], surface_norms[0], 0.01, 0.01, 20) #Need to make this plane projection in the frame of whatever link we have (add link parameter, transform points)

bounding_item = bounding_box(item)
field, bounds, extent, spacing = processVTI()
gx, gy, gz = numpy.gradient(field)

lower_bound, upper_bound = centerItem(item, bounds)

point = [1,1,1] #This is the point that is used for distance to mesh calculation

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
point_to_grid_distance = math.sqrt((point[0] - xGridCoord)**2 + (point[1] - yGridCoord)**2 + (point[2] - zGridCoord)**2)
point_to_mesh_distance = math.sqrt(signed_distance_function_distance**2 + point_to_grid_distance**2)

#x, y, z = offset_SDF(x, y, z, offset1)

#and_coords = getBarryPoints(getRobotVerts(), getVerts(robot.GetLinks()[1])




env.SetViewer('qtcoin')
