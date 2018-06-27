import openravepy
from openravepy import *
import numpy
import math
import time
import sys
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

def getVerts(surface):
    link_geom = surface.GetGeometries()[0]
    link_tris = link_geom.GetCollisionMesh()
    verts = link_tris.vertices.tolist()
    return verts

def surfaces():
    return [robot.GetLinks()[7], robot.GetLinks()[6],  robot.GetLinks()[12],  robot.GetLinks()[13],  robot.GetLinks()[14], item.GetLinks()[0]]

def getRobotVerts(robot):
	verts = []
	for i in range(0, len(robot.GetLinks())):
		verts += getVerts(robot.GetLinks()[i])
	return verts

def getDistance(point, triangle): #Barrycentric coordinates are 0.5,0.5,0.5 so we can just compute distance between center of triangle and point
	center = [(triangle[0] + triangle[3] + triangle[6])/3, (triangle[1] + triangle[4] + triangle(7))/3, (triangle[2] + triangle[5] + triangle[8])/3]
	distance = math.sqrt((point[0]-center[0])^2 + (point[1] - center[1])^2 + (point[2] - center[2])^2)
	return distance

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

def getBarryPoints(handVerts, linkVert):#call this for each point, or if loading multiple points through linkVerts
	distances = norms_barry_triangles = []						 #then change return to append
	handTriangles = numpy.reshape(handVerts, (-1,9))
	point = linkVert #single point sampled in txt file, for palm, finger, etc
	for i in range(0, len(linkVerts)):
		distances = []   #list of distances between point specified and entire hand
		for j in range(0, len(handTriangles)):
			distances.append(getDistance(point, handTriangles[j])) #list of distances
		minDistance = min(distances)
		minDistanceIndex = distances.index(minDistance)
		triangleBary = handTriangles[minDistanceIndex]
		barry_coords = getBarryCoordinates(point, triangleBary)
		surface_norm = numpy.linalg.norm(getSurfaceNormal(triangleBary))
		norms_barry_triangles.append(surface_norm, barry_coords, triangleBary)
	return norms_barry_triangles

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

def computeNormalGrid(grid, extent, step):
	normal_grid = numpy.array((extent[1],extent[3],extent[5])) - numpy.array((extent[0],extent[2],extent[4])) + 1
	xshape = numpy.array((extent[1],extent[3],extent[5])) - numpy.array((extent[0],extent[2],extent[4])) + 1
	xshape_rev = xshape[-1::-1]
	x = numpy.zeros(xshape_rev,numpy.float32)
	for zi in range(xshape[2]):
	    for yi in range(xshape[1]):
	        for xi in range(xshape[0]):
				center = grid[z,y,x]
				zUp = grid[z+1, y, x]
				zDown = grid[z-1,y, x]
				yUp = grid[z, y+1, x]
				yDown = grid[z,y-1, x]
				xUp = grid[z, y, x+1]
				xDown = grid[z,y, x-1]

env = openravepy.Environment()
robot = loadRobot(env)
item = loadObject(env)

bounding_item = bounding_box(item)
field, bounds, extent, spacing = processVTI()

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
