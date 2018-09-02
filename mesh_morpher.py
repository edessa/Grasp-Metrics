import vtk
import numpy
from vtk.util import numpy_support
import openravepy

def transformPoint(T, point):
	return numpy.matmul(T, (list(point) + [1]))

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
        triangles.append([[ax, ay, az], [bx, by, bz], [cx, cy, cz]])
    return numpy.array(triangles)

def getTris(robot, trianglesTransformed): #ShadowHand is 1, barrett is 0
    triangles = []
    vertices = []
    indices = []
    for i in range(0, len(robot.GetLinks())):
        link_geom = robot.GetLinks()[i].GetGeometries()
	if trianglesTransformed:
	    transforms = [numpy.eye(4), robot.GetTransform()]
	else:
	    transforms = [robot.GetLinks()[i].GetTransform(), robot.GetTransform()]
        for j in range(0, len(link_geom)):
            link_tris = link_geom[j].GetCollisionMesh()
	    vertices += link_tris.vertices.tolist()
	    indices += link_tris.indices.tolist()
            #    triangles = numpy.concatenate((triangles, verticesToTriangles(link_tris.vertices.tolist(), link_tris.indices.tolist(), transforms)), axis=0)
            #    triangles = verticesToTriangles(link_tris.vertices.tolist(), link_tris.indices.tolist(), transforms)
    return numpy.array(vertices), numpy.array(indices)


env = openravepy.Environment()
env.Load('/home/eadom/Grasp-Metrics/barretthand.robot.xml')
robot = env.GetBodies()[0]
vertices, indices = getTris(robot, 0)
data = numpy_support.numpy_to_vtk(num_array=vertices, deep=True)

b = vtk.vtkPolyData()
c = vtk.vtkPoints()
c.SetData(data)
b.SetPoints(c)

cells = vtk.vtkCellArray()
cells.SetCells(len(indices), numpy_support.numpy_to_vtkIdTypeArray(indices), deep=1)
b.SetPolys(cells)
a = vtk.vtkCellLocator(b)
a.Update()
cellId = vtk.mutable(0)
subId = vtk.mutable(0)
dist = vtk.mutable(0.0)
closestPoint = numpy.array([0, 0, 0])
a.FindClosestPoint(numpy.array([1,2,3]), closestPoint, cellId, subId, dist)
