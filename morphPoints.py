import hand_feature_generation
from hand_feature_generation import *


def transformTriangle(T_obj_world, triangles):
    transformedTriangles = []
    for triangle in triangles:
        triangle1 = triangles[0:3]
        triangle2 = triangles[3:6]
        triangle3 = triangles[6:9]
        triangle1Transformed = transformPoint(T_obj_world, triangle1)
        triangle2Transformed = transformPoint(T_obj_world, triangle2)
        triangle3Transformed = transformPoint(T_obj_world, triangle3)
        transformedTriangles.append(triangle1Transformed + triangle2Transformed + triangle3Transformed)
    return transformedTriangles

def getTriangles(robot):
    link_geom = surface.GetGeometries()
    verts = []
    for i in range(0, len(link_geom)):
        T_obj_world = numpy.linalg.inv(numpy.linalg.inv(robot.GetLinks()[i]))
        link_tris = link_geom[i].GetCollisionMesh()
        verts += transformTriangle(T_obj_world, triangles)
    return verts
