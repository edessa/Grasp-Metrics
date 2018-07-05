
import get_skew_data
from get_skew_data import *
import pylab

env = openravepy.Environment()
env.Load('/home/eadom/Grasp-Metrics/SprayBottle.STL', {'scalegeometry':'0.001'})
env.Load('/home/eadom/Grasp-Metrics/barretthand.robot.xml')
item = env.GetBodies()[0]
orig, extents = BoundingBox_param(item)
step = 10
a, faces = get_positions_wrt_obj(item, '', 0.13, 0, step)
robot = env.GetBodies()[1]
robot.SetTransform(a[0])

bound1 = [0, 0, 0]
bound2 = [1, 1, 1]


list1 = []
for i in pylab.range(bound1[0], bound2[0], 10):
    list1.append([i, bound1[1], bound1[2])

env.SetViewer('qtcoin')
I = env.GetViewer().GetCameraImage(640,480,env.GetViewer().GetCameraTransform(),[640,640,320,240])
