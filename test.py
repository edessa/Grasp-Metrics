
import get_skew_data
from get_skew_data import *
import pylab

import openravepy
import numpy
import math
import time
import matlab.engine


def getTransformBetweenLinks(trns1, trns2):
    return numpy.dot(numpy.linalg.inv(trns2),trns1)

def sampleCone(obj_transform, hand_transform, sample_rate, radiusA, radiusB, angle): #T_reference_source
    transforms = []
    roll_transform = rotation_matrix_from_rpy(-angle, 0, 0)
    hand_transform = numpy.dot(hand_transform, roll_transform)
    T_between_init = getTransformBetweenLinks(hand_transform, obj_transform) #T_obj_hand
    for theta in range(0, sample_rate):
        x = radiusA * math.cos(theta * math.pi/sample_rate)
        y = radiusB * math.sin(theta * math.pi/sample_rate)
        T_R = rotation_matrix_from_rpy(0,theta * math.pi/sample_rate,0) #rotation to apply to hand
        T_rotation = numpy.eye(4)
        T_rotation[0:3,0:3] = T_R
        T_rotation[0][3] = x
        T_rotation[2][3] = y
        T_between = numpy.dot(T_between_init, T_rotation) #T_obj_hand
        T_world_hand = numpy.dot(T_obj, T_between) #T_world_hand = T_world_obj * T_obj_hand
        transforms.append(T_world_hand)
    return transforms

class PlotSpinner(threading.Thread):
    def __init__(self,handle):
        threading.Thread.__init__(self)
        self.starttime = time.time()
        self.handle=handle
        self.ok = True
    def run(self):
        while self.ok:
            self.handle.SetShow(bool(mod(time.time()-self.starttime,2.0) < 1.0))

validgrasp = numpy.array([ 0.00000000e+00,  0.00000000e+00,  1.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  1.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -1.00000000e+00,  1.22464680e-16,
        0.00000000e+00, -1.22464680e-16, -1.00000000e+00, -1.22499985e-02,
       -1.22499985e-02, -4.27600034e-02,  1.72805325e+09,  6.12323400e-17,
        1.00000000e+00,  0.00000000e+00,  0.00000000e+00, -1.22499985e-02,
       -1.22499985e-02, -4.27600034e-02,  1.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -1.00000000e+00,  1.22464680e-16,
        0.00000000e+00, -1.22464680e-16, -1.00000000e+00, -1.22499985e-02,
       -1.22499985e-02, -4.28500034e-02,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -5.46753360e-04,  0.00000000e+00,
        0.00000000e+00, -1.00000000e+00,  1.62500003e+00,  2.44346095e+00,
        1.85200003e+00,  0.00000000e+00,  1.00000000e+00,  1.00000000e+00,
        1.00000000e+00,  0.00000000e+00,  0.00000000e+00,  4.14257300e-03,
       -1.22499985e-02, -1.22499985e-02,  2.22499987e-02,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00])
eng = matlab.engine.start_matlab()
eng.cd(r'/home/eadom/NearContactStudy/ShapeGenerator/',nargout=0)
parameters = [0.1, 0.050000000000000003, 0.1, 10, 0, 10]
type = 'cube'
ret = eng.generateShapes(type, parameters, nargout=1)

import openravepy
import numpy
import threading
import time
T = numpy.array([[ 0.98232732, -0.14927452,  0.11291656, -0.05], [ 0.1317587,   0.97997794,  0.14927452,  0.  ], [-0.13293862, -0.1317587, 0.98232732,  0.05 ], [ 0. , 0.  ,0.  ,1. ]])

env = openravepy.Environment()
env.Load('/home/eadom/Grasp-Metrics/Shapes/cube_h5_w5_e5.stl')
env.Load('/home/eadom/Grasp-Metrics/barretthand.robot.xml')

item = env.GetBodies()[0]
robot = env.GetBodies()[1]
gmodel = openravepy.databases.grasping.GraspingModel(robot,item)
contacts,finalconfig,mindist,volume = gmodel.testGrasp(grasp=validgrasp,translate=True,forceclosure=False)
point = numpy.array([0.02, 0.02, 0.3])
handles = []
item = env.GetBodies()[0]
T = numpy.eye(4)
T[2][3] = 0.2
item.SetTransform(T)
robot.SetDOFValues([1.3,1.3,1.3,0],[0,1,2,3])
env.SetViewer('qtcoin')
handles.append(env.drawarrow(p1=[0,0,0], p2 = [1,1,1], linewidth=0.01, color=[1.0, 0.0, 0.0]))
spinner = PlotSpinner(handles[-1])
spinner.start()
env.plot3(points=point, pointsize=15.0)


def plotPoints(grid):
    handles = []
    handles.append(env.plot3(points=point, pointsize=0.001, colors=numpy.array(((0,1,0))), drawstyle=1))
    return handles


robot = env.GetBodies()[1]
item = env.GetBodies()[0]
T = numpy.eye(4)
T[2][3] = 0.2
item.SetTransform(T)

filename = '/home/eadom/Grasp-Metrics/Shapes/cube_h5_w10_e10.vti'
r = vtk.vtkXMLImageDataReader()
r.SetFileName(filename)
r.Update()
data = r.GetOutput()
data.GetDataDimension()
data.ComputeBounds()
data.GetExtent()
data.GetSpacing()

T_graspit_transform = T_openrave + 0.0835 in z-direction



#rotation = [math.pi/5, 0, 0]
#T_rotation = openravepy.matrixFromAxisAngle(rotation)
T_R = rotation_matrix_from_rpy(0,0.4,0)
T_rotation = numpy.eye(4)
T_rotation[0:3,0:3] = T_R

T_between = numpy.dot(T_between, T_rotation)
T_world_hand = numpy.dot(T_obj, T_between)
robot.SetTransform(T_world_hand)


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


 graspdof = {'igraspdir':3,'igrasppos':3,'igrasproll':1,'igraspstandoff':1,'igrasppreshape':len(self.manip.GetGripperIndices()),'igrasptrans':12,'imanipulatordirection':3,
 'forceclosure':1,'grasptrans_nocol':12,'performance':1,'vintersectplane':4, 'igraspfinalfingers':len(self.manip.GetGripperIndices()), 'ichuckingdirection':len(self.manip.GetGripperIndices()), 'graspikparam_nocol':8, 'approachdirectionmanip':3, 'igrasptranslationoffset':3 }

igraspdir = validgrasp[0:3]
igrasppos = validgrasp[3:6]
igrasproll = validgrasp[6:7]
igraspstandoff = validgrasp[7:8]
igrasppreshape = validgrasp[8:12]
igrasptrans = validgrasp[12:24]
imanipulatordirection = validgrasp[24:27]
forceclosure = validgrasp[27:28]
grasptrans_nocol = validgrasp[28:40]
performance = validgrasp[40:41]
vintersectplane = validgrasp[41:45]
igraspfinalfingers = validgrasp[45:49]
ichuckingdirection = validgrasp[49:53]
graspikparam_nocol = validgrasp[53:61]
approachdirectionmanip = validgrasp[61:64]
igrasptranslationoffset = validgrasp[64:67]
reconstructed = numpy.concatenate((igraspdir, igrasppos, igrasproll, igraspstandoff, igrasppreshape, igrasptrans, imanipulatordirection, forceclosure, grasptrans_nocol, performance, vintersectplane, igraspfinalfingers, ichuckingdirection, graspikparam_nocol, approachdirectionmanip, igrasptranslationoffset))
