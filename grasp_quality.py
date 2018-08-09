cd graspit_ros_ws
source devel/setup.bash
roslaunch graspit_interface graspit_interface.roslaunch

import numpy
from graspit_commander import GraspitCommander
from geometry_msgs.msg import Pose
gc = GraspitCommander()
gc.importRobot('Barrett')
gc.importGraspableBody('/home/eadom/Grasp-Metrics/Shapes/cube_h5_w5_e5.ply')
r = gc.getRobot(0)
dofs = list(r.robot.dofs)
q = [0,0,0,1]

T = numpy.eye(4)
T[2][3] = 0.023
objPose = Pose()
objPose.position.x = T[0][3]
objPose.position.y = T[1][3]
objPose.position.z = T[2][3]
objPose.orientation.x = q[0]
objPose.orientation.y = q[1]
objPose.orientation.z = q[2]
objPose.orientation.w = q[3]
T = numpy.eye(4)

robotPose = Pose()
robotPose.position.x = T[0][3]
robotPose.position.y = T[1][3]
robotPose.position.z = 0.05
robotPose.orientation.x = q[0]
robotPose.orientation.y = q[1]
robotPose.orientation.z = q[2]
robotPose.orientation.w = q[3]

gc.setGraspableBodyPose(0, objPose)
gc.setRobotPose(robotPose)
dofs = [0, 3, 3, 3]
gc.moveDOFToContacts(dofs, [0.01,0.01,0.01,0.01], True)
gc.forceRobotDof(dofs)
graspQuality = gc.computeQuality()

gc.approachToContact()
gc.moveDOFToContacts(dofs, [0.01,0.01,0.01,0.01], True)
gc.setRobotDOFForces([0, 100000000, 100000000, 100000000])
gc.autoGrasp(0)
graspQuality = gc.computeQuality()
gc.autoOpen()


Openrave Transform:
>>> T
array([[1.   , 0.   , 0.   , 0.   ],
       [0.   , 1.   , 0.   , 0.   ],
       [0.   , 0.   , 1.   , 0.117],
       [0.   , 0.   , 0.   , 1.   ]])

Graspit Transform:

openrave apply 180 roll to hand

import time
gc.loadWorld("plannerMug")

gc.approachToContact()
gc.setDynamics(True)
gc.autoGrasp()
while not gc.dynamicAutoGraspComplete():
    time.sleep(0.01)

gc.setDynamics(False)
result = gc.computeQuality()

gc.setDynamics(False)
gc.toggleDynamicsController(False)

while not gc.dynamicAutoGraspComplete():
    r = gc.getRobot(0)
    print r.robot.dofs
    gc.setRobotDOFForces([0, 100000000, 100000000, 100000000])


gc.stepDynamics(1)