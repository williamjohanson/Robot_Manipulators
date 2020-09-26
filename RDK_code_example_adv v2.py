# A more advanced example to get you moving with the RoboDK python API
# Note, as there are many solutions for a given pose, sometimes when
# running this, the robot may choose a weird pose that then doesn't allow
# the subsequent motion (due to being near a singularity etc). If this occurs, 
# just manually reset the robot startingposition to somewhere else and try again
# C Pretty, 18 Sept 2019
# version 2

import robolink as rl    # RoboDK API
import robodk as rdk     # Robot toolbox
import numpy as np

RDK = rl.Robolink()

robot = RDK.Item('UR5')
world_frame = RDK.Item('UR5 Base')
target = RDK.Item('Home')   # existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())

#Existing subprograms
grinder_tool_attach = RDK.Item('Grinder Tool Attach')
grinder_tool_detach = RDK.Item('Grinder Tool Detach')

# Directly use the RDK Matrix object from to hold pose (its an HT)
T_home = rdk.Mat([[     0.000000,     0.000000,     1.000000,   523.370000 ],
     [-1.000000,     0.000000,     0.000000,  -109.000000 ],
     [-0.000000,    -1.000000,     0.000000,   607.850000 ],
      [0.000000,     0.000000,     0.000000,     1.000000 ]])

# Joint angles
J_intermediatePointPusher = [-151.880896, -97.616411, -59.103383, -112.890980, 90.242082, -161.879346]
J_intermediateCupTool = [-183.954225, -98.097022, -52.302160, -119.399545, 90.411813, -193.952774]
T_intermediatePortafilter_np = np.array([[ 0,-1, 0, -399.59],
                                 [-1, 0, 0,  -55.10],
                                 [ 0, 0,-1,  534.02],
                                 [ 0, 0, 0,    1.00]])
# Convert a numpy array into a Mat (e.g.after calculation)
T_grinderToolApproach_np = np.array([[     0.173648,    -0.984800,    -0.004000,  -502.103741],
    [ -0.984789,    -0.173618,    -0.006928,  -145.353888 ],
    [  0.006128,     0.005142,    -0.999968,   535.250260 ],
    [  0.000000,     0.000000,     0.000000,     1.000000 ]])

theta_coffee_machine = 15.04
theta_grinder_machine = 134.86
theta_press = 59.73

T_coffeeMachineBase_np = np.array([[ np.cos(theta_coffee_machine), -np.sin(theta_coffee_machine),  0.000000,  -366.20],
                                   [ np.sin(theta_coffee_machine),  np.cos(theta_coffee_machine),  0.000000,  -389.80],
                                   [                     0.000000,                      0.000000,  1.000000,   341.38],
                                   [                     0.000000,                      0.000000,  0.000000, 1.000000]])

T_grinderBase_np = np.array([[ np.cos(theta_grinder_machine), -np.sin(theta_grinder_machine),  0.000000,  482.29],
                             [ np.sin(theta_grinder_machine),  np.cos(theta_grinder_machine),  0.000000,  -433.74],
                             [                      0.000000,                       0.000000,  1.000000,   314.13],
                             [                      0.000000,                       0.000000,  0.000000, 1.000000]])
  
T_pressBase_np = np.array([[ np.cos(theta_press), -np.sin(theta_press),  0.000000,  -366.20],
                        [ np.sin(theta_press),  np.cos(theta_press),  0.000000,  -389.80],
                        [            0.000000,             0.000000,  1.000000,   341.38],
                        [            0.000000,             0.000000,  0.000000, 1.000000]])


#Tryimg to get the portafilter over to the grinder
T_portafilterCallPoint_np = np.array([[ 0.000000, np.sin(50),  np.cos(50),  157.61],
                                   [ 0.000000, np.cos(50), -np.sin(50), 0],
                                   [ -1.00000,  0.0000000,  0.00000000,   -250.45],
                                   [ 0.000000,  0.0000000,  0.00000000,   1.000]])

#portaPosition = np.array([1.00, 1.00, 1.00, 1.00])
grinder_base_position_np = ([[482.29, -433.74, 314.13]])
#T_portafilterCallPoint_np = T_grinderBase_np.dot(portaPosition)

grinder_base_position = rdk.Mat(grinder_base_position_np)
T_grinderToolApproach = rdk.Mat(T_grinderToolApproach_np.tolist())
T_coffeeMachineBase = rdk.Mat(T_coffeeMachineBase_np.tolist())
T_grinderBase = rdk.Mat(T_grinderBase_np.tolist())
T_pressBase = rdk.Mat(T_pressBase_np.tolist())
T_intermediatePortafilter = rdk.Mat(T_intermediatePortafilter_np.tolist())

T_portafilterCallPoint = rdk.Mat(T_portafilterCallPoint_np.tolist())



robot.MoveJ(T_home, blocking=True)
#robot.MoveJ(J_intermediateCupTool, blocking=True)
robot.MoveJ(T_intermediatePortafilter, blocking=True)

#robot.MoveJ(T_grinderToolApproach, blocking=True)
#robot.MoveJ(J_intermediatePointPusher, blocking=True)
#robot.MoveL(T_grinderToolApproach, blocking=True)
#robot.MoveJ(T_coffeeMachineBase, blocking=True)


#RDK.RunProgram('Portafilter Tool Attach (Stand)', True) # call subprogram
#RDK.RunProgram('Portafilter Tool Detach (Stand)', True) # call subprogram
#RDK.RunProgram('Grinder Tool Attach (Stand)', True) # call subprogram
#RDK.RunProgram('Grinder Tool Attach (Stand)', True) # call subprogram
#RDK.RunProgram('Grinder Tool Detach (Stand)', True) # call subfunction

#robot.MoveJ(T_grinderBase, blocking=True)
#robot.MoveJ(T_portafilterCallPoint, blocking=True)
robot.MoveJ(grinder_base_position, blocking=True)
# The following pause is very important - if it is not present, or long enough
# the frame reset below it occurs before the subprogram completes and this
# causes problems...
rdk.pause(3)    
# Note, the subfunctions change the reference frame, so you need to change it back
# after calling them
robot.setPoseFrame(world_frame)
# you may also need to reset the toolframe
robot.setPoseTool(robot.PoseTool())

# and... move home to an existing target
#robot.MoveJ(target)


