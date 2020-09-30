# A more advanced example to get you moving with the RoboDK python API
# Note, as there are many solutions for a given pose, sometimes when
# running this, the robot may choose a weird pose that then doesn't allow
# the subsequent motion (due to being near a singularity etc). If this occurs, 
# just manually reset the robot startingposition to somewhere else and try again
# C Pretty, 18 Sept 2019
# version 2

# Define Imports.
import robolink as rl       # RoboDK API
import robodk as rdk        # Robot toolbox
import numpy as np          # Mathematics toolbox

# Set up Robot.
RDK = rl.Robolink()
robot = RDK.Item('UR5')                     # Define the robot.
world_frame = RDK.Item('UR5 Base')          # Define global frame as base of robot.
target = RDK.Item('Home')                   # Existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())

master_tool = RDK.Item('Master Tool')
robot.setPoseTool(master_tool)

# Define existing subprograms.

###
###RDK.RunProgram("Cup Tool Attach (Stand)", True)
###RDK.RunProgram("Cup Tool Detach (Stand)", True)
###RDK.RunProgram("Cup Tool Open", True)
###RDK.RunProgram("Cup Tool Close", True)
###

# Global angle transforms. Degrees --> radians.
theta_coffee_machine = np.radians(15.04)
theta_grinder = np.radians(134.86)
theta_press = np.radians(59.73)

# Directly use the RDK Matrix object from to hold pose (its an HT)
T_home = rdk.Mat([[ 0.000000,  0.000000, 1.000000,  523.370000 ],
                  [-1.000000,  0.000000, 0.000000, -109.000000 ],
                  [-0.000000, -1.000000, 0.000000,  607.850000 ],
                  [ 0.000000,  0.000000, 0.000000,    1.000000 ]])

# Define the utilised base transforms.
T_cup_base_np = np.array([[1.000000, 0.000000,  0.000000,    1.490000],
                          [0.000000, 1.000000,  0.000000, -600.540000],
                          [0.000000, 0.000000, -1.000000,  -20.000000],
                          [0.000000, 0.000000,  0.000000,    1.000000]])

T_cup_base = rdk.Mat(T_cup_base_np.tolist())

T_cup_frame_np = np.array([[-1.000000,  0.000000, 0.000000,    1.490000],
                           [0.000000,  0.000000, -1.000000, -600.540000],
                           [0.000000, 1.000000, 0.000000,  -20.000000],
                           [0.000000,  0.000000, 0.000000,    1.000000]])

T_cup_frame = rdk.Mat(T_cup_frame_np.tolist())

#Induced offset.
offset = 135.194
T_cup_centre_tool_np = T_cup_frame_np + np.array([[0.000000, 0.000000, 0.000000,   0.000000],
                                                  [0.000000, 0.000000, 0.000000, 186.110000],
                                                  [0.000000, 0.000000, 0.000000,  47.000000 + offset],
                                                  [0.000000, 0.000000, 0.000000,   1.000000]])

T_cup_centre_tool = rdk.Mat(T_cup_centre_tool_np.tolist())

T_cup_edge_np = T_cup_centre_tool_np + np.array([[0.000000, 0.000000, 0.000000,   0.000000],
                                                 [0.000000, 0.000000, 0.000000,  36.080000],
                                                 [0.000000, 0.000000, 0.000000, 147.250000],
                                                 [0.000000, 0.000000, 0.000000,   1.000000]])

T_cup_edge = rdk.Mat(T_cup_edge_np.tolist())

T_cup_angle_np =  np.matmul(np.array([[np.cos(np.radians(40)), -np.sin(np.radians(40)), 0.000000, 0.000000],
                                      [np.sin(np.radians(40)),  np.cos(np.radians(40)), 0.000000, 0.000000],
                                      [              0.000000,                0.000000, 1.000000, 0.000000],
                                      [              0.000000,                0.000000, 0.000000, 1.000000]]), T_cup_edge_np)

T_cup_angle = rdk.Mat(T_cup_angle_np.tolist())                           

T_tool_stand_base_np = np.array([[1.000000, 0.000000,  0.000000, -544.570000],
                                 [0.000000, 1.000000,  0.000000,  -80.150000],
                                 [0.000000, 0.000000, -1.000000,   19.050000],
                                 [0.000000, 0.000000,  0.000000,    1.000000]])

T_tool_stand_base = rdk.Mat(T_tool_stand_base_np.tolist())

T_cup_tool_np = np.array([[1.000000, 0.000000,   0.000000, -358.130000],
                          [0.000000, 1.000000,   0.000000,  102.680000],
                          [0.000000, 0.000000, -1.000000,   532.670000],
                          [0.000000, 0.000000,   0.000000,    1.000000]])

T_cup_tool = rdk.Mat(T_cup_tool_np.tolist())

#T_cup_tool_orient_np = np.array([[np.cos(np.radians(50)), -np.sin(np.radians(50)),  0.000000, -458.130000],
#                                 [np.sin(np.radians(50)),  np.cos(np.radians(50)),  0.000000,  102.680000],
#                                 [              0.000000,                0.000000, -1.000000,  532.670000],
#                                 [              0.000000,                0.000000,  0.000000,    1.000000]])

# Define Joint angle moves.

J_cup_tool_orient = [-180.000000, -77.598188, -78.444613, -114.520498, 90.000000, -205.000000]

J_cup_intermediate_point = [-45.454914, -68.283104, -92.404605, -109.833463, 90.216157, -11.624553]

J_cup_intermediate_point_2 =[-55.601183, -64.330829, -153.208867, -140.295331, -59.546467, 139.675612]

J_cup_align_grab = [-63.510000, -85.360000, -143.820000, -130.810000, -63.510000, 142.140000]#
#T_cup_tool_orient = rdk.Mat(T_cup_tool_orient_np.tolist())



""" ROBOT SCHEDULER. """
# Set up Robot moves and function calls in desired fashion.
#robot.MoveJ(T_home, blocking=True)                   

#robot.MoveJ(J_cup_tool_orient, blocking=True) 

#RDK.RunProgram("Cup Tool Attach (Stand)", True)

#RDK.RunProgram("Cup Tool Open", True)

#robot.MoveJ(J_cup_intermediate_point, blocking=True) 

#robot.MoveJ(J_cup_intermediate_point_2, blocking=True) 

#robot.MoveJ(T_cup_base, blocking=True) 

#robot.MoveJ(T_cup_frame, blocking=True) 

#robot.MoveJ(T_cup_centre_tool, blocking=True) 

robot.MoveJ(T_cup_edge, blocking=True) 

robot.MoveJ(T_cup_angle, blocking=True) 

#robot.MoveJ(J_cup_align_grab, blocking=True) 

#rdk.pause(10)

#RDK.RunProgram("Cup Tool Close", True)

#robot.MoveJ(J_cup_intermediate_point_2, blocking=True) 

#robot.MoveJ(J_cup_intermediate_point, blocking=True) 

#robot.MoveJ(J_cup_tool_orient, blocking=True) 

#RDK.RunProgram("Cup Tool Detach (Stand)", True)

# Note, the subfunctions change the reference frame, so you need to change it back
# after calling them
robot.setPoseFrame(world_frame)

# you may also need to reset the toolframe
robot.setPoseTool(robot.PoseTool())


