# A more advanced example to get you moving with the RoboDK python API
# Note, as there are many solutions for a given pose, sometimes when
# running this, the robot may choose a weird pose that then doesn't allow
# the subsequent motion (due to being near a singularity etc). If this occurs, 
# just manually reset the robot startingposition to somewhere else and try again
# C Pretty, 18 Sept 2019
# version 2

# Define Imports.
import robolink as rl    # RoboDK API
import robodk as rdk     # Robot toolbox
import numpy as np

# Set up Robot.
RDK = rl.Robolink()

robot = RDK.Item('UR5')
world_frame = RDK.Item('UR5 Base')
target = RDK.Item('Home')   # existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())

# Existing subprograms

'''
RDK.RunProgram("Grinder Tool Attach (Stand)", True)
RDK.RunProgram("Grinder Tool Attach (Stand)", True)
RDK.RunProgram("Portafilter Tool Attach (Stand)", True)
RDK.RunProgram("Portafilter Tool Detach (Stand)", True)
RDK.RunProgram("Cup Tool Attach (Stand)", True)
RDK.RunProgram("Cup Tool Detach (Stand)", True)
RDK.RunProgram("Cup Tool Open", True)
RDK.RunProgram("Cup Tool Close", True)
RDK.RunProgram("Portafilter Tool Attach (Grinder)", True)
RDK.RunProgram("Portafilter Tool Detach (Grinder)", True)
RDK.RunProgram("Portafilter Tool Detach (Silvia)", True)
'''

# Global angle transforms. Degrees --> radians.
theta_coffee_machine = np.radians(15.04)
theta_grinder = np.radians(134.86)
theta_press = np.radians(59.73)

# Define the base transforms.
T_coffee_machine_base_np = np.array([[np.cos(theta_coffee_machine), -np.sin(theta_coffee_machine), 0.000000, -366.200000],
                                     [np.sin(theta_coffee_machine),  np.cos(theta_coffee_machine), 0.000000, -389.800000],
                                     [                    0.000000,                      0.000000, 1.000000,  341.380000],
                                     [                    0.000000,                      0.000000, 0.000000,    1.000000]])

T_coffee_machine_base = rdk.Mat(T_coffee_machine_base_np.tolist())

T_grinder_base_np = np.array([[np.cos(theta_grinder), -np.sin(theta_grinder), 0.000000,  482.290000],
                              [np.sin(theta_grinder),  np.cos(theta_grinder), 0.000000, -433.740000],
                              [             0.000000,               0.000000, 1.000000,  314.130000],
                              [             0.000000,               0.000000, 0.000000,    1.000000]])

T_grinder_base = rdk.Mat(T_grinder_base_np.tolist())

T_press_base_np = np.array([[np.cos(theta_press), -np.sin(theta_press), 0.000000,  599.130000],
                            [np.sin(theta_press),  np.cos(theta_press), 0.000000,    0.000000],
                            [           0.000000,             0.000000, 1.000000,  156.070000],
                            [           0.000000,             0.000000, 0.000000,    1.000000]])

T_press_base = rdk.Mat(T_press_base_np.tolist())
                                    
T_cup_base_np = np.array([[0.000000, 0.000000, 0.000000,    1.490000],
                          [0.000000, 0.000000, 0.000000, -600.540000],
                          [0.000000, 0.000000, 1.000000,  -20.000000],
                          [0.000000, 0.000000, 0.000000,    1.000000]])

T_cup_base = rdk.Mat(T_cup_base_np.tolist())

T_tool_stand_base_np = np.array([[0.000000, 0.000000, 0.000000, -544.570000],
                                 [0.000000, 0.000000, 0.000000,  -80.150000],
                                 [0.000000, 0.000000, 1.000000,   19.050000],
                                 [0.000000, 0.000000, 0.000000,    1.000000]])

T_tool_stand_base = rdk.Mat(T_tool_stand_base_np.tolist())

""" Initial Code. """                                    
# Directly use the RDK Matrix object from to hold pose (its an HT)
T_home = rdk.Mat([[ 0.000000,  0.000000, 1.000000,  523.370000 ],
                  [-1.000000,  0.000000, 0.000000, -109.000000 ],
                  [-0.000000, -1.000000, 0.000000,  607.850000 ],
                  [ 0.000000,  0.000000, 0.000000,    1.000000 ]])
                                 
# Joint angles
J_intermediatepoint = [-151.880896, -97.616411, -59.103383, -112.890980, 90.242082, -161.879346]

# Convert a numpy array into a Mat (e.g.after calculation)
T_grinderapproach_np = np.array([[     0.173648,    -0.984800,    -0.004000,  -502.103741],
    [ -0.984789,    -0.173618,    -0.006928,  -145.353888 ],
    [  0.006128,     0.005142,    -0.999968,   535.250260 ],
    [  0.000000,     0.000000,     0.000000,     1.000000 ]])

T_grinderapproach = rdk.Mat(T_grinderapproach_np.tolist())

""" Finish. """

# Set up Robot moves and function calls in desired fashion.
robot.MoveJ(T_home, blocking=True)

rdk.pause(3)

robot.MoveJ(J_intermediatepoint, blocking=True)

rdk.pause(3)

robot.MoveL(T_grinderapproach, blocking=True)

rdk.pause(3)

robot.MoveJ(T_coffee_machine_base, blocking=True)

rdk.pause(3)

robot.MoveJ(T_grinder_base, blocking=True)

rdk.pause(3)

robot.MoveJ(T_press_base, blocking=True) 

rdk.pause(3)

robot.MoveJ(T_cup_base, blocking=True) 

rdk.pause(3)

robot.MoveJ(T_tool_stand_base, blocking=True)

 # call subprogram
 # to allow subprogram to complete
 # call subfunction

# The following pause is very important - if it is not present, or long enough
# the frame reset below it occurs before the subprogram completes and this
# causes problems...
   
# Note, the subfunctions change the reference frame, so you need to change it back
# after calling them
robot.setPoseFrame(world_frame)
# you may also need to reset the toolframe
robot.setPoseTool(robot.PoseTool())

# and... move home to an existing target
robot.MoveJ(target)


