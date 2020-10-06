###################################################################################################
""" Coffee machine button presses sole file/function.
    Movement defined to collect the button (grinder) tool and move to an intermediary.
    The buttons then have to be aligned in a perpendicular frame to the coffee machine to
    allow for the buttons to be pressed.
    Complete requisite button presses, watch for the tool rack and complete the tool use.


    William Johanson 
    ENMT482

"""
###################################################################################################


# Define Imports.
import robolink as rl       # RoboDK API
import robodk as rdk        # Robot toolbox
import numpy as np          # Mathematics toolbox

# Set up Robot.
RDK = rl.Robolink()
robot = RDK.Item('UR5')                     # Define the robot.
world_frame = RDK.Item('UR5 Base')          # Define global frame as base of robot.
target = RDK.Item('Home')                   # Existing target in station
master_tool = RDK.Item('Master Tool')
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())

###################################################################################################
""" Transforms. """
###################################################################################################

# Orientate frame to get the grabber to match cups.
T_coffee_button_frame_np = np.array([[0.000000, 0.000000,  1.000000, 0.000000],
                                     [0.000000, 1.000000, 0.000000, 0.000000],
                                     [-1.000000, 0.000000,  0.000000, 0.000000],
                                     [0.000000, 0.000000,  0.000000, 1.000000]])

# Set the base frame of the coffee machine.
T_coffee_machine_base_np = np.array([[ np.cos(np.radians(74.96)), np.sin(np.radians(74.96)),  0.000000, -366.200000],
                                     [-np.sin(np.radians(74.96)), np.cos(np.radians(74.96)),  0.000000, -389.800000],
                                     [                  0.000000,                  0.000000, 1.000000,  341.380000],
                                     [                  0.000000,                  0.000000,  0.000000,    1.000000]])

# Angle to orient the end effector.
T_cup_grabber_angle_np = np.array([[ np.cos(np.radians(40)), np.sin(np.radians(40)), 0.000000, 0.000000],
                                   [-np.sin(np.radians(40)), np.cos(np.radians(40)), 0.000000, 0.000000],
                                   [               0.000000,               0.000000, 1.000000, 0.000000],
                                   [               0.000000,               0.000000, 0.000000, 1.000000]])

# Coffee machine frame from world frame transform.
coffee_machine_frame_np = np.matmul(T_coffee_machine_base_np, T_coffee_button_frame_np)
# Multiply to orient the end effector.
coffee_machine_orient_frame_np = np.matmul(coffee_machine_frame_np, T_cup_grabber_angle_np)

###################################################################################################
""" Offsets. """
###################################################################################################    
button_center_np = np.array([[0.000000, 0.000000, 0.000000,   35.250000],
                             [0.000000, 0.000000, 0.000000,  -30.000000],
                             [0.000000, 0.000000, 0.000000, -170.000000],
                             [0.000000, 0.000000, 0.000000,    0.000000]])   

button_top_np = np.array([[0.000000, 0.000000, 0.000000,   25.250000],
                          [0.000000, 0.000000, 0.000000,  -30.000000],
                          [0.000000, 0.000000, 0.000000, -152.000000],
                          [0.000000, 0.000000, 0.000000,    0.000000]])         

button_bottom_np = np.array([[0.000000, 0.000000, 0.000000,   40.25000],
                             [0.000000, 0.000000, 0.000000,  -30.000000],
                             [0.000000, 0.000000, 0.000000, -152.000000],
                             [0.000000, 0.000000, 0.000000,    0.000000]])                                                                                                


button_center_np = np.matmul(coffee_machine_frame_np, T_cup_grabber_angle_np + button_center_np)
button_top_np = np.matmul(coffee_machine_frame_np, T_cup_grabber_angle_np + button_top_np)
button_bottom_np = np.matmul(coffee_machine_frame_np, T_cup_grabber_angle_np + button_bottom_np)
###################################################################################################
""" NP to rdk.Mat conversions. """
###################################################################################################                     
coffee_machine_frame = rdk.Mat(coffee_machine_frame_np.tolist())
coffee_machine_orient_frame = rdk.Mat(coffee_machine_orient_frame_np.tolist())
button_center = rdk.Mat(button_center_np.tolist())
button_top = rdk.Mat(button_top_np.tolist())
button_bottom = rdk.Mat(button_bottom_np.tolist())

###################################################################################################
""" Intermediate points. """
###################################################################################################
J_intermediateGrinderTool = [-157.500000, -83.570000, -77.140000, -91.220000, 175.490000, -222.360000]

###################################################################################################
""" Scheduler. """
###################################################################################################
#robot.setPoseTool(master_tool)
#robot.MoveJ(J_intermediateGrinderTool, blocking=True)
#RDK.RunProgram('Grinder Tool Attach (Stand)', True)
#robot.setPoseTool(master_tool)
#robot.MoveJ(J_intermediateGrinderTool, blocking=True)
#robot.MoveJ(coffee_machine_frame, blocking=True)
#robot.MoveJ(coffee_machine_orient_frame, blocking=True)
#robot.MoveJ(button_center, blocking=True)
#robot.MoveJ(button_top, blocking=True)
#rdk.pause(7)
#robot.MoveJ(button_center, blocking=True)
#robot.MoveJ(button_bottom, blocking=True)
#robot.MoveJ(button_center, blocking=True)
#robot.MoveJ(J_intermediateGrinderTool, blocking=True)
#RDK.RunProgram('Grinder Tool Detach (Stand)', True)

###################################################################################################
""" Good content. """
###################################################################################################
robot.setPoseTool(master_tool)

robot.MoveJ(J_intermediateGrinderTool, blocking=True)

RDK.RunProgram('Grinder Tool Attach (Stand)', True)

robot.setPoseTool(master_tool)

robot.MoveJ(J_intermediateGrinderTool, blocking=True)

robot.MoveJ(button_center, blocking=True)

robot.MoveJ(button_top, blocking=True)

rdk.pause(7)

robot.MoveJ(button_center, blocking=True)

robot.MoveJ(button_bottom, blocking=True)

robot.MoveJ(button_center, blocking=True)

robot.MoveJ(J_intermediateGrinderTool, blocking=True)

RDK.RunProgram('Grinder Tool Detach (Stand)', True)