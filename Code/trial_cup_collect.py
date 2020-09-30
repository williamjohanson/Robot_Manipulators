###################################################################################################
""" Cup collection sole file/function.
    Movement defined to collect the grabber unit and proceed through a series of intermediates to the 
    cup stand.
    Conversion to cup orientation and complete the grabbing of the cups.
    Shifting the cup to the coffee machine in coffee machine orientation.
    Shift the cup (without spilling) to the TA.

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

# Base of the cups position in world frame.
T_cup_base_np = np.array([[-1.000000, 0.000000,  0.000000,    1.490000],
                          [ 0.000000, 1.000000,  0.000000, -600.540000],
                          [ 0.000000, 0.000000, -1.000000,  -20.000000],
                          [ 0.000000, 0.000000,  0.000000,    1.000000]])

# Orientate frame to get the grabber to match cups.
T_cup_grabber_frame_np = np.array([[1.000000, 0.000000,  0.000000, 0.000000],
                                   [0.000000, 0.000000, -1.000000, 0.000000],
                                   [0.000000, 1.000000,  0.000000, 0.000000],
                                   [0.000000, 0.000000,  0.000000, 1.000000]])

# Cup frame from world frame transform.
cup_frame_np = np.matmul(T_cup_base_np, T_cup_grabber_frame_np)

# Twist the end effector to complete orientation of grabber with cups and shift back to center the grabber piece with the cup base.
T_cup_angle_np = np.array([[ np.cos(np.radians(40)), np.sin(np.radians(40)), 0.000000,    0.000000],
                           [-np.sin(np.radians(40)), np.cos(np.radians(40)), 0.000000,  -47.000000],
                           [               0.000000,               0.000000, 1.000000, -186.110000],
                           [               0.000000,               0.000000, 0.000000,    1.000000]])

# Shift by the desired angle with the cup transform.
oriented_frame_np = np.matmul(cup_frame_np, T_cup_angle_np)   

T_coffee_machine_base_np = np.array([[-np.cos(np.radians(15.04)), -np.sin(np.radians(15.04)),  0.000000, -366.200000],
                                     [ np.sin(np.radians(15.04)),  np.cos(np.radians(15.04)),  0.000000, -389.800000],
                                     [                  0.000000,                   0.000000, -1.000000,  341.380000],
                                     [                  0.000000,                   0.000000,  0.000000,    1.000000]])

T_cup_grabber_angle_np = np.array([[ np.cos(np.radians(40)), np.sin(np.radians(40)), 0.000000, 0.000000],
                                   [-np.sin(np.radians(40)), np.cos(np.radians(40)), 0.000000, 0.000000],
                                   [               0.000000,               0.000000, 1.000000, 0.000000],
                                   [               0.000000,               0.000000, 0.000000, 1.000000]])

# Coffee machine frame from world frame transform.
coffee_machine_frame_np = np.matmul(T_coffee_machine_base_np, T_cup_grabber_frame_np)
coffee_machine_orient_frame_np = np.matmul(coffee_machine_frame_np, T_cup_grabber_angle_np)

###################################################################################################
""" Offsets. """
###################################################################################################

# Set the offset matrix to move to the edge of the cup.
cup_offset_np = np.array([[0.000000, 0.000000, 0.000000,    0.000000],
                          [0.000000, 0.000000, 0.000000, -204.000000],
                          [0.000000, 0.000000, 0.000000,  -80.000000],
                          [0.000000, 0.000000, 0.000000,    0.000000]])    

# Set the offset matrix to move to the edge of the cup.
cup_centre_np = np.array([[0.000000, 0.000000, 0.000000,    0.000000],
                          [0.000000, 0.000000, 0.000000, -204.000000],
                          [0.000000, 0.000000, 0.000000,  -10.000000],
                          [0.000000, 0.000000, 0.000000,    0.000000]])   

# Set an offset to align with coffee machine base
coffee_machine_offset_np = np.array([[0.000000, 0.000000, 0.000000,   90.000000],
                                     [0.000000, 0.000000, 0.000000,  180.000000],
                                     [0.000000, 0.000000, 0.000000, -120.000000],
                                     [0.000000, 0.000000, 0.000000,    0.000000]])                                                                

# Add in offsets in the cup frame.
Align_cup_np = np.matmul(cup_frame_np, T_cup_angle_np + cup_offset_np)  
Grabber_cup_centre_np = np.matmul(cup_frame_np, T_cup_angle_np + cup_centre_np)  
Align_coffee_machine_np = np.matmul(coffee_machine_frame_np, T_cup_grabber_angle_np + coffee_machine_offset_np)

###################################################################################################
""" NP to rdk.Mat conversions. """
###################################################################################################

# Convert to rdk.Mat format.
T_cup_base = rdk.Mat(T_cup_base_np.tolist())

T_cup_grabber_frame = rdk.Mat(T_cup_grabber_frame_np.tolist())

cup_frame = rdk.Mat(cup_frame_np.tolist())

T_cup_angle = rdk.Mat(T_cup_angle_np.tolist())

oriented_frame = rdk.Mat(oriented_frame_np.tolist())

Align_cup = rdk.Mat(Align_cup_np.tolist())

Grabber_cup_centre = rdk.Mat(Grabber_cup_centre_np.tolist())

coffee_machine_frame = rdk.Mat(coffee_machine_frame_np.tolist())

coffee_machine_orient_frame = rdk.Mat(coffee_machine_orient_frame_np.tolist())

Align_coffee_machine = rdk.Mat(Align_coffee_machine_np.tolist())

###################################################################################################
""" Intermediate points. """
###################################################################################################

J_cup_tool_orient = [-180.000000, -77.598188, -78.444613, -114.520498, 90.000000, -205.000000]

J_cup_intermediate_point = [-45.454914, -68.283104, -92.404605, -109.833463, 90.216157, -11.624553]

J_cup_intermediate_point_2 =[-55.601183, -64.330829, -153.208867, -140.295331, -59.546467, 139.675612]

J_cup_to_coffee_machine_intermediate_point = [-151.474573, -65.463223, -138.288288, -151.772707, -155.335681, 144.842182]

J_cup_to_coffee_machine_intermediate_point_2 = [-177.411002, -107.463961, -105.867160, -146.668880, -192.451002, 140.000000]
###################################################################################################
""" Scheduler. """
###################################################################################################
'''
#robot.MoveJ(T_cup_base, blocking=True)

#robot.MoveJ(cup_frame, blocking=True)

#robot.MoveJ(oriented_frame, blocking=True)
'''

robot.MoveJ(J_cup_tool_orient, blocking=True)

RDK.RunProgram("Cup Tool Attach (Stand)", True)

robot.setPoseTool(master_tool)

robot.MoveJ(J_cup_intermediate_point, blocking=True)

robot.MoveJ(J_cup_intermediate_point_2, blocking=True)

robot.MoveJ(Align_cup, blocking=True)

RDK.RunProgram("Cup Tool Open", True)

robot.MoveJ(Grabber_cup_centre, blocking=True)

RDK.RunProgram("Cup Tool Close", True)

robot.MoveJ(Align_cup, blocking=True)

robot.MoveJ(J_cup_intermediate_point_2, blocking=True)

robot.MoveJ(J_cup_to_coffee_machine_intermediate_point, blocking=True)

robot.MoveJ(J_cup_to_coffee_machine_intermediate_point_2, blocking=True)

#robot.MoveJ(coffee_machine_frame, blocking=True)

#robot.MoveJ(coffee_machine_orient_frame, blocking=True)

robot.MoveJ(Align_coffee_machine, blocking=True)

RDK.RunProgram("Cup Tool Open", True)

# Two intermediates.

# Coffee machine align.

# Coffee machine place.

# Return grabber to rack.

# ...

# Recollect grabber.

# BAck through coffee machine.

# Collect and small intermediate movements to the TA?

#robot.MoveJ(frame_base, blocking=True)

#RDK.RunProgram("Cup Tool Detach (Stand)", True)
#RDK.RunProgram("Cup Tool Attach (Stand)", True)
