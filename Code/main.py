# William Johanson
# ENMT482
# Robot Manipulators
#
#
#
#
#
#####################################################################################################################################################


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
#####################################################################################################################################################
""" General Code variables. """
#####################################################################################################################################################

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

#####################################################################################################################################################
""" Coffee machine buttons global variables. """
#####################################################################################################################################################
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

#####################################################################################################################################################


def coffee_machine_buttons():
    """ Coffee machine buttons to turn the coffee machine on to pour the coffee *** 7 seconds ***. """
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

    robot.setPoseTool(master_tool)

    robot.MoveJ(J_intermediateGrinderTool, blocking=True)

#####################################################################################################################################################
""" Cup collection global variables. """
#####################################################################################################################################################

###################################################################################################
""" Transforms. """
###################################################################################################

# Base of the cups position in world frame.
T_cup_base_np = np.array([[-1.000000, 0.000000, 0.000000,    1.490000],
                          [0.000000, 1.000000, 0.000000, -600.540000],
                          [0.000000, 0.000000, -1.000000,  -20.000000],
                          [0.000000, 0.000000, 0.000000,    1.000000]])

# Orientate frame to get the grabber to match cups.
T_cup_grabber_frame_np = np.array([[1.000000, 0.000000,  0.000000, 0.000000],
                                   [0.000000, 0.000000, -1.000000, 0.000000],
                                   [0.000000, 1.000000,  0.000000, 0.000000],
                                   [0.000000, 0.000000,  0.000000, 1.000000]])

# Orientate frame to get the grabber to match cups.
T_coffee_grabber_frame_np = np.array([[1.000000, 0.000000,  0.000000, 0.000000],
                                      [0.000000, 0.000000, -1.000000, 0.000000],
                                      [0.000000, -1.000000,  0.000000, 0.000000],
                                      [0.000000, 0.000000,  0.000000, 1.000000]])                                   

# Cup frame from world frame transform.
cup_frame_np = np.matmul(T_cup_base_np, T_cup_grabber_frame_np)

# Twist the end effector to complete orientation of grabber with cups and shift back to center the grabber piece with the cup base.
T_cup_angle_np = np.array([[ np.cos(np.radians(140)), -np.sin(np.radians(140)), 0.000000,    0.000000],
                           [np.sin(np.radians(140)), np.cos(np.radians(140)), 0.000000,  -47.000000],
                           [               0.000000,               0.000000, 1.000000, -186.110000],
                           [               0.000000,               0.000000, 0.000000,    1.000000]])

# Shift by the desired angle with the cup transform.
oriented_frame_np = np.matmul(cup_frame_np, T_cup_angle_np)   

T_coffee_machine_base_np = np.array([[ np.cos(np.radians(74.96)), np.sin(np.radians(74.96)),  0.000000, -366.200000],
                                     [-np.sin(np.radians(74.96)), np.cos(np.radians(74.96)),  0.000000, -389.800000],
                                     [                  0.000000,                  0.000000, 1.000000,  341.380000],
                                     [                  0.000000,                  0.000000,  0.000000,    1.000000]])

T_cup_grabber_angle_np = np.array([[np.cos(np.radians(140)), -np.sin(np.radians(140)), 0.000000, 0.000000],
                                   [np.sin(np.radians(140)), np.cos(np.radians(140)), 0.000000, 0.000000],
                                   [               0.000000,               0.000000, 1.000000, 0.000000],
                                   [               0.000000,               0.000000, 0.000000, 1.000000]])

# Coffee machine frame from world frame transform.
coffee_machine_frame_np = np.matmul(T_coffee_machine_base_np, T_coffee_grabber_frame_np)
coffee_machine_orient_frame_np = np.matmul(coffee_machine_frame_np, T_cup_grabber_angle_np)

###################################################################################################
""" Offsets. """
###################################################################################################

# Set the offset matrix to move to the edge of the cup.
cup_offset_np = np.array([[0.000000, 0.000000, 0.000000,    0.000000],
                          [0.000000, 0.000000, 0.000000,  -54.000000],
                          [0.000000, 0.000000, 0.000000, -100.000000],
                          [0.000000, 0.000000, 0.000000,    0.000000]])    

# Set the offset matrix to move to the edge of the cup.
cup_centre_np = np.array([[0.000000, 0.000000, 0.000000,   0.000000],
                          [0.000000, 0.000000, 0.000000, -54.000000],
                          [0.000000, 0.000000, 0.000000,   0.000000],
                          [0.000000, 0.000000, 0.000000,   0.000000]])   

# Above cup to remove from stack
above_cup_centre_np = np.array([[0.000000, 0.000000, 0.000000,    0.000000],
                                [0.000000, 0.000000, 0.000000, -254.000000],
                                [0.000000, 0.000000, 0.000000,    0.000000],
                                [0.000000, 0.000000, 0.000000,    0.000000]])                             

# Set an offset to align with coffee machine base
coffee_machine_offset_np = np.array([[0.000000, 0.000000, 0.000000,    0.000000],
                                     [0.000000, 0.000000, 0.000000,  -50.000000],
                                     [0.000000, 0.000000, 0.000000, -250.000000],
                                     [0.000000, 0.000000, 0.000000,    0.000000]])      

# Center cup under portafilter
coffee_machine_center_np = np.array([[0.000000, 0.000000, 0.000000,    0.000000],
                                     [0.000000, 0.000000, 0.000000,  -50.000000],
                                     [0.000000, 0.000000, 0.000000, -100.000000],
                                     [0.000000, 0.000000, 0.000000,    0.000000]])                                                                                                

# Add in offsets in the cup frame.
Align_cup_np = np.matmul(cup_frame_np, T_cup_angle_np + cup_offset_np)  
Grabber_cup_centre_np = np.matmul(cup_frame_np, T_cup_angle_np + cup_centre_np)  
Grabber_above_cup_centre_np = np.matmul(cup_frame_np, T_cup_angle_np + above_cup_centre_np)  

Align_coffee_machine_np = np.matmul(coffee_machine_frame_np, T_cup_grabber_angle_np + coffee_machine_offset_np)
Center_coffee_machine_np = np.matmul(coffee_machine_frame_np, T_cup_grabber_angle_np + coffee_machine_center_np)

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

Center_coffee_machine = rdk.Mat(Center_coffee_machine_np.tolist())

Grabber_above_cup_centre = rdk.Mat(Grabber_above_cup_centre_np.tolist())

###################################################################################################
""" Intermediate points. """
###################################################################################################

J_cup_tool_orient = [-177.697218, -59.469365, -88.212296, -122.881182, 90.022633, -202.697330]

J_cup_intermediate_point = [-74.210000, -55.860000, -90.220000, -213.920000, 0.000000, -40.000000]

J_cup_intermediate_point_2 = [-76.740000, -102.140000, -81.270000, -356.590000, 0.000000, -40.000000]

J_cup_intermediate_point_3 =[1.592416, -118.102180, -219.675634, 337.777813, 1.592416, -40.000000]

J_cup_to_coffee_machine_intermediate_point = [95.050000, -105.730000, 119.070000, -13.330000, 109.290000, -220.000000]

#####################################################################################################################################################

def cup_collect():
    ###################################################################################################
    """ Scheduler. """
    ###################################################################################################
    #robot.MoveJ(T_cup_base, blocking=True)
    #robot.MoveJ(cup_frame, blocking=True)
    #robot.MoveJ(oriented_frame, blocking=True)
    #robot.MoveJ(J_cup_tool_orient, blocking=True)
    #robot.MoveJ(oriented_frame, blocking=True)
    #RDK.RunProgram("Cup Tool Attach (Stand)", True)
    #robot.setPoseTool(master_tool)
    #robot.MoveJ(J_cup_tool_orient, blocking=True)
    #robot.MoveJ(T_cup_base, blocking=True)
    #robot.MoveJ(J_cup_intermediate_point, blocking=True)
    #robot.MoveJ(cup_frame, blocking=True)
    #robot.MoveJ(J_cup_intermediate_point, blocking=True)
    #robot.MoveJ(oriented_frame, blocking=True)
    #robot.MoveJ(J_cup_intermediate_point, blocking=True)
    #robot.MoveJ(J_cup_intermediate_point_2, blocking=True)
    #robot.MoveJ(J_cup_intermediate_point_2, blocking=True)
    #robot.MoveJ(Align_cup, blocking=True)
    #RDK.RunProgram("Cup Tool Open", True)
    #robot.MoveJ(Grabber_cup_centre, blocking=True)
    #RDK.RunProgram("Cup Tool Close", True)
    #robot.MoveJ(Align_cup, blocking=True)
    #robot.MoveJ(J_cup_intermediate_point, blocking=True)
    #robot.MoveJ(J_cup_intermediate_point_2, blocking=True)
    #robot.MoveJ(J_cup_to_coffee_machine_intermediate_point, blocking=True)
    #robot.MoveJ(coffee_machine_frame, blocking=True)
    #robot.MoveJ(coffee_machine_orient_frame, blocking=True)
    #robot.MoveJ(Align_coffee_machine, blocking=True)
    #RDK.RunProgram("Cup Tool Open", True)

    #robot.MoveJ(Center_coffee_machine, blocking=True)

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


    ###################################################################################################
    """ Good content. """
    ###################################################################################################
    robot.MoveJ(J_cup_tool_orient, blocking=True)

    RDK.RunProgram("Cup Tool Attach (Stand)", True)

    robot.setPoseTool(master_tool)

    robot.MoveJ(J_cup_tool_orient, blocking=True)

    robot.MoveJ(J_cup_intermediate_point, blocking=True)

    robot.MoveJ(Align_cup, blocking=True)

    RDK.RunProgram("Cup Tool Open", True)

    robot.MoveJ(Grabber_cup_centre, blocking=True)

    RDK.RunProgram("Cup Tool Close", True)

    robot.MoveJ(Grabber_above_cup_centre, blocking=True)

    robot.MoveJ(J_cup_intermediate_point, blocking=True)

    robot.MoveJ(J_cup_to_coffee_machine_intermediate_point, blocking=True)

    robot.MoveJ(Align_coffee_machine, blocking=True)

    robot.MoveJ(Center_coffee_machine, blocking=True)

    RDK.RunProgram("Cup Tool Open", True)

    robot.MoveJ(Align_coffee_machine, blocking=True)

    RDK.RunProgram("Cup Tool Close", True)

    robot.MoveJ(J_cup_to_coffee_machine_intermediate_point, blocking=True)

    robot.MoveJ(J_cup_intermediate_point, blocking=True)

    robot.MoveJ(J_cup_tool_orient, blocking=True)

    RDK.RunProgram("Cup Tool Detach (Stand)", True)

    robot.setPoseTool(master_tool)


#####################################################################################################################################################
""" Cup completion variables. """
#####################################################################################################################################################
###################################################################################################
""" Transforms. """
###################################################################################################

# Base of the cups position in world frame.
T_cup_base_np = np.array([[-1.000000, 0.000000, 0.000000,    1.490000],
                          [0.000000, 1.000000, 0.000000, -600.540000],
                          [0.000000, 0.000000, -1.000000,  -20.000000],
                          [0.000000, 0.000000, 0.000000,    1.000000]])

# Orientate frame to get the grabber to match cups.
T_cup_grabber_frame_np = np.array([[1.000000, 0.000000,  0.000000, 0.000000],
                                   [0.000000, 0.000000, -1.000000, 0.000000],
                                   [0.000000, 1.000000,  0.000000, 0.000000],
                                   [0.000000, 0.000000,  0.000000, 1.000000]])

# Orientate frame to get the grabber to match cups.
T_coffee_grabber_frame_np = np.array([[1.000000, 0.000000,  0.000000, 0.000000],
                                      [0.000000, 0.000000, -1.000000, 0.000000],
                                      [0.000000, -1.000000, 0.000000, 0.000000],
                                      [0.000000, 0.000000,  0.000000, 1.000000]])                                    

# Cup frame from world frame transform.
cup_frame_np = np.matmul(T_cup_base_np, T_cup_grabber_frame_np)

# Twist the end effector to complete orientation of grabber with cups and shift back to center the grabber piece with the cup base.
T_cup_angle_np = np.array([[ np.cos(np.radians(140)), -np.sin(np.radians(140)), 0.000000,    0.000000],
                           [np.sin(np.radians(140)), np.cos(np.radians(140)), 0.000000,  -47.000000],
                           [               0.000000,               0.000000, 1.000000, -186.110000],
                           [               0.000000,               0.000000, 0.000000,    1.000000]])

# Shift by the desired angle with the cup transform.
oriented_frame_np = np.matmul(cup_frame_np, T_cup_angle_np)   

T_coffee_machine_base_np = np.array([[ np.cos(np.radians(74.96)), np.sin(np.radians(74.96)),  0.000000, -366.200000],
                                     [-np.sin(np.radians(74.96)), np.cos(np.radians(74.96)),  0.000000, -389.800000],
                                     [                  0.000000,                  0.000000, 1.000000,  341.380000],
                                     [                  0.000000,                  0.000000,  0.000000,    1.000000]])

T_cup_grabber_angle_np = np.array([[np.cos(np.radians(140)), -np.sin(np.radians(140)), 0.000000, 0.000000],
                                   [np.sin(np.radians(140)),  np.cos(np.radians(140)), 0.000000, 0.000000],
                                   [               0.000000,                 0.000000, 1.000000, 0.000000],
                                   [               0.000000,                 0.000000, 0.000000, 1.000000]])

# Coffee machine frame from world frame transform.
coffee_machine_frame_np = np.matmul(T_coffee_machine_base_np, T_coffee_grabber_frame_np)
coffee_machine_orient_frame_np = np.matmul(coffee_machine_frame_np, T_cup_grabber_angle_np)

###################################################################################################
""" Offsets. """
###################################################################################################

# Set the offset matrix to move to the edge of the cup.
cup_offset_np = np.array([[0.000000, 0.000000, 0.000000,    0.000000],
                          [0.000000, 0.000000, 0.000000,  -54.000000],
                          [0.000000, 0.000000, 0.000000, -100.000000],
                          [0.000000, 0.000000, 0.000000,    0.000000]])    

# Set the offset matrix to move to the edge of the cup.
cup_centre_np = np.array([[0.000000, 0.000000, 0.000000,   0.000000],
                          [0.000000, 0.000000, 0.000000, -54.000000],
                          [0.000000, 0.000000, 0.000000,   0.000000],
                          [0.000000, 0.000000, 0.000000,   0.000000]])   

# Above cup to remove from stack
above_cup_centre_np = np.array([[0.000000, 0.000000, 0.000000,    0.000000],
                                [0.000000, 0.000000, 0.000000, -254.000000],
                                [0.000000, 0.000000, 0.000000,    0.000000],
                                [0.000000, 0.000000, 0.000000,    0.000000]])                             

# Set an offset to align with coffee machine base
coffee_machine_offset_np = np.array([[0.000000, 0.000000, 0.000000,    0.000000],
                                     [0.000000, 0.000000, 0.000000, -50.000000],
                                     [0.000000, 0.000000, 0.000000, -250.000000],
                                     [0.000000, 0.000000, 0.000000,    0.000000]])      

# Center cup under portafilter
coffee_machine_center_np = np.array([[0.000000, 0.000000, 0.000000,    0.000000],
                                     [0.000000, 0.000000, 0.000000, -50.000000],
                                     [0.000000, 0.000000, 0.000000, -100.000000],
                                     [0.000000, 0.000000, 0.000000,    0.000000]])     

# Shift cup up to top of coffee machine
coffee_machine_offset_and_above_np = np.array([[0.000000, 0.000000, 0.000000,    0.000000],
                                               [0.000000, 0.000000, 0.000000,  -325.000000],
                                               [0.000000, 0.000000, 0.000000, -250.000000],
                                               [0.000000, 0.000000, 0.000000,    0.000000]])      

# Center cup above coffee machine
coffee_machine_center_and_above_np = np.array([[0.000000, 0.000000, 0.000000,    0.000000],
                                               [0.000000, 0.000000, 0.000000,  -325.000000],
                                               [0.000000, 0.000000, 0.000000, -100.000000],
                                               [0.000000, 0.000000, 0.000000,    0.000000]])                                                                                                                                                                         

# Add in offsets in the cup frame.
Align_cup_np = np.matmul(cup_frame_np, T_cup_angle_np + cup_offset_np)  
Grabber_cup_centre_np = np.matmul(cup_frame_np, T_cup_angle_np + cup_centre_np)  
Grabber_above_cup_centre_np = np.matmul(cup_frame_np, T_cup_angle_np + above_cup_centre_np)  

Align_coffee_machine_np = np.matmul(coffee_machine_frame_np, T_cup_grabber_angle_np + coffee_machine_offset_np)
Center_coffee_machine_np = np.matmul(coffee_machine_frame_np, T_cup_grabber_angle_np + coffee_machine_center_np)

Move_coffee_machine_offset_and_above_np = np.matmul(coffee_machine_frame_np, T_cup_grabber_angle_np + coffee_machine_offset_and_above_np)
Move_coffee_machine_center_and_above_np = np.matmul(coffee_machine_frame_np, T_cup_grabber_angle_np + coffee_machine_center_and_above_np)

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

Center_coffee_machine = rdk.Mat(Center_coffee_machine_np.tolist())

Grabber_above_cup_centre = rdk.Mat(Grabber_above_cup_centre_np.tolist())

Move_coffee_machine_offset_and_above = rdk.Mat(Move_coffee_machine_offset_and_above_np.tolist())

Move_coffee_machine_center_and_above = rdk.Mat(Move_coffee_machine_center_and_above_np.tolist())

###################################################################################################
""" Intermediate points. """
###################################################################################################

J_cup_tool_orient = [-177.697218, -59.469365, -88.212296, -122.881182, 90.022633, -202.697330]

J_cup_intermediate_point = [-74.210000, -55.860000, -90.220000, -213.920000, 0.000000, -40.000000]

J_cup_intermediate_point_2 = [-76.740000, -102.140000, -81.270000, -356.590000, 0.000000, -40.000000]

J_cup_intermediate_point_3 =[1.592416, -118.102180, -219.675634, 337.777813, 1.592416, -40.000000]

J_cup_to_coffee_machine_intermediate_point = [95.050000, -105.730000, 119.070000, -13.330000, 109.290000, -220.000000]

J_back_out_point = [81.467310, -107.138159, 95.762790, 11.375369, 156.427310, -220.000000]

#####################################################################################################################################################


def cup_complete():
    ###################################################################################################
    """ Scheduler. """
    ###################################################################################################
    #robot.MoveJ(T_cup_base, blocking=True)
    #robot.MoveJ(cup_frame, blocking=True)
    #robot.MoveJ(oriented_frame, blocking=True)
    #robot.MoveJ(J_cup_tool_orient, blocking=True)
    #robot.MoveJ(oriented_frame, blocking=True)
    #RDK.RunProgram("Cup Tool Attach (Stand)", True)
    #robot.setPoseTool(master_tool)
    #robot.MoveJ(J_cup_tool_orient, blocking=True)
    #robot.MoveJ(T_cup_base, blocking=True)
    #robot.MoveJ(J_cup_intermediate_point, blocking=True)
    #robot.MoveJ(cup_frame, blocking=True)
    #robot.MoveJ(J_cup_intermediate_point, blocking=True)
    #robot.MoveJ(oriented_frame, blocking=True)
    #robot.MoveJ(J_cup_intermediate_point, blocking=True)
    #robot.MoveJ(J_cup_intermediate_point_2, blocking=True)
    #robot.MoveJ(J_cup_intermediate_point_2, blocking=True)
    #robot.MoveJ(Align_cup, blocking=True)
    #RDK.RunProgram("Cup Tool Open", True)
    #robot.MoveJ(Grabber_cup_centre, blocking=True)
    #RDK.RunProgram("Cup Tool Close", True)
    #robot.MoveJ(Align_cup, blocking=True)
    #robot.MoveJ(J_cup_intermediate_point, blocking=True)
    #robot.MoveJ(J_cup_to_coffee_machine_intermediate_point, blocking=True)
    #robot.MoveJ(coffee_machine_frame, blocking=True)
    #robot.MoveJ(coffee_machine_orient_frame, blocking=True)
    #robot.MoveJ(Align_coffee_machine, blocking=True)
    #RDK.RunProgram("Cup Tool Open", True)

    #robot.MoveJ(Center_coffee_machine, blocking=True)

    #robot.MoveJ(Move_coffee_machine_offset_and_above, blocking=True)

    #robot.MoveJ(Move_coffee_machine_center_and_above, blocking=True)

    #robot.MoveJ(J_back_out_point, blocking=True)

    #robot.MoveJ(J_cup_to_coffee_machine_intermediate_point, blocking=True)


    #robot.MoveJ(frame_base, blocking=True)

    #RDK.RunProgram("Cup Tool Detach (Stand)", True)
    #RDK.RunProgram("Cup Tool Attach (Stand)", True)


    ###################################################################################################
    """ Good content. """
    ###################################################################################################
    robot.MoveJ(J_cup_tool_orient, blocking=True)

    RDK.RunProgram("Cup Tool Attach (Stand)", True)

    robot.setPoseTool(master_tool)

    robot.MoveJ(J_cup_tool_orient, blocking=True)

    robot.MoveJ(J_cup_intermediate_point, blocking=True)

    robot.MoveJ(J_cup_to_coffee_machine_intermediate_point, blocking=True)

    robot.MoveJ(Align_coffee_machine, blocking=True)

    RDK.RunProgram("Cup Tool Open", True)

    robot.MoveJ(Center_coffee_machine, blocking=True)

    RDK.RunProgram("Cup Tool Close", True)

    robot.MoveJ(Align_coffee_machine, blocking=True)

    robot.MoveJ(Move_coffee_machine_offset_and_above, blocking=True)

    robot.MoveJ(Move_coffee_machine_center_and_above, blocking=True)

    RDK.RunProgram("Cup Tool Open", True)

    robot.MoveJ(J_back_out_point, blocking=True)

    robot.MoveJ(J_cup_to_coffee_machine_intermediate_point, blocking=True)

    robot.MoveJ(J_cup_intermediate_point, blocking=True)

    robot.MoveJ(J_cup_tool_orient, blocking=True)

    RDK.RunProgram("Cup Tool Close", True)

    RDK.RunProgram("Cup Tool Detach (Stand)", True)

    robot.setPoseTool(master_tool)

    robot.MoveJ(J_cup_tool_orient, blocking=True)







def main():
    """ Main Function. """

    cup_collect()

    coffee_machine_buttons()

    cup_complete()

main()