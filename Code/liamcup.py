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

T_TCP_cup_base_np = np.array([[-1.000000, 0.000000,  0.000000,    1.490000],
                          [0.000000, -1.000000,  0.000000, -600.540000],
                          [0.000000, 0.000000, 1.000000,  -20.000000],
                          [0.000000, 0.000000,  0.000000,    1.000000]])

T_cup_base_grabberFrame_np = np.array([[1.000000, 0.000000,  0.000000,   0.0000],
                                       [0.000000, 0.000000,  1.000000, 0.0000],
                                       [0.000000, -1.000000, 0.000000,  0.000000],
                                      [0.000000, 0.000000,  0.000000,    1.000000]])    

T_cup_base_grabberFrame = rdk.Mat(T_cup_base_grabberFrame_np.tolist())

theta_TCP_CT = np.radians(40)
T_cup_angle_np =  np.array([[np.cos(theta_TCP_CT),  np.sin(theta_TCP_CT),  0.000000, 00.000000],
                            [-np.sin(theta_TCP_CT), np.cos(theta_TCP_CT), 0.0000, -47.000000],
                            [             0.000000, 0.000000, 1.000000,    -186.11],
                            [              0.000000,                0.000000,        0.000000, 1.000000]])            


T_TCP_GrabberFrame_np = np.matmul(np.matmul(T_TCP_cup_base_np, T_cup_base_grabberFrame_np), T_cup_angle_np) 
T_TCP_cup_base = rdk.Mat(T_TCP_cup_base_np.tolist())
T_TCP_GrabberFrame = rdk.Mat(T_TCP_GrabberFrame_np.tolist())
#RDK.RunProgram("Cup Tool Detach (Stand)", True)
#RDK.RunProgram("Cup Tool Attach (Stand)", True)
robot.MoveJ(T_TCP_cup_base, blocking=True)
robot.MoveJ(T_TCP_GrabberFrame, blocking=True)
#robot.MoveJ(T_cup_base_grabberFrame, blocking=True)