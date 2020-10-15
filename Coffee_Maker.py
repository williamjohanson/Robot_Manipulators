#####################################################################################################################################################
#
# William Johanson 12015205
# Liam Hare 27686710
# ENMT482
# Robot Manipulators
#
# 13/10/2020
#####################################################################################################################################################


###################################################################################################
""" READ ME: This code is set up into two sections. The first sections handles all of the tasks from 1-6 as defined in the introduction of the report and the
second section handles the tasks from 7-10. In Section 1 to help with the accronys, there is a Key at the start. The acronyms shouls be read where T = Transform
followed by the parent and then the child frame. 

"""

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
""" SECTION 1 """
#####################################################################################################################################################
"""LEGEND to help desypher the acronyms"""
# T   = Transform
# RB  = Robot Base Frame

# Tool frames
# TCP = Master tool center point with no additional tools added
# PFA = Portafilter angle change of 50 degrees
# PFB = Portafilter Ball connection point 
# PFF = The center of the Portafilter filter part
# GTP = Grinder Tool Pusher frame
# GTL = Grinder Tool Lever frame

# Equipment Frames
# GR  = Grinder base frame
# PFP = Portafilter Placement frame under the grinder
# GB  = Grinder Buttons Frame
# GL  = Grinder Lever Frame
# TBF = Tamper Base Frame
# SC  = Tamper Stand Scraper Frame
# TA  = Tamper Frame


#####################################################################################################################################################
""" TRANSFORMING TO THE PROVIDED TOOLS FROM THE CURRENT FRAME  """
#####################################################################################################################################################
""" Transform to Portafilter Ball Connection """
###################################################################################################
# First need to spin TCP 50degrees using following transform
theta_TCP_PFA = np.radians(50) #change 50 degree angle before moving 7.5 degrees PFA portafilter angle
T_TCP_PFA_np = np.array([[ np.cos(theta_TCP_PFA), -np.sin(theta_TCP_PFA),  0.00,   0.00],
                         [ np.sin(theta_TCP_PFA),  np.cos(theta_TCP_PFA),  0.00,   0.00],
                         [                  0.00,                   0.00,  1.00,   0.00],
                         [                  0.00,                   0.00,  0.00,   1.00]])

# Tranforms from the rotated TCP to the portafilter ball connection (PFB) point
theta_T_TCP_PFB = np.radians(7.5) #Tilt 7.5 degrees
T_TCP_PFB_np = np.array([[ np.cos(theta_T_TCP_PFB ),  0.00, np.sin(theta_T_TCP_PFB ),  32.00],
                         [                     0.00,  1.00,                     0.00,   0.00],
                         [-np.sin(theta_T_TCP_PFB ),  0.00, np.cos(theta_T_TCP_PFB ), -27.56],
                         [                     0.00,  0.00,                     0.00,   1.00]])

#Transform from TCP to the center of the potafilter part that goes into the Tamper 
theta_TCP_PFF = np.radians(7.5)  
T_TCP_PFF_np = np.array([[ np.cos(theta_TCP_PFF),   0.000000,   np.sin(theta_TCP_PFF),      -4.71],
                         [             0.000000,   1.000000,               0.000000,       0.00],
                         [-np.sin(theta_TCP_PFF),   0.000000,   np.cos(theta_TCP_PFF),    -142.76],
                         [             0.000000,   0.000000,               0.000000,       1.00]])

T_TCP_PFA = rdk.Mat(T_TCP_PFA_np.tolist())

T_TCP_PFB = rdk.Mat(T_TCP_PFB_np.tolist())
T_TCP_PFB = T_TCP_PFB * T_TCP_PFA #Resulting transform from the TCP to the PFB

T_TCP_PF = rdk.Mat(T_TCP_PF_np.tolist())
T_TCP_PFF = T_TCP_PFF* T_TCP_PFA #Resulting transform from the TCP to the PFF

###################################################################################################
""" Transform to the Grinder Tool Button Pusher (GTP) and then to the Grinder Tool Lever (GTL) """
###################################################################################################
#Transform from TCP to the grinder tool button pusher (GTP)
theta_TCP_GTP = np.radians(50)
T_TCP_GTP_np = np.array([[ np.cos(theta_TCP_GTP), -np.sin(theta_TCP_GTP),  0.00,    0.00],
                         [ np.sin(theta_TCP_GTP),  np.cos(theta_TCP_GTP),  0.00,    0.00],
                         [                 0.00,                  0.00,  1.00, -102.82],
                         [                 0.00,                  0.00,  0.00,    1.00]])

#Transform to rotate TCP 50 degrees to the grinder tool angle (GTA)
theta_TCP_GTL = np.radians(50)
T_TCP_GTA_np = np.array([[ np.cos(theta_TCP_GTL), -np.sin(theta_TCP_GTL),  0.00,   0.00],
                         [ np.sin(theta_TCP_GTL),  np.cos(theta_TCP_GTL),  0.00,   0.00],
                         [                 0.00,                  0.00,  1.00,   0.00],
                         [                 0.00,                  0.00,  0.00,   1.00]])

#Transform from grinder tool angle (GTA) to lever frame on the tool and move it to the start postiton
T_GTA_GTL_np = np.array([[  0.000000, 0.000000, -1.000000,    67.06],
                        [  0.000000,-1.000000,  0.000000,  0.00000],
                        [ -1.000000, 0.000000,  0.000000,   -40.00],
                        [  0.000000, 0.000000,  0.000000, 1.000000]])

T_TCP_GTP = rdk.Mat(T_TCP_GTP_np.tolist())
T_TCP_GTA = rdk.Mat(T_TCP_GTA_np.tolist())
T_GTA_GTL = rdk.Mat(T_GTA_GTL_np.tolist()) 

#####################################################################################################################################################
""" Grinder Transforms """
###################################################################################################
#Transform from robot base frame to the grinder machine base frame
theta_RB_GR = np.radians(44.79)
T_RB_GR_np = np.array([[ -np.cos(theta_RB_GR),  -np.sin(theta_RB_GR),  0.000000,   482.29],
                       [  np.sin(theta_RB_GR),  -np.cos(theta_RB_GR),  0.000000,  -433.74],
                       [             0.000000,              0.000000,  1.000000,   314.13],
                       [             0.000000,              0.000000,  0.000000, 1.000000]])
  
#Transform from the grinder machine to the button frame
theta_GR_GB = np.radians(164.75)
T_GR_GB_np = np.array([[  np.cos(theta_GR_GB),  0.000000, -np.sin(theta_GR_GB),    -80.71],
                       [  np.sin(theta_GR_GB),  0.000000,  np.cos(theta_GR_GB),     94.26],
                       [             0.000000,  -1.00000,             0.000000,   -227.68],
                       [             0.000000,  0.000000,             0.000000,  1.000000]])

#Transform from the Grinder Machine to the Lever frame
theta_GR_GL = np.radians(10)
T_GR_GL_np = np.array([[ np.cos(theta_GR_GL),  0.000000,  np.sin(theta_GR_GL),    -35.82],
                       [  np.sin(theta_GR_GL), 0.000000, -np.cos(theta_GR_GL),     83.80],
                       [             0.000000,  1.00000,             0.000000,   -153.00],
                       [             0.000000, 0.000000,             0.000000, 1.000000]])

#Transfrom from grinder machine to the Portafilter placement position
T_GR_PFP_np = np.array([[ 0.000000, 0.000000, -1.000000,   157.61],
                        [ 0.000000, 1.000000,  0.000000,  0.00000],
                        [ 1.000000, 0.000000,  0.000000,  -250.45],
                        [ 0.000000, 0.000000,  0.000000, 1.000000]])

#NP to RDK.MAT conversions
T_RB_GR  = rdk.Mat(T_RB_GR_np.tolist())
T_GR_GB = rdk.Mat(T_GR_GB_np.tolist())
T_GR_GL = rdk.Mat(T_GR_GL_np.tolist())
T_GR_PFP = rdk.Mat(T_GR_PFP_np.tolist())

###################################################################################################
""" Tamper Stand Transforms"""
###################################################################################################
#Transform from the robot base frame (RB) to the tamper stand base frame (TBF)
theta_RB_TBF = np.radians(59.73)
T_RB_TBF_np = np.array([[  np.cos(theta_RBA_TBF),  np.sin(theta_RBA_TBF),  0.000000, 599.13],
                        [ -np.sin(theta_RBA_TBF),  np.cos(theta_RBA_TBF),  0.000000,    0.0],
                        [               0.000000,               0.000000,  1.000000, 211.07],
                        [               0.000000,               0.000000,  0.000000,   1.00]])

#Transform from tamper stand base frame (TBF) to the centre of the tamper frame (TA)
T_TBF_TA_np = np.array([[ 0.00, 1.00,  0.00,  -80],
                        [ 0.00, 0.00,  1.00,  0.0],
                        [ 1.00, 0.00,  0.00,  -55],
                        [ 0.00, 0.00,  0.00, 1.00]])

#Transform from tamper stand base frame (TBF) to the scraper centre (SC)
T_TBF_SC_np = np.array([[ 0.00, 1.00,  0.00,   70],
                        [ 0.00, 0.00,  1.00,  0.0],
                        [ 1.00, 0.00,  0.00,  -32],
                        [ 0.00, 0.00,  0.00, 1.00]])

#NP to RDK.MAT conversions                 
T_RB_TBF = rdk.Mat(T_RB_TA_np.tolist()) #transform from base to Tamper stand
T_TBF_TA = rdk.Mat(T_TBF_TA_np.tolist())
T_TBF_SC = rdk.Mat(T_TBF_SC_np.tolist()) #Transform from tamper stand to SC frame


#####################################################################################################################################################
""" LET THE FUNCTIONS BEGIN """
#####################################################################################################################################################
def portafilterPlacement():
   """ This function performs task 1 and places the portafilter under the grinder """
   
   #Calculate placement position for portafilter under the grinder
   T_RB_PFP = T_RB_GR *T_GR_PFP #Waht we multiple by before the adjustment
   
   
   T_TCP_PFPPreset = T_RB_PFP * rdk.transl(0,-1,-60)* rdk.roty(np.radians(-2)) * T_TCP_PFB
   T_TCP_PFPSet = T_RB_PFP * rdk.transl(-3,-1,-3)* rdk.roty(np.radians(-1)) * T_TCP_PFB

   #Steps from tools tand to Grinder
   J_Step1 = [-156.440000, -81.180000, -75.390000, -181.780000, 181.780000, -182.170000]
   J_Step2 = [-156.440000, -81.180000, -127.130000, -181.780000, 181.780000, -182.170000]
   J_Step3 = [-71.050000, -81.180000, -127.130000, -136.630000, 181.780000, -182.170000]
   J_Step4 = [-14.210000, -65.350000, -148.510000, -148.510000, 293.470000, -231.680000]
   J_Step5 = [-17.818290, -100.202192, -139.887909, -111.506940, 296.722534, -223.800304]

   RDK.RunProgram('Portafilter Tool Attach (Stand)', True)
   robot.MoveJ(J_Step1, blocking=True)
   robot.MoveJ(J_Step2, blocking=True)
   robot.MoveJ(J_Step3, blocking=True)
   robot.MoveJ(J_Step4, blocking=True) 

   robot.MoveL(T_TCP_PFPPreset, blocking=True)#placement position
   robot.MoveL(T_TCP_PFPSet, blocking=True)#placement position
   RDK.RunProgram('Portafilter Tool Detach (Grinder)', True)

###################################################################################################
def pushButtonsOnGrinder(): 
   """ This function performs task to by fetching the grinder tool and turning the grinder on for 3s """
   #Steps to go from coffee grinder to toolstand
   J_GR_TS_Step1 = [-10.984030, -94.722451, -149.513590, -106.760819, 303.478379, -224.994922]
   J_GR_TS_Step2 = [-42.630000, -94.720000, -149.510000, -106.760000, 303.470000, -224.990000]
   J_GR_TS_Step3 = [-61.580000, -94.720000, -41.580000, -106.760000, 360.000000, -224.990000]
   J_GR_TS_Step4 = [-118.420000, -94.720000, -41.580000, -106.760000, 84.360000, -224.990000]

   robot.MoveJ(J_GR_TS_Step1, blocking=True)
   robot.MoveJ(J_GR_TS_Step2, blocking=True)
   robot.MoveJ(J_GR_TS_Step3, blocking=True)   
   robot.MoveJ(J_GR_TS_Step4, blocking=True)  
   
   #go and grab the Grinder tool
   RDK.RunProgram('Grinder Tool Attach (Stand)', True)
   
   #Steps to go from the tool stand to the coffee grinder
   J_TS_GR_Step1 = [-78.160000, -93.120000, -64.110000, -112.910000, 89.940000, -167.400000]
   J_TS_GR_Step2 = [-11.840000, -93.120000, -64.110000, -112.910000, 89.940000, -167.400000]
   J_TS_GR_Step3 = [11.840000, -93.120000, 67.720000, -112.910000, 89.940000, -167.400000]
   J_TS_GR_Step4 = [80.530000, -93.120000, 112.870000, -112.910000, 89.940000, -167.400000]
   J_TS_GR_Step5 = [87.630000, -81.980000, 112.860000, -112.910000, 3.560000, -167.400000]
   
   robot.MoveJ(J_TS_GR_Step1, blocking=True)
   robot.MoveJ(J_TS_GR_Step2, blocking=True)
   robot.MoveJ(J_TS_GR_Step3, blocking=True)   
   robot.MoveJ(J_TS_GR_Step4, blocking=True)   
   robot.MoveJ(J_TS_GR_Step5, blocking=True)
   
   #Calculate position of button
   T_RB_GB = T_RB_GR * T_GR_GB * T_TCP_GTP * rdk.rotz(np.radians(40)) #Had to move 40 degrees to make the y adjustment parrallel to floor
   
   # Calculate useful positions for the button pushes
   T_TCP_GB1Preset = T_RB_GB * rdk.transl(0,0,-20)
   T_TCP_GB1Push = T_RB_GB * rdk.transl(0,0,10)
   T_TCP_GB2Preset = T_RB_GB * rdk.transl(0,17,-30)
   T_TCP_GB2Push = T_RB_GB * rdk.transl(0,17,10)

   robot.MoveJ(T_TCP_GB2Preset, blocking=True)
   robot.MoveL(T_TCP_GB2Push, blocking=True)
   robot.MoveL(T_TCP_GB2Preset, blocking=True)
   
   rdk.pause(3) #Let the grinder run for 3 seconds  
   
   robot.MoveL(T_TCP_GB1Preset, blocking=True)
   robot.MoveL(T_TCP_GB1Push, blocking=True)
   robot.MoveL(T_TCP_GB1Preset, blocking=True)

###################################################################################################
def pullLever():
   """ This function performs task 3 by pulling dosing lever on the grinder 3 times """
   
   #Calculate the position of the lever
   T_TCP_GTL = T_RB_GR * T_GR_GL * T_GTA_GTL

   J_TCP_GTLPreset    = [107.180244, -60.445471, 91.787676, -31.342205, 51.970244, -130.000000] 
   T_TCP_GTLStart     = T_TCP_GTL * T_TCP_GTA # All of the other positions calculated relative to start
   T_TCP_GTLPull1     = T_TCP_GTL * T_TCP_GTA * rdk.transl(0,0,-50) # This is the value I need to change to increase pulling position
   T_TCP_GTLPull_Turn = T_TCP_GTL * rdk.roty(np.radians(15)) * rdk.transl(0,0,-65) * T_TCP_GTA
   T_TCP_GTLPull2     = T_TCP_GTL * rdk.roty(np.radians(15)) * rdk.transl(-20, 0,-110) * T_TCP_GTA #CHANGE LAST VALUE TO MAKE IT MOVE FURTHER  

   itterations = 3 # Number of times the grinder lever is pulled
   robot.MoveJ(J_TCP_GTLPreset, blocking=True)  
   
   for i in range(itterations):
      robot.MoveL(T_TCP_GTLStart, blocking=True)
      robot.MoveL(T_TCP_GTLPull1, blocking=True)
      robot.MoveL(T_TCP_GTLPull_Turn, blocking=True)
      robot.MoveL(T_TCP_GTLPull2, blocking=True)
      robot.MoveL(T_TCP_GTLPull_Turn, blocking=True)
      robot.MoveL(T_TCP_GTLPull1, blocking=True)
      robot.MoveL(T_TCP_GTLStart, blocking=True)

   robot.MoveL(J_TCP_GTLPreset, blocking=True)
   
   # Steps to move from the grinder to the tool stand to drom the grinder tool off
   J_GR_TS_Step1 = [99.470000, -60.440000, 91.780000, -31.340000, 51.970000, -129.990000]
   J_GR_TS_Step2 = [87.630000, -81.980000, 112.860000, -112.910000, 3.560000, -167.400000] 
   J_GR_TS_Step3 = [80.530000, -93.120000, 112.870000, -112.910000, 89.940000, -167.400000]
   J_GR_TS_Step4 = [11.840000, -93.120000, 67.720000, -112.910000, 89.940000, -167.400000]
   J_GR_TS_Step5 = [-11.840000, -93.120000, -64.110000, -112.910000, 89.940000, -167.400000]
   J_GR_TS_Step6 = [-78.160000, -93.120000, -64.110000, -112.910000, 89.940000, -167.400000]
   
   # Robot moves to the tool stand and drops the grinder tool off
   robot.MoveJ(J_GR_TS_Step1, blocking=True)
   robot.MoveJ(J_GR_TS_Step2, blocking=True)
   robot.MoveJ(J_GR_TS_Step3, blocking=True)   
   robot.MoveJ(J_GR_TS_Step4, blocking=True) 
   robot.MoveJ(J_GR_TS_Step5, blocking=True)
   robot.MoveJ(J_GR_TS_Step6, blocking=True) 
   
   RDK.RunProgram('Grinder Tool Detach (Stand)', True)

###################################################################################################
   
def fetchFullPortafilter():  
   """ This function is an intermediate function to grab the portafilter from under the grinder
   and position it near the tamper stand """
   
   # Steps to go from the tool stand to the grinder (same points used in portaplacement)
   J_TS_GR_Step1 = [-156.440000, -81.180000, -75.390000, -181.780000, 181.780000, -182.170000]
   J_TS_GR_Step2 = [-156.440000, -81.180000, -127.130000, -181.780000, 181.780000, -182.170000]
   J_TS_GR_Step3 = [-71.050000, -81.180000, -127.130000, -136.630000, 181.780000, -182.170000]
   J_TS_GR_Step4 = [-14.210000, -65.350000, -148.510000, -148.510000, 293.470000, -231.680000]
   J_TS_GR_Step5 = [-17.818290, -100.202192, -139.887909, -111.506940, 296.722534, -223.800304]   

   # Go and fetch the portafilter
   robot.MoveJ(J_TS_GR_Step1, blocking=True)
   robot.MoveJ(J_TS_GR_Step2, blocking=True)
   robot.MoveJ(J_TS_GR_Step3, blocking=True)   
   robot.MoveJ(J_TS_GR_Step4, blocking=True)   
   robot.MoveJ(J_TS_GR_Step5, blocking=True)   
   
   RDK.RunProgram('Portafilter Tool Attach (Grinder)', True)
   
   # Steps to take the portafilter from the grinder to near the tamper stand
   J_GR_TA_Step1 = [-15.011332, -97.290411, -145.804176, -108.512264, 297.551540, -224.181867]
   J_GR_TA_Step2 = [2.370000, -97.290000, -145.800000, -108.510000, 297.550000, -224.180000]
   J_GR_TA_Step3 = [-21.320000, -103.370000, -141.390000, -100.990000, 230.680000, -212.670000]
   
   # Move towards the tamper stand
   robot.MoveJ(J_GR_TA_Step1, blocking=True)
   robot.MoveJ(J_GR_TA_Step2, blocking=True)
   robot.MoveJ(J_GR_TA_Step3, blocking=True)
   
###################################################################################################
   
def tamperScraper(): 
   """ This function performs task 4 and scrapes the coffee off the rim of the portafilter """
   
   T_RB_PFSC  = T_RB_TBF * T_TBF_SC    #This defines the frame for the scraper part of the tamper stand
   
   # Define the points to move between relative to frame of the scraper
   T_RB_SCPreset = T_RB_PFSC * rdk.transl(-30,0,-100)     * T_TCP_PFT_F  
   T_RB_SCPush   = T_RB_PFSC * rdk.transl(-30,0,50)       * T_TCP_PFT_F      
   T_RB_SCDrop   = T_RB_PFSC * rdk.transl(-100,0,-100)    * T_TCP_PFT_F    
   T_RB_SCSlide  = T_RB_PFSC * rdk.transl(-100,-150,-100) * T_TCP_PFT_F    
   
   # Move portafilter back and forth along scraper to remove excess coffee
   robot.MoveJ(T_RB_SCPreset, blocking=True)
   robot.MoveL(T_RB_SCPush, blocking=True)
   robot.MoveL(T_RB_SCPreset, blocking=True)
   
   # Now move the portafiler near the tamper tool
   robot.MoveJ(T_RB_SCDrop, blocking=True)
   robot.MoveJ(T_RB_SCSlide, blocking=True)
   
###################################################################################################
   
def tamperPusher():
   """ This function performs task 5 and tamps the coffee """
   
   T_RB_PFTA = T_RB_TBF * T_TBF_TA   #This defines the frame for the tamper tool
   
   # Define the points to move up and down relative to the tamper frame where x points vertically
   T_RB_TAPreset = T_RB_PFTA * rdk.transl(-80,0,0) * T_TCP_PFF 
   T_RB_TAPush   = T_RB_PFTA * rdk.transl(-30,0,0) * T_TCP_PFF  
   
   # Condense the coffee in the portafilter
   robot.MoveL(T_RB_TAPreset, blocking=True)
   robot.MoveL(T_RB_TAPush, blocking=True)
   robot.MoveL(T_RB_TAPreset, blocking=True)
   
   J_Clear_Tamper = [-31.658996, -93.568750, -155.250750, -95.550201, 208.976750, -206.247112]
   robot.MoveJ(J_Clear_Tamper, blocking=True)
   
################################################################################################### 
   
def passPortafilter():
   """ This functions moves the portafilter towards the coffee machine and pauses for 12seconds at a handoff location """
   
   # Steps to take the fully loaded portafiler near the cups to give to the human
   J_TA_HU_Step1 = [-66.320000, -93.570000, -155.250000, -95.550000, 208.970000, -206.240000]
   J_TA_HU_Step2 = [-66.320000, -93.570000, -155.250000, -103.370000, 260.200000, -217.430000]
   J_TA_HU_Step3 = [-66.320000, -93.570000, -134.260000, -122.380000, 260.200000, -217.430000]
   
   # The portafilter is given to the human
   robot.MoveJ(J_TA_HU_Step1, blocking=True)
   robot.MoveJ(J_TA_HU_Step2, blocking=True)
   robot.MoveJ(J_TA_HU_Step3, blocking=True)
   
   rdk.pause(12) # Give the Human 12s to put the portafilter in the coffee machine

     
#####################################################################################################################################################
""" SECTION 2 """
#####################################################################################################################################################
""" Coffee machine buttons global variables. """
###################################################################################################
###################################################################################################
""" Coffee Machine Transforms """
###################################################################################################

# Orientate frame to get the grabber to match cups.
T_coffee_button_frame_np = np.array([[ 0.000000, 0.000000, 1.000000, 0.000000],
                                     [ 0.000000, 1.000000, 0.000000, 0.000000],
                                     [-1.000000, 0.000000, 0.000000, 0.000000],
                                     [ 0.000000, 0.000000, 0.000000, 1.000000]])

# Set the base frame of the coffee machine.
T_coffee_machine_base_np = np.array([[ np.cos(np.radians(74.96)), np.sin(np.radians(74.96)), 0.000000, -366.200000],
                                     [-np.sin(np.radians(74.96)), np.cos(np.radians(74.96)), 0.000000, -389.800000],
                                     [                  0.000000,                  0.000000, 1.000000,  341.380000],
                                     [                  0.000000,                  0.000000, 0.000000,    1.000000]])

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
button_center_np = np.array([[0.000000, 0.000000, 0.000000,   25.250000],
                             [0.000000, 0.000000, 0.000000,  -30.000000],
                             [0.000000, 0.000000, 0.000000, -170.000000],
                             [0.000000, 0.000000, 0.000000,    0.000000]])   

button_top_np = np.array([[0.000000, 0.000000, 0.000000,   10.250000],
                          [0.000000, 0.000000, 0.000000,  -30.000000],
                          [0.000000, 0.000000, 0.000000, -152.000000],
                          [0.000000, 0.000000, 0.000000,    0.000000]])         

button_bottom_np = np.array([[0.000000, 0.000000, 0.000000,   30.250000],
                             [0.000000, 0.000000, 0.000000,  -30.000000],
                             [0.000000, 0.000000, 0.000000, -152.000000],
                             [0.000000, 0.000000, 0.000000,    0.000000]])                                                                                                

#Add the offsets to the base frame of coffee machine
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
J_intermediateGrinderTool = [-157.010000, -67.940000, -79.440000, -157.500000, 177.770000, -208.440000]
#####################################################################################################################################################


def coffee_machine_buttons():
    """ Coffee machine buttons to turn the coffee machine on to pour the coffee  7 seconds . """

    robot.setPoseTool(master_tool)

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
coffee_machine_offset_np = np.array([[0.000000, 0.000000, 0.000000,   -7.000000],
                                     [0.000000, 0.000000, 0.000000,  -34.000000],
                                     [0.000000, 0.000000, 0.000000, -250.000000],
                                     [0.000000, 0.000000, 0.000000,    0.000000]])      

# Center cup under portafilter
coffee_machine_center_np = np.array([[0.000000, 0.000000, 0.000000,   -7.000000],
                                     [0.000000, 0.000000, 0.000000,  -34.000000],
                                     [0.000000, 0.000000, 0.000000, -120.000000],
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
T_cup_base               = rdk.Mat(T_cup_base_np.tolist())
T_cup_grabber_frame      = rdk.Mat(T_cup_grabber_frame_np.tolist())
cup_frame                = rdk.Mat(cup_frame_np.tolist())
T_cup_angle              = rdk.Mat(T_cup_angle_np.tolist())
oriented_frame           = rdk.Mat(oriented_frame_np.tolist())
Align_cup                = rdk.Mat(Align_cup_np.tolist())
Grabber_cup_centre       = rdk.Mat(Grabber_cup_centre_np.tolist())
coffee_machine_frame     = rdk.Mat(coffee_machine_frame_np.tolist())
coffee_machine_orient_frame = rdk.Mat(coffee_machine_orient_frame_np.tolist())
Align_coffee_machine     = rdk.Mat(Align_coffee_machine_np.tolist())
Center_coffee_machine    = rdk.Mat(Center_coffee_machine_np.tolist())
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
coffee_machine_offset_np = np.array([[0.000000, 0.000000, 0.000000,  -7.000000],
                                     [0.000000, 0.000000, 0.000000, -34.000000],
                                     [0.000000, 0.000000, 0.000000, -250.000000],
                                     [0.000000, 0.000000, 0.000000,    0.000000]])      

# Center cup under portafilter
coffee_machine_center_np = np.array([[0.000000, 0.000000, 0.000000,  -7.000000],
                                     [0.000000, 0.000000, 0.000000, -34.000000],
                                     [0.000000, 0.000000, 0.000000, -120.000000],
                                     [0.000000, 0.000000, 0.000000,    0.000000]])     

# Shift cup up to top of coffee machine
coffee_machine_offset_and_above_np = np.array([[0.000000, 0.000000, 0.000000,    0.000000],
                                               [0.000000, 0.000000, 0.000000,  -380.000000],
                                               [0.000000, 0.000000, 0.000000, -300.000000],
                                               [0.000000, 0.000000, 0.000000,    0.000000]])      

# Center cup above coffee machine
coffee_machine_center_and_above_np = np.array([[0.000000, 0.000000, 0.000000,    0.000000],
                                               [0.000000, 0.000000, 0.000000,  -380.000000],
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
T_cup_base                           = rdk.Mat(T_cup_base_np.tolist())
T_cup_grabber_frame                  = rdk.Mat(T_cup_grabber_frame_np.tolist())
cup_frame                            = rdk.Mat(cup_frame_np.tolist())
T_cup_angle                          = rdk.Mat(T_cup_angle_np.tolist())
oriented_frame                       = rdk.Mat(oriented_frame_np.tolist())
Align_cup                            = rdk.Mat(Align_cup_np.tolist())
Grabber_cup_centre                   = rdk.Mat(Grabber_cup_centre_np.tolist())
coffee_machine_frame                 = rdk.Mat(coffee_machine_frame_np.tolist())
coffee_machine_orient_frame          = rdk.Mat(coffee_machine_orient_frame_np.tolist())
Align_coffee_machine                 = rdk.Mat(Align_coffee_machine_np.tolist())
Center_coffee_machine                = rdk.Mat(Center_coffee_machine_np.tolist())
Grabber_above_cup_centre             = rdk.Mat(Grabber_above_cup_centre_np.tolist())
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
    

def main():
    """ Main Function. """
    portafilterPlacement()
    pushButtonsOnGrinder()
    pullLever()
    fetchFullPortafilter()
    tamperScraper()
    tamperPusher()
    passPortafilter()
    cup_collect()
    coffee_machine_buttons()
    cup_complete()

main()