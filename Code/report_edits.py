#####################################################################################################################################################
#
# William Johanson Student ID
# Liam Hare 27686710
# ENMT482
# Robot Manipulators
#
# 13/10/2020
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

#LEGEND
# T   = Transform
# TCP = Master tool center point with no additional tools added
# TA  = Tamper Stand base fram 
# PFA = Portafilter angle change of 50 degrees
# PFT = Portafilter tilt change of 7.5 degrees
# GR  = Grinder base frame
# GB  = Brinder Button frame
# GL  = Grinder Lever frame
# PFP = Portafilter Placement frame under the grinder


# Directly use the RDK Matrix object from to hold pose (its an HT)
T_home = rdk.Mat([[ 0.000000,     0.000000,     1.000000,   523.370000 ],
                  [-1.000000,     0.000000,     0.000000,  -109.000000 ],
                  [-0.000000,    -1.000000,     0.000000,   607.850000 ],
                  [ 0.000000,     0.000000,     0.000000,     1.000000 ]])



#####################################################################################################################################################
# Transform to Portafilter Tool
# First need to spin TCP 50degrees using following transform

theta_TCP_PFA = np.radians(50) #change 50 degree angle before moving 7.5 degrees PFA portafilter angle
T_TCP_PFA_np = np.array([[ np.cos(theta_TCP_PFA), -np.sin(theta_TCP_PFA), 0.00,    0.00],
                     [ np.sin(theta_TCP_PFA),  np.cos(theta_TCP_PFA), 0.00,    0.00],
                     [                   0.00,                    0.00, 1.00,    0.00],
                     [                   0.00,                    0.00, 0.00,    1.00]])
T_TCP_PFA = rdk.Mat(T_TCP_PFA_np.tolist())

theta_T_TCP_PFT = np.radians(7.5) #PFT portafilter tilt for placing the portafilter on the ball
T_TCP_PFT_np = np.array([[np.cos(theta_T_TCP_PFT ), 0.00, np.sin(theta_T_TCP_PFT ), 32.00],
                         [    0.00, 1.00,   0.00, 0.00],
                         [-np.sin(theta_T_TCP_PFT ), 0.00,  np.cos(theta_T_TCP_PFT ), -27.56],
                         [    0.00, 0.00,   0.00,    1.00]])

T_CL_PFT_np = np.matmul(T_TCP_PFT_np, T_TCP_PFA_np) 
#####################################################################################################################################################
# Grinder Matrices

theta_grinder_machine = np.radians(44.79)
T_TCP_GR_np = np.array([[ -np.cos(theta_grinder_machine), -np.sin(theta_grinder_machine),  0.000000,  482.29],
                             [ np.sin(theta_grinder_machine),  -np.cos(theta_grinder_machine),  0.000000,  -433.74],
                             [                      0.000000,                       0.000000,  1.000000,   314.13],
                             [                      0.000000,                       0.000000,  0.000000, 1.000000]])

#Transform from the Grinder machine to the botton push frame
theta_GR_GB = np.radians(164.75)
T_GR_GB_np = np.array([[ np.cos(theta_GR_GB), 0.000000, -np.sin(theta_GR_GB),    -80.71],
                        [  np.sin(theta_GR_GB), 0.000000, np.cos(theta_GR_GB),     94.26],
                        [             0.000000, -1.00000,             0.000000,   -227.68],
                        [             0.000000, 0.000000,             0.000000, 1.000000]])

#Transform from the Grinder Machine tot he Lever frame
theta_GR_GL = np.radians(10)
T_GR_GL_np = np.array([[ np.cos(theta_GR_GL),  0.000000,  np.sin(theta_GR_GL),    -35.82],
                       [  np.sin(theta_GR_GL), 0.000000, -np.cos(theta_GR_GL),     83.80],
                       [             0.000000,  1.00000,             0.000000,   -153.00],
                       [             0.000000, 0.000000,             0.000000, 1.000000]])

#Transfrom from grinder local frame to Portafilterplacement (PFP)
T_GR_PFP_np = np.array([[ 0.000000, 0.000000, -1.000000,   157.61],
                        [ 0.000000, 1.000000,  0.000000,  0.00000],
                        [ 1.000000, 0.000000,  0.000000,  -250.45],
                        [ 0.000000, 0.000000,  0.000000, 1.000000]])


#####################################################################################################################################################
###   TRANSFORMING TO THE PROVIDED TOOLS FROM THE CURRENT FRAME  ####

#Transform to the Grinding Tool
theta_CL_GT = np.radians(50)
T_TCP_GTP_np = np.array([[ np.cos(theta_CL_GT), -np.sin(theta_CL_GT), 0.00,    0.00],
                        [ np.sin(theta_CL_GT),  np.cos(theta_CL_GT), 0.00,    0.00],
                        [                0.00,                 0.00, 1.00, -102.82],
                        [                0.00,                 0.00, 0.00,    1.00]])

#Transform to grinding tool but lever specifically 
#Rotate the 50 degrees first
theta_CL_GTL = np.radians(50)
T_TCP_GT_np = np.array([[ np.cos(theta_CL_GT), -np.sin(theta_CL_GT), 0.00,    0.00],
                        [ np.sin(theta_CL_GT),  np.cos(theta_CL_GT), 0.00,    0.00],
                        [                0.00,                 0.00, 1.00,   0.00],
                        [                0.00,                 0.00, 0.00,    1.00]])
##Then rotate to lever frame on the tool and move it to the postiton
T_GT_GTL_np = np.array([[  0.000000, 0.000000, -1.000000,   67.06],
                        [  0.000000,-1.000000,  0.000000,  0.00000],
                        [ -1.000000, 0.000000,  0.000000,  -40.00],
                        [ 0.000000, 0.000000,  0.000000, 1.000000]])

#####################################################################################################################################################
#Coffee Machine Matrices
theta_coffee_machine = np.radians(105.038)
T_TCP_CMBase_np = np.array([[ np.cos(theta_coffee_machine), -np.sin(theta_coffee_machine),  0.000000,  -366.20],
                            [ np.sin(theta_coffee_machine),  np.cos(theta_coffee_machine),  0.000000,  -389.80],
                            [                     0.000000,                      0.000000,  1.000000,   341.38],
                            [                     0.000000,                      0.000000,  0.000000, 1.000000]])
#Rotation matrix from coffee machine to the buttons
R_CM_B_np = np.array([[ 0.00, 0.00,-1.00],
                      [-1.00, 0.00, 0.00],
                      [ 0.00, 1.00, 0.00]])



#####################################################################################################################################################
#Tamper Stand Matrices
theta_TCP_TA = np.radians(59.73)
T_TCP_TA_np = np.array([[ np.cos(theta_TCP_TA), np.sin(theta_TCP_TA),  0.000000,  599.13],
                        [ -np.sin(theta_TCP_TA),  np.cos(theta_TCP_TA),  0.000000,  0.0],
                        [            0.000000,             0.000000,  1.000000,   211.07],
                        [            0.000000,             0.000000,  0.000000, 1.000000]])

#Transform from tamper stand to the push up centre
T_TA_PU_np = np.array([[ 0.00, 1.00,  0.00,  -80],
                        [ 0.00, 0.00,  1.00,  0.0],
                        [ 1.00, 0.00,  0.00,  -55],
                        [ 0.00, 0.00,  0.00, 1.000000]])

#Transform from tamper stand to the scraper centre
T_TA_SC_np = np.array([[ 0.00, 1.00,  0.00,  70],
                        [ 0.00, 0.00,  1.00,  0.0],
                        [ 1.00, 0.00,  0.00,  -32],
                        [ 0.00, 0.00,  0.00, 1.000000]])


#Transform from TCP to the center of the potafilter part that goes into the pusherupperer (note 50degree is done after this)
#ADJUST end values to shift the centre of portafilter circular end
theta_TCP_PF = np.radians(-7.5)
T_TCP_PF_np = np.array([[ np.cos(theta_TCP_PF),  0.000000,  -np.sin(theta_TCP_PF),    -4.71],
                       [              0.000000,  1.000000,               0.000000,     0.00],
                       [  np.sin(theta_TCP_PF),  0.000000,   np.cos(theta_TCP_PF),    -142.76],
                       [              0.000000,  0.000000,               0.000000, 1.000000]])

#Transform to change position and rotate about z   
theta_TCP_PFA_TA = np.radians(50) #change 50 degree angle before moving 7.5 degrees PFA portafilter angle
T_TCP_PFA_TA_np = np.array([[ np.cos(theta_TCP_PFA_TA), -np.sin(theta_TCP_PFA_TA), 0.00,    0.00],
                            [ np.sin(theta_TCP_PFA_TA),  np.cos(theta_TCP_PFA_TA), 0.00,    0.00],
                            [                     0.00,                      0.00, 1.00,    0.00],
                            [                     0.00,                      0.00, 0.00,    1.00]])                    
   



def portafilterPlacement():
   #Calculate placement position for portafilter under the grinder
   PFP_compensation = rdk.transl(0,0,5)
   T_CL_PFT = rdk.Mat(T_CL_PFT_np.tolist()) #what we multiply after making adjustment
   
   T_GR_PFP = rdk.Mat(T_GR_PFP_np.tolist())
   T_TCP_GR  = rdk.Mat(T_TCP_GR_np.tolist())
   T_TCP_PFP = T_TCP_GR *T_GR_PFP #Waht we multiple by before the adjustment
   
   
   T_TCP_PFPPreset = T_TCP_PFP * rdk.transl(0,-1,-60)* rdk.roty(np.radians(-2))*T_CL_PFT
   #T_TCP_PFPPreset = T_TCP_PFP * rdk.transl(0,0,-40)* rdk.roty(np.radians(-2))*T_CL_PFT
   T_TCP_PFPSet = T_TCP_PFP * rdk.transl(-3,-1,-3)* rdk.roty(np.radians(-1))*T_CL_PFT

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
   #robot.MoveL(T_TCP_PFP_ADJ, blocking=True)
   RDK.RunProgram('Portafilter Tool Detach (Grinder)', True)

def pushButtonsOnGrinder():   
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
   T_TCP_GB_np = np.matmul(np.matmul(T_TCP_GR_np, T_GR_GB_np),T_TCP_GTP_np)
   
   T_TCP_GB = rdk.Mat(T_TCP_GB_np.tolist())
   T_TCP_GB = T_TCP_GB * rdk.rotz(np.radians(40))
   T_TCP_GB1Preset = T_TCP_GB * rdk.transl(0,0,-20)
   T_TCP_GB1Push = T_TCP_GB * rdk.transl(0,0,10)
   T_TCP_GB2Preset = T_TCP_GB * rdk.transl(0,17,-30)
   T_TCP_GB2Push = T_TCP_GB * rdk.transl(0,17,10)

   #robot.MoveJ(T_TCP_GB, blocking=True)
   robot.MoveJ(T_TCP_GB2Preset, blocking=True)
   robot.MoveL(T_TCP_GB2Push, blocking=True)
   robot.MoveL(T_TCP_GB2Preset, blocking=True)
   rdk.pause(4)  
   robot.MoveL(T_TCP_GB1Preset, blocking=True)
   robot.MoveL(T_TCP_GB1Push, blocking=True)
   robot.MoveL(T_TCP_GB1Preset, blocking=True)


   #robot.MoveL(T_TCP_GB, blocking=True) #This is the origin of plane
   
def pullLever():   
   #Calculate the position of the lever
   T_TCP_GTL_np = np.matmul(np.matmul(T_TCP_GR_np, T_GR_GL_np),T_GT_GTL_np )
   T_TCP_GT = rdk.Mat(T_TCP_GT_np.tolist())
   T_TCP_GTL = rdk.Mat(T_TCP_GTL_np.tolist())

   T_TCP_GTLStart = T_TCP_GTL*T_TCP_GT
   J_TCP_GTLPreset = [107.180244, -60.445471, 91.787676, -31.342205, 51.970244, -130.000000] 
   T_TCP_GTLPull1 = T_TCP_GTL * T_TCP_GT * rdk.transl(0,0,-50) #This is the value I need to change to increase pulling position
   T_TCP_GTLPull_Turn = T_TCP_GTL * rdk.roty(np.radians(15)) *rdk.transl(0,0,-65)*T_TCP_GT
   T_TCP_GTLPull2 = T_TCP_GTL * rdk.roty(np.radians(15)) *rdk.transl(-20, 0,-110)*T_TCP_GT #CHANGE LAST VALUE TO MAKE IT MOVE FURTHER  

   
   itterations= 3
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
   
   J_GR_TS_Step1 = [99.470000, -60.440000, 91.780000, -31.340000, 51.970000, -129.990000]
   J_GR_TS_Step2 = [87.630000, -81.980000, 112.860000, -112.910000, 3.560000, -167.400000] 
   J_GR_TS_Step3 = [80.530000, -93.120000, 112.870000, -112.910000, 89.940000, -167.400000]
   J_GR_TS_Step4 = [11.840000, -93.120000, 67.720000, -112.910000, 89.940000, -167.400000]
   J_GR_TS_Step5 = [-11.840000, -93.120000, -64.110000, -112.910000, 89.940000, -167.400000]
   J_GR_TS_Step6 = [-78.160000, -93.120000, -64.110000, -112.910000, 89.940000, -167.400000]
   
   robot.MoveJ(J_GR_TS_Step1, blocking=True)
   robot.MoveJ(J_GR_TS_Step2, blocking=True)
   robot.MoveJ(J_GR_TS_Step3, blocking=True)   
   robot.MoveJ(J_GR_TS_Step4, blocking=True) 
   robot.MoveJ(J_GR_TS_Step5, blocking=True)
   robot.MoveJ(J_GR_TS_Step6, blocking=True) 
   
   RDK.RunProgram('Grinder Tool Detach (Stand)', True)
   
def fetchFullPortafilter():  
   
   
   J_TS_GR_Step1 = [-156.440000, -81.180000, -75.390000, -181.780000, 181.780000, -182.170000]
   J_TS_GR_Step2 = [-156.440000, -81.180000, -127.130000, -181.780000, 181.780000, -182.170000]
   J_TS_GR_Step3 = [-71.050000, -81.180000, -127.130000, -136.630000, 181.780000, -182.170000]
   J_TS_GR_Step4 = [-14.210000, -65.350000, -148.510000, -148.510000, 293.470000, -231.680000]
   J_TS_GR_Step5 = [-17.818290, -100.202192, -139.887909, -111.506940, 296.722534, -223.800304]   

   robot.MoveJ(J_TS_GR_Step1, blocking=True)
   robot.MoveJ(J_TS_GR_Step2, blocking=True)
   robot.MoveJ(J_TS_GR_Step3, blocking=True)   
   robot.MoveJ(J_TS_GR_Step4, blocking=True)   
   robot.MoveJ(J_TS_GR_Step5, blocking=True)   
   
   RDK.RunProgram('Portafilter Tool Attach (Grinder)', True)
   
   J_GR_TA_Step1 = [-15.011332, -97.290411, -145.804176, -108.512264, 297.551540, -224.181867]
   J_GR_TA_Step2 = [2.370000, -97.290000, -145.800000, -108.510000, 297.550000, -224.180000]
   J_GR_TA_Step3 = [-21.320000, -103.370000, -141.390000, -100.990000, 230.680000, -212.670000]
   
   robot.MoveJ(J_GR_TA_Step1, blocking=True)
   robot.MoveJ(J_GR_TA_Step2, blocking=True)
   robot.MoveJ(J_GR_TA_Step3, blocking=True)

def tamperScraper():
   T_TCP_TA = rdk.Mat(T_TCP_TA_np.tolist()) #transform from base to Tamper stand
   T_TA_SC = rdk.Mat(T_TA_SC_np.tolist()) #Transform from tamper stand to SC frame
   T_TCP_PF = rdk.Mat(T_TCP_PF_np.tolist()) #Change 7.5degree and move back TCP relative to portafilter
   T_TCP_PFA_TA = rdk.Mat(T_TCP_PFA_TA_np.tolist()) #Change 50degree angle of the portafilter   
   
   T_TCP_PFSC = T_TCP_TA * T_TA_SC #This defines the frame for the pusher tool
   T_TCP_PFT = T_TCP_PF* T_TCP_PFA_TA #This part defines the transform from the TCP to the PFT and must go at the end of calc   
   
   T_TCP_SCPreset = T_TCP_PFSC * rdk.transl(-30,0,-100) * T_TCP_PFT #Edit z direction if needed 
   T_TCP_SCPush = T_TCP_PFSC * rdk.transl(-30,0,50) * T_TCP_PFT   #Edit z direction of push too bog or too small   
   T_TCP_SCDrop = T_TCP_PFSC * rdk.transl(-100,0,-100) * T_TCP_PFT   #Edit z direction of push too bog or too small  
   T_TCP_SCSlide = T_TCP_PFSC * rdk.transl(-100,-150,-100) * T_TCP_PFT   #Edit z direction of push too bog or too small  
   
   robot.MoveJ(T_TCP_SCPreset, blocking=True)
   robot.MoveL(T_TCP_SCPush, blocking=True)
   robot.MoveL(T_TCP_SCPreset, blocking=True)
   
   robot.MoveJ(T_TCP_SCDrop, blocking=True)
   robot.MoveJ(T_TCP_SCSlide, blocking=True)
   
def tamperPusher():
   
   #Move to tamper pusher
   
   J_Slide = [-31.658996, -93.568750, -155.250750, -95.550201, 208.976750, -206.247112]
   
   T_TCP_TA = rdk.Mat(T_TCP_TA_np.tolist()) #transform from base to Tamper stand
   T_TA_PU = rdk.Mat(T_TA_PU_np.tolist()) #Transform from tamper stand to PU frame
   T_TCP_PF = rdk.Mat(T_TCP_PF_np.tolist()) #Change 7.5degree and move back TCP relative to portafilter
   T_TCP_PFA_TA = rdk.Mat(T_TCP_PFA_TA_np.tolist()) #Change 50degree angle of the portafilter

   T_TCP_PFPU = T_TCP_TA * T_TA_PU #This defines the frame for the pusher tool
   T_TCP_PFT = T_TCP_PF* T_TCP_PFA_TA #This part defines the transform from the TCP to the PFT and must go at the end of calc
   
   T_TCP_PUPreset = T_TCP_PFPU * rdk.transl(-80,0,0) * T_TCP_PFT #Edit z direction if needed 
   T_TCP_PUPush = T_TCP_PFPU * rdk.transl(-30,0,0) * T_TCP_PFT   #Edit z direction of push too bog or too small
   
   robot.MoveL(T_TCP_PUPreset, blocking=True)
   robot.MoveL(T_TCP_PUPush, blocking=True)
   robot.MoveL(T_TCP_PUPreset, blocking=True)
   robot.MoveJ(J_Slide, blocking=True)
   #RDK.RunProgram('Portafilter Tool Attach (Stand)', True)
   
def passPortafilter():
   
   J_TA_HU_Step1 = [-66.320000, -93.570000, -155.250000, -95.550000, 208.970000, -206.240000]
   J_TA_HU_Step2 = [-66.320000, -93.570000, -155.250000, -103.370000, 260.200000, -217.430000]
   J_TA_HU_Step3 = [-66.320000, -93.570000, -134.260000, -122.380000, 260.200000, -217.430000]

   
   robot.MoveJ(J_TA_HU_Step1, blocking=True)
   robot.MoveJ(J_TA_HU_Step2, blocking=True)
   robot.MoveJ(J_TA_HU_Step3, blocking=True)
   rdk.pause(12)

     

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
button_center_np = np.array([[0.000000, 0.000000, 0.000000,   25.250000],
                             [0.000000, 0.000000, 0.000000,  -30.000000],
                             [0.000000, 0.000000, 0.000000, -170.000000],
                             [0.000000, 0.000000, 0.000000,    0.000000]])   

button_top_np = np.array([[0.000000, 0.000000, 0.000000,   10.250000],
                          [0.000000, 0.000000, 0.000000,  -30.000000],
                          [0.000000, 0.000000, 0.000000, -152.000000],
                          [0.000000, 0.000000, 0.000000,    0.000000]])         

button_bottom_np = np.array([[0.000000, 0.000000, 0.000000,   30.25000],
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
J_intermediateGrinderTool = [-157.010000, -67.940000, -79.440000, -157.500000, 177.770000, -208.440000]

#####################################################################################################################################################


def coffee_machine_buttons():
    """ Coffee machine buttons to turn the coffee machine on to pour the coffee *** 7 seconds ***. """
    ###################################################################################################
    """ Scheduler. """
    ###################################################################################################
    robot.setPoseTool(master_tool)

    #robot.MoveJ(J_intermediateGrinderTool, blocking=True)

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
T_cup_base_np = np.array([[-1.000000, 0.000000,  0.000000,    1.490000],
                          [ 0.000000, 1.000000,  0.000000, -600.540000],
                          [ 0.000000, 0.000000, -1.000000,  -20.000000],
                          [ 0.000000, 0.000000,  0.000000,    1.000000]])

# Orientate frame to get the grabber to point at the cup stack.
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
T_cup_angle_np = np.array([[np.cos(np.radians(140)), -np.sin(np.radians(140)), 0.000000,    0.000000],
                           [np.sin(np.radians(140)),  np.cos(np.radians(140)), 0.000000,  -47.000000],
                           [               0.000000,                 0.000000, 1.000000, -186.110000],
                           [               0.000000,                 0.000000, 0.000000,    1.000000]])

# Shift by the desired angle with the cup transform.
oriented_frame_np = np.matmul(cup_frame_np, T_cup_angle_np)   

# Base of the coffee machine position in world frame.
T_coffee_machine_base_np = np.array([[ np.cos(np.radians(74.96)), np.sin(np.radians(74.96)), 0.000000, -366.200000],
                                     [-np.sin(np.radians(74.96)), np.cos(np.radians(74.96)), 0.000000, -389.800000],
                                     [                  0.000000,                  0.000000, 1.000000,  341.380000],
                                     [                  0.000000,                  0.000000, 0.000000,    1.000000]])

# Twist the end effector to complete orientation of grabber to place the cup the correct way up in the coffee machine.
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
    #robot.MoveJ(J_cup_tool_orient, blocking=True)
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

T_coffee_machine_base_np = np.array([[ np.cos(np.radians(74.26)), np.sin(np.radians(74.26)), 0.000000, -366.200000],
                                     [-np.sin(np.radians(74.26)), np.cos(np.radians(74.26)), 0.000000, -389.800000],
                                     [                  0.000000,                  0.000000, 1.000000,  341.380000],
                                     [                  0.000000,                  0.000000, 0.000000,    1.000000]])

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
    #robot.MoveJ(J_cup_tool_orient, blocking=True)

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
    '''
    RDK.RunProgram("Cup Tool Open", True)

    robot.MoveJ(J_back_out_point, blocking=True)

    robot.MoveJ(J_cup_to_coffee_machine_intermediate_point, blocking=True)

    robot.MoveJ(J_cup_intermediate_point, blocking=True)

    robot.MoveJ(J_cup_tool_orient, blocking=True)

    RDK.RunProgram("Cup Tool Close", True)

    RDK.RunProgram("Cup Tool Detach (Stand)", True)

    robot.setPoseTool(master_tool)

    robot.MoveJ(J_cup_tool_orient, blocking=True)
    '''






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
