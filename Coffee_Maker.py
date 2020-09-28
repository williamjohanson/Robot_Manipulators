# A more advanced example to get you moving with the RoboDK python API
# Note, as there are many solutions for a given pose, sometimes when
# running this, the robot may choose a weird pose that then doesn't allow
# the subsequent motion (due to being near a singularity etc). If this occurs, 
# just manually reset the robot startingposition to somewhere else and try again
# C Pretty, 18 Sept 2019
# version 2

   # RoboDK API
import robodk as rdk
import robolink as rl # Robot toolbox
import numpy as np

RDK = rl.Robolink()

robot = RDK.Item('UR5')
world_frame = RDK.Item('UR5 Base')
target = RDK.Item('Home')   # existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())


# Directly use the RDK Matrix object from to hold pose (its an HT)
T_home = rdk.Mat([[     0.000000,     0.000000,     1.000000,   523.370000 ],
     [-1.000000,     0.000000,     0.000000,  -109.000000 ],
     [-0.000000,    -1.000000,     0.000000,   607.850000 ],
      [0.000000,     0.000000,     0.000000,     1.000000 ]])

J_Home = [-211.045963, -83.359584, 54.376727, 28.982857, 58.954037, -180.000000] #to help keep home position constant

# Joint angles
J_intermediateGrinderTool = [-141.263517, -93.187023, -64.045061, -112.910653, 89.942400, -166.994273]
J_intermediateCupTool = [-183.954225, -98.097022, -52.302160, -119.399545, 90.411813, -193.952774]
J_intermediatePortafilter = [-156.440056, -81.180555, -75.398392, -113.543732, 89.907041, -182.170785]


#Transform to Portafilter Tool
#First need to spin TCP 50degrees using following transform
theta_CL_PFA = np.radians(50) #change 50 degree angle before moving 7.5 degrees
T_TCP_PFA_np = np.array([[ np.cos(theta_CL_PFA), -np.sin(theta_CL_PFA), 0.00,    0.00],
                     [ np.sin(theta_CL_PFA),  np.cos(theta_CL_PFA), 0.00,    0.00],
                     [                   0.00,                    0.00, 1.00,    0.00],
                     [                   0.00,                    0.00, 0.00,    1.00]])
theta_T_TCP_PFT = np.radians(7.5)
T_TCP_PFT_np = np.array([[np.cos(theta_T_TCP_PFT ), 0.00, np.sin(theta_T_TCP_PFT ), 32.00],
                         [    0.00, 1.00,   0.00, 0.00],
                         [-np.sin(theta_T_TCP_PFT ), 0.00,  np.cos(theta_T_TCP_PFT ), -27.56],
                         [    0.00, 0.00,   0.00,    1.00]])

T_compensation_np = np.array([[ 1.000000, 0.000000, 0.000000,   0.00],
                              [ 0.000000, 1.000000,  0.000000,  0.00000],
                              [ 0.000000, 0.000000,  1.000000,  0.00],
                              [ 0.000000, 0.000000,  0.000000, 1.000000]])
T_CL_PFT_np = np.matmul(T_TCP_PFT_np, np.matmul(T_TCP_PFA_np, T_compensation_np))  
#Grinder Matrices
theta_grinder_machine = np.radians(44.79)
T_TCP_GR_np = np.array([[ -np.cos(theta_grinder_machine), -np.sin(theta_grinder_machine),  0.000000,  482.29],
                             [ np.sin(theta_grinder_machine),  -np.cos(theta_grinder_machine),  0.000000,  -433.74],
                             [                      0.000000,                       0.000000,  1.000000,   314.13],
                             [                      0.000000,                       0.000000,  0.000000, 1.000000]])

#Transfrom from grinder local frame to Portafilterplacement (PFP)
T_GR_PFP_np = np.array([[ 0.000000, 0.000000, -1.000000,   157.61],
                        [ 0.000000, 1.000000,  0.000000,  0.00000],
                        [ 1.000000, 0.000000,  0.000000,  -250.45],
                        [ 0.000000, 0.000000,  0.000000, 1.000000]])

#Transform to the Grinding Tool
theta_CL_GT = np.radians(50)
T_TCP_GT_np = np.array([[ np.cos(theta_CL_GT), -np.sin(theta_CL_GT), 0.00,    0.00],
                        [ np.sin(theta_CL_GT),  np.cos(theta_CL_GT), 0.00,    0.00],
                        [                0.00,                 0.00, 1.00, -102.82],
                        [                0.00,                 0.00, 0.00,    1.00]])
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



theta_press = np.radians(59.73)
T_pressBase_np = np.array([[ np.cos(theta_press), -np.sin(theta_press),  0.000000,  370.66],
                        [ np.sin(theta_press),  np.cos(theta_press),  0.000000,  -321.55],
                        [            0.000000,             0.000000,  1.000000,   66.86],
                        [            0.000000,             0.000000,  0.000000, 1.000000]])



def coffeeMachineButtons():
   
 
   T_GT_CMBase_np = np.matmul(T_TCP_CMBase_np, T_TCP_GT_np)
      
   P_CM_B1Set_np = np.array([ 70, 98.75, -32]) #B1 set position
   P_CM_B1Set1_np = np.array([ 70, 0.00, -32]) 
   P_CM_B1Push_np = np.array([ 50, 98.75, -32])
   
   T_CM_B1Set_np = np.zeros((4,4))
   T_CM_B1Set1_np = np.zeros((4,4))
   T_CM_B1Push_np = np.zeros((4,4))
   
   T_CM_B1Set_np[3,3] = 1.0
   T_CM_B1Set_np[0:3,0:3] = R_CM_B_np
   T_CM_B1Set1_np[3,3] = 1.0
   T_CM_B1Set1_np[0:3,0:3] = R_CM_B_np   
   T_CM_B1Push_np[3,3] = 1.0
   T_CM_B1Push_np[0:3,0:3] = R_CM_B_np
   T_CM_B1Set_np[0:3,3] = P_CM_B1Set_np
   T_CM_B1Set1_np[0:3,3] = P_CM_B1Set1_np
   T_CM_B1Push_np[0:3,3] = P_CM_B1Push_np
   
   
   T_GT_B1Set_np = np.matmul( np.matmul(T_TCP_CMBase_np, T_CM_B1Set_np), T_TCP_GT_np)
   T_GT_B1Set1_np = np.matmul( np.matmul(T_TCP_CMBase_np, T_CM_B1Set1_np), T_TCP_GT_np)
   T_GT_B1Push_np = np.matmul( np.matmul(T_TCP_CMBase_np, T_CM_B1Push_np), T_TCP_GT_np)
   
   T_GT_CMBase = rdk.Mat(T_GT_CMBase_np.tolist())
   T_GT_B1Set = rdk.Mat(T_GT_B1Set_np.tolist())
   T_GT_B1Set1 = rdk.Mat(T_GT_B1Set_np.tolist())
   T_GT_B1Push = rdk.Mat(T_GT_B1Push_np.tolist())
   
   #Movement to press B1 from home
   robot.MoveJ(J_Home, blocking=True)
   robot.MoveJ(J_intermediateGrinderTool, blocking=True)
   RDK.RunProgram('Grinder Tool Attach (Stand)', True)
   J_GT_B1Set1 = [-153.922130, -111.554777, -111.667569, 43.222346, 168.960130, -130.000000]
   robot.MoveL(J_GT_B1Set1, blocking=True)
   robot.MoveL(T_GT_B1Set, blocking=True)
   robot.MoveL(T_GT_B1Push, blocking=True) 
   robot.MoveJ(T_GT_B1Set, blocking=True)
   robot.MoveJ(J_intermediateGrinderTool, blocking=True)
   RDK.RunProgram('Grinder Tool Detach (Stand)', True)
   #robot.MoveJ(J_intermediatePointPusher, blocking=True)
   #RDK.RunProgram('Grinder Tool Detach (Stand)', True)
   



def portafilterPlacement():
   #Calculate placement position for portafilter under the grinder
   T_TCP_PFP_np = np.matmul(T_TCP_GR_np, np.matmul(T_GR_PFP_np, T_CL_PFT_np)) 
   
   T_TCP_PFP = rdk.Mat(T_TCP_PFP_np.tolist())
   T_TCP_GR  = rdk.Mat(T_TCP_GR_np.tolist())
   
   #Steps from tools tand to Grinder
   J_Step1 = [-156.440000, -81.180000, -75.390000, -181.780000, 181.780000, -182.170000]
   J_Step2 = [-156.440000, -81.180000, -127.130000, -181.780000, 181.780000, -182.170000]
   J_Step3 = [-71.050000, -81.180000, -127.130000, -136.630000, 181.780000, -182.170000]
   J_Step4 = [-14.210000, -65.350000, -148.510000, -148.510000, 293.470000, -231.680000]
   J_Step5 = [-17.818290, -100.202192, -139.887909, -111.506940, 296.722534, -223.800304]
   robot.MoveJ(J_Home, blocking=True)
   robot.MoveJ(J_intermediatePortafilter, blocking=True)
   RDK.RunProgram('Portafilter Tool Attach (Stand)', True)
   robot.MoveJ(J_Step1, blocking=True)
   robot.MoveJ(J_Step2, blocking=True)
   robot.MoveJ(J_Step3, blocking=True)
   robot.MoveJ(J_Step4, blocking=True)
   robot.MoveJ(J_Step5, blocking=True)
   #robot.MoveJ(T_TCP_GR, blocking=True)
   robot.MoveL(T_TCP_PFP, blocking=True)
   RDK.RunProgram('Portafilter Tool Detach (Grinder)', True)
   
   #go and grab the Grinder tool
   #RDK.RunProgram('Portafilter Tool Attach (Grinder)', True)
   robot.MoveJ(J_intermediateGrinderTool, blocking=True)
   RDK.RunProgram('Grinder Tool Attach (Stand)', True)
   
   
   
#coffeeMachineButtons()
portafilterPlacement()



   

#portaPosition = np.array([1.00, 1.00, 1.00, 1.00])
grinder_base_position_np = np.array([0, 50, -50, 1])
#T_portafilterCallPoint_np = T_grinderBase_np.dot(grinder_base_position_np)

portFilterCallPoint = T_grinderPortaPlace_np.dot(grinder_base_position_np)

T_portafilterCallPoint_np = [328.78, -398.04, 63.68, 0, 0, 0]
grinder_base_position = rdk.Mat(grinder_base_position_np)
T_grinderToolApproach = rdk.Mat(T_grinderToolApproach_np.tolist())
T_TCP_CMBase = rdk.Mat(T_TCP_CMBase_np.tolist())
T_pressBase = rdk.Mat(T_pressBase_np.tolist())
T_intermediatePortafilter = rdk.Mat(T_intermediatePortafilter_np.tolist())
T_grinderPortaPlace = rdk.Mat(T_grinderPortaPlace_np.tolist())
#print(T_grinderPortaPlace)
#T_portafilterCallPoint = rdk.Mat(T_portafilterCallPoint_np.tolist())
J_portaPlace = [122.938096, -98.253502, 137.691455, -39.437953, 83.226765, -0.000000]


#robot.MoveJ(T_home, blocking=True)
#robot.MoveJ(J_intermediateCupTool, blocking=True)
#robot.MoveJ(T_intermediatePortafilter, blocking=True)
#RDK.RunProgram('Portafilter Tool Attach (Stand)', True) 
#robot.MoveJ(T_grinderToolApproach, blocking=True)
#robot.MoveJ(J_intermediatePointPusher, blocking=True)
#RDK.RunProgram('Grinder Tool Attach (Stand)', True)
#robot.MoveL(T_grinderToolApproach, blocking=True)
#robot.MoveJ(T_TCP_CMBase, blocking=True)
#robot.MoveJ(T_GT_CMBase, blocking=True)
#robot.MoveJ(T_GT_B1Set, blocking=True)
#robot.MoveL(T_GT_B1Push, blocking=True)


#robot.MoveJ(T_grinderPortaPlace, blocking=True)


#RDK.RunProgram('Portafilter Tool Detach (Grinder)', True) # call subprogram
#RDK.RunProgram('Portafilter Tool Attach (Grinder)', True) # call subprogram
#RDK.RunProgram('Portafilter Tool Detach (Stand)', True) # call subprogram
#RDK.RunProgram('Grinder Tool Attach (Stand)', True) # call subprogram
#RDK.RunProgram('Grinder Tool Attach (Stand)', True) # call subprogram
#RDK.RunProgram('Grinder Tool Detach (Stand)', True) # call subfunction

#robot.MoveJ(T_grinderBase, blocking=True)
#robot.MoveJ(T_portafilterCallPoint_np, blocking=True)
#robot.MoveJ (grinder_base_position, blocking=True)
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


