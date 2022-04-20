#obuses positions
#b and c are always the same
pose_b = 0
pose_c = 179
# a is -73 or 106.9
pose_a_left = -179
pose_a_right = 1
# z is always safe
pose_z_safe = 1550 #1535.39



##AXIS POSITIONS A1 and A6
pick_A1=-87.0
pick_right_A6=267.0
pick_left_A6=87.0
place_A1=87.0
place_left_A6=267.0
place_right_A6=87.0
#rotation table
table_pose_x = 1620
table_pose_y = -8.45
table_pose_z = pose_z_safe
table_pose_a = -178
table_pose_b = pose_b
table_pose_c = pose_c

#Datos hueveras
center_distance_16 = 129.38
center_distance_8 = 260
center_distance_4 = 280
center_distance_2 = 380
#
#x: 390.0305
#y: -1334.5508
#z: 1435.3049
#A: -130.4625
#B: 0.4523
#C: 179.7874
#

#huevera 16
#obus16


#obus1
H16O1_Pose_x = -260  #DATA
H16O1_Pose_y = -1342 #DATA
H16O1_Pose_z = pose_z_safe
H16O1_Pose_a = -179
H16O1_Pose_b = pose_b
H16O1_Pose_c = pose_c
#obus2
H16O2_Pose_x = H16O1_Pose_x+center_distance_16
H16O2_Pose_y = H16O1_Pose_y #The whole first line has the same y value
H16O2_Pose_z = pose_z_safe
H16O2_Pose_a = pose_a_left
H16O2_Pose_b = pose_b
H16O2_Pose_c = pose_c
#obus3
H16O3_Pose_x = H16O2_Pose_x+center_distance_16
H16O3_Pose_y = H16O1_Pose_y
H16O3_Pose_z = pose_z_safe
H16O3_Pose_a = pose_a_left
H16O3_Pose_b = pose_b
H16O3_Pose_c = pose_c
#obus4
H16O4_Pose_x = H16O3_Pose_x+center_distance_16
H16O4_Pose_y = H16O1_Pose_y
H16O4_Pose_z = pose_z_safe
H16O4_Pose_a = pose_a_left
H16O4_Pose_b = pose_b
H16O4_Pose_c = pose_c
#obus5
H16O5_Pose_x = 11  #DATA
H16O5_Pose_y = H16O1_Pose_y
H16O5_Pose_z = pose_z_safe
H16O5_Pose_a = pose_a_right
H16O5_Pose_b = pose_b
H16O5_Pose_c = pose_c
#obus6
H16O6_Pose_x = H16O5_Pose_x+center_distance_16
H16O6_Pose_y = H16O1_Pose_y
H16O6_Pose_z = pose_z_safe
H16O6_Pose_a = pose_a_right
H16O6_Pose_b = pose_b
H16O6_Pose_c = pose_c
#obus7
H16O7_Pose_x = H16O6_Pose_x+center_distance_16
H16O7_Pose_y = H16O1_Pose_y
H16O7_Pose_z = pose_z_safe
H16O7_Pose_a = pose_a_right
H16O7_Pose_b = pose_b
H16O7_Pose_c = pose_c
#obus8
H16O8_Pose_x = H16O7_Pose_x+center_distance_16
H16O8_Pose_y = H16O1_Pose_y
H16O8_Pose_z = pose_z_safe
H16O8_Pose_a = pose_a_right
H16O8_Pose_b = pose_b
H16O8_Pose_c = pose_c
#obus9
H16O9_Pose_x = H16O1_Pose_x
H16O9_Pose_y = -1875 #DATA
H16O9_Pose_z = pose_z_safe
H16O9_Pose_a = pose_a_left
H16O9_Pose_b = pose_b
H16O9_Pose_c = pose_c
#obus10
H16O10_Pose_x = H16O2_Pose_x
H16O10_Pose_y = H16O9_Pose_y
H16O10_Pose_z = pose_z_safe
H16O10_Pose_a = pose_a_left
H16O10_Pose_b = pose_b
H16O10_Pose_c = pose_c
#obus11
H16O11_Pose_x = H16O3_Pose_x
H16O11_Pose_y = H16O9_Pose_y
H16O11_Pose_z = pose_z_safe
H16O11_Pose_a = pose_a_left
H16O11_Pose_b = pose_b
H16O11_Pose_c = pose_c

#obus12
H16O12_Pose_x = H16O4_Pose_x
H16O12_Pose_y = H16O9_Pose_y
H16O12_Pose_z = pose_z_safe
H16O12_Pose_a = pose_a_left
H16O12_Pose_b = pose_b
H16O12_Pose_c = pose_c
#obus13
H16O13_Pose_x = H16O5_Pose_x
H16O13_Pose_y = H16O9_Pose_y
H16O13_Pose_z = pose_z_safe
H16O13_Pose_a = pose_a_right
H16O13_Pose_b = pose_b
H16O13_Pose_c = pose_c
#obus14
H16O14_Pose_x = H16O6_Pose_x
H16O14_Pose_y = H16O9_Pose_y
H16O14_Pose_z = pose_z_safe
H16O14_Pose_a = pose_a_right
H16O14_Pose_b = pose_b
H16O14_Pose_c = pose_c
#obus15
H16O15_Pose_x = H16O7_Pose_x
H16O15_Pose_y = H16O9_Pose_y
H16O15_Pose_z = pose_z_safe
H16O15_Pose_a = pose_a_right
H16O15_Pose_b = pose_b
H16O15_Pose_c = pose_c
#obus16
H16O16_Pose_x = H16O8_Pose_x
H16O16_Pose_y = H16O9_Pose_y
H16O16_Pose_z = pose_z_safe
H16O16_Pose_a = pose_a_right
H16O16_Pose_b = pose_b
H16O16_Pose_c = pose_c



#huevera 8
#obus1
H8O1_Pose_x = 1645.58 #DATA
H8O1_Pose_y = -412.77 #DATA
H8O1_Pose_z = pose_z_safe
H8O1_Pose_a = pose_a_left
H8O1_Pose_b = pose_b
H8O1_Pose_c = pose_c
#obus2
H8O2_Pose_x = H8O1_Pose_x+center_distance_8
H8O2_Pose_y = H8O1_Pose_y
H8O2_Pose_z = pose_z_safe
H8O2_Pose_a = pose_a_left
H8O2_Pose_b = pose_b
H8O2_Pose_c = pose_c
#obus3
H8O3_Pose_x = 1514.58 #DATA
H8O3_Pose_y = H8O1_Pose_y
H8O3_Pose_z = pose_z_safe
H8O3_Pose_a = pose_a_right
H8O3_Pose_b = pose_b
H8O3_Pose_c = pose_c
#obus4
H8O4_Pose_x = H8O3_Pose_x+center_distance_8
H8O4_Pose_y = H8O1_Pose_y
H8O4_Pose_z = pose_z_safe
H8O4_Pose_a = pose_a_right
H8O4_Pose_b = pose_b
H8O4_Pose_c = pose_c
#obus5
H8O5_Pose_x = H8O1_Pose_x
H8O5_Pose_y = -116.31 #DATA
H8O5_Pose_z = pose_z_safe
H8O5_Pose_a = pose_a_left
H8O5_Pose_b = pose_b
H8O5_Pose_c = pose_c
#obus6
H8O6_Pose_x = H8O2_Pose_x
H8O6_Pose_y = H8O5_Pose_y
H8O6_Pose_z = pose_z_safe
H8O6_Pose_a = pose_a_left
H8O6_Pose_b = pose_b
H8O6_Pose_c = pose_c
#obus7
H8O7_Pose_x = H8O3_Pose_x
H8O7_Pose_y = H8O5_Pose_y
H8O7_Pose_z = pose_z_safe
H8O7_Pose_a = pose_a_right
H8O7_Pose_b = pose_b
H8O7_Pose_c = pose_c
#obus8
H8O8_Pose_x = H8O4_Pose_x
H8O8_Pose_y = H8O5_Pose_y
H8O8_Pose_z = pose_z_safe
H8O8_Pose_a = pose_a_right
H8O8_Pose_b = pose_b
H8O8_Pose_c = pose_c


#huevera 4
#obus1
H4O1_Pose_x = 1803.39 #DATA
H4O1_Pose_y = -346.9 #DATA
H4O1_Pose_z = pose_z_safe
H4O1_Pose_a = pose_a_left
H4O1_Pose_b = pose_b
H4O1_Pose_c = pose_c
#obus2
H4O2_Pose_x = 1803.39 #DATA
H4O2_Pose_y = H4O1_Pose_y
H4O2_Pose_z = pose_z_safe
H4O2_Pose_a = pose_a_left
H4O2_Pose_b = pose_b
H4O2_Pose_c = pose_c
#obus3
H4O3_Pose_x = H4O1_Pose_x
H4O3_Pose_y = -13.90 #DATA
H4O3_Pose_z = pose_z_safe
H4O3_Pose_a = pose_a_right
H4O3_Pose_b = pose_b
H4O3_Pose_c = pose_c
#obus4
H4O4_Pose_x = H4O2_Pose_x
H4O4_Pose_y = H4O3_Pose_y
H4O4_Pose_z = pose_z_safe
H4O4_Pose_a = pose_a_right
H4O4_Pose_b = pose_b
H4O4_Pose_c = pose_c



#huevera 2
#obus2
H2O2_Pose_x = 82.23
H2O2_Pose_y = -1600
H2O2_Pose_z = pose_z_safe
H2O2_Pose_a = pose_a_right
H2O2_Pose_b = pose_b
H2O2_Pose_c = pose_c
#obus1
H2O1_Pose_x = -194.23
H2O1_Pose_y = -1590.28
H2O1_Pose_z = pose_z_safe
H2O1_Pose_a = pose_a_left
H2O1_Pose_b = pose_b
H2O1_Pose_c = pose_c
