# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1. ชัยภัทร_6516
2. ธรา_6529
3.
'''

import numpy as np
import math
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
from spatialmath import SE3
from math import pi
from HW3_utils import FKHW3

#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q: list[float], print_jacobian: bool = False) -> np.ndarray:
    
    # เรียกใช้งานฟังก์ชัน FKHW3 เพื่อหาค่าการหมุนและตำแหน่ง
    R, P, R_e, p_e = FKHW3(q)
    
    J_e = np.zeros((6, 3))
    
    p_0_0 = P[:, 0]  # ตำแหน่งของเฟรม 0
    p_0_1 = P[:, 1]  # ตำแหน่งของเฟรม 1
    p_0_2 = P[:, 2]  # ตำแหน่งของเฟรม 2
    p_0_e = p_e      # ตำแหน่งของ End Effector

    # แกนการหมุนของข้อต่อในเฟรมหลัก (Base Frame)
    z0 = np.array([0.0, 0.0, 1.0])  # แกน z ของเฟรม 0
    z1 = np.array([-np.sin(q[0]),np.cos(q[1]),0])           # แกน z ของเฟรม 1
    z2 = np.array([-np.sin(q[0]),np.cos(q[1]),0])          # แกน z ของเฟรม 2

    # คำนวณส่วนตำแหน่งของ Jacobian Matrix (J_v)
    J_v1 = np.cross(z0, (p_0_e - p_0_0))  # J_v สำหรับข้อต่อ 1
    J_v2 = np.cross(z1, (p_0_e - p_0_1))  # J_v สำหรับข้อต่อ 2
    J_v3 = np.cross(z2, (p_0_e - p_0_2))  # J_v สำหรับข้อต่อ 3

    # คำนวณส่วนมุมของ Jacobian Matrix (J_w)
    J_w1 = z0  # J_w สำหรับข้อต่อ 1
    J_w2 = z1  # J_w สำหรับข้อต่อ 2
    J_w3 = z2  # J_w สำหรับข้อต่อ 3
    
    # ใส่ค่าในเมทริกซ์ Jacobian
    J_e[0:3, 0] = J_v1
    J_e[0:3, 1] = J_v2
    J_e[0:3, 2] = J_v3

    J_e[3:6, 0] = J_w1
    J_e[3:6, 1] = J_w2
    J_e[3:6, 2] = J_w3

    # Replace small values with 0 to prevent precision issues
    J_e[abs(J_e) < 0.0001] = 0

    if print_jacobian:
        print("Jacobian Matrix at Base Frame:")
        print(J_e)
    
    return J_e

q = [0, 0, 0]  # Example joint angles
jacobian = endEffectorJacobianHW3(q, print_jacobian=True)
  
# #==============================================================================================================#
# #=============================================<คำตอบข้อ 2>======================================================#
# #code here
def checkSingularityHW3(q:list[float])->bool:
    J = endEffectorJacobianHW3(q)  # Get the Jacobian matrix
    det_J = np.linalg.det(J[:3, :])  # Calculate the determinant of the linear part of J
    print("\n" f"Determinant of Jacobian: {det_J}")
    # Check for singularity
    if abs(det_J) < 0.001:
        print("Singularity detected.")
        return True
    else:
        print("No singularity detected.")
        return False
    
# Example joint angles
q = [0, 0, 0]
is_singular = checkSingularityHW3(q)

    
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    J_e = endEffectorJacobianHW3(q) #Get Jacobian Matrix from endEffectorJacobianHW3 function
    J_ret = np.transpose(J_e) #Transpose Jacobian Matrix
    w_t = np.array(w) #Transpose wrench Matrix to 6x1
    tau = J_ret @ w_t #Find tau from Transpose Jacobian Matrix dot wrench Matrix
    return tau #Return Joint forces/torques due to w

q = [0,0,0] # กำหนดค่า q
w = [1.0,1.0,5.0,1.0,2.0,1.0] # กำหนดค่า w
print("\n""Computer Effort: ")
print(computeEffortHW3(q,w))
#==============================================================================================================#