# file สำหรับตรวจคำตอบ
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
import FRA333_HW3_6516_6529 as hand
from HW3_utils import FKHW3
from roboticstoolbox import robot
from spatialmath import SE3
from math import pi

# กำหนดระยะห่างแต่ละข้อ
d1 = 0.0892
a2 = -0.425
a3 = -0.39243
d4 = 0.109
d5 = 0.093
d6 = 0.082

# ปรับค่าพารามิเตอร์ DH ให้ตรงกับข้อมูลที่ให้มา
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d= d1 , offset = pi),
        rtb.RevoluteMDH(alpha = pi/2 ),
        rtb.RevoluteMDH(a=a2),
    ],
    name = "RRR_Robot"
    )


#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
def testscript_1(q: list[float]) -> np.ndarray:

  # เรียกใช้งานฟังก์ชัน FKHW3 เพื่อหาค่าการหมุนและตำแหน่ง
  R, P, R_e, p_e = FKHW3(q)

  tool_frame = SE3((a3-d6),-d5,d4) @ SE3.RPY(0.0,-pi/2,0.0) #Transformation Matrix from last joint to end-effector
  robot.tool = tool_frame #add End-effector to robot
  
  # คำนวณ Jacobian Matrix ใน Base Frame
  
  J_base = robot.jacob0(q)  # ใช้มุมข้อต่อ 3 ข้อที่กำหนดเท่านั้น
  J_base[abs(J_base) < 0.0001] = 0
  return J_base

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
def testscript_2(robot: robot, q: list[float]) -> np.ndarray:
    # รับ Jacobian จาก Robotic Toolbox
    J = robot.jacob0(q)  # jacob0 จะคำนวณ Jacobian ที่ end-effector ในกรอบการอ้างอิงโลก

    # ดึงส่วน Linear Velocity ของ Jacobian Matrix
    J_Velocity = J[:3, :]  # ได้เมทริกซ์ขนาด 3x6 (หรือขึ้นกับจำนวน DOF ของหุ่นยนต์)

    # คำนวณ Determinant ของ J_Velocity
    if J_Velocity.shape[0] == J_Velocity.shape[1]:  # ตรวจสอบว่าเป็นเมทริกซ์กำลังสอง
        det_J_Velocity = np.linalg.det(J_Velocity)
    else:
        print("\n""Cannot compute determinant, J_Velocity is not square matrix.")
        det_J_Velocity = None

    # ตรวจสอบสภาวะ Singularity
    if det_J_Velocity is not None and abs(det_J_Velocity) < 0.001:
        print("\n""Singularity", det_J_Velocity)
    elif det_J_Velocity is not None:
        print("\n""Normal =", det_J_Velocity)

    return det_J_Velocity

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
def testscript_3(q: list[float], w: list[float], robot: rtb.DHRobot) -> np.ndarray:
    # รับ Jacobian จาก Robotic Toolbox
    J_e = robot.jacob0(q)  # jacob0 ให้ Jacobian ที่ end-effector ในกรอบการอ้างอิงโลก

    # ทรานสโพส Jacobian matrix
    J_ret = np.transpose(J_e)  # ทรานสโพส Jacobian matrix

    # แปลง wrench (แรงที่กระทำต่อ end-effector) เป็น numpy array ขนาด 6x1
    w_t = np.array(w).reshape(6, 1)  # ตรวจสอบให้แน่ใจว่า w เป็นเวกเตอร์ 6x1

    # คำนวณ tau โดย dot product ของ Transposed Jacobian และ Wrench
    tau = J_ret @ w_t  # tau คือแรง/ทอร์กที่ข้อต่อเนื่องจากแรง/ทอร์ก w ที่ end-effector
    tau = tau.flatten()
    return tau # คืนค่า tau เป็นเวกเตอร์ 1 มิติ
#==============================================================================================================#

q = hand.q #กำหนดให้ค่า q มีค่าเท่ากับ q จากโค้ด FRA333_HW3_6516_6529
w = hand.w #กำหนดให้ค่า w มีค่าเท่ากับ w จากโค้ด FRA333_HW3_6516_6529

print(testscript_1(q)) #ปริ้นค่าที่ได้จาก Function testscript 1 

determinant_velocity = testscript_2(robot, q) #ใ ห้ determinant_velocity = ค้าที่ได้มาจาก testscript 2
print("Determinant of the linear part of the Jacobian:", determinant_velocity)#ปริ้นค่าที่ได้จาก Function testscript 2

tau = testscript_3(q, w, robot) # ให้ tau = ค้าที่ได้มาจาก testscript 3
print("\n""Joint torques/forces due to the wrench applied at the end-effector:", tau)# ปริ้นค่าที่ได้จาก Function testscript 3