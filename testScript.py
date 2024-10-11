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
from HW3_utils import FKHW3
import roboticstoolbox as rtb

from spatialmath import SE3
from math import pi
import matplotlib.pyplot as plt


#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
def testscript_1(q: list[float]) -> np.ndarray:
  d1 = 0.0892
  a2 = -0.425
  a3 = -0.39243
  d4 = 0.109
  d5 = 0.093
  d6 = 0.082
  # กำหนดค่ามุมข้อต่อเริ่มต้น

  # เรียกใช้งานฟังก์ชัน FKHW3 เพื่อหาค่าการหมุนและตำแหน่ง
  R, P, R_e, p_e = FKHW3(q)

  # ปรับค่าพารามิเตอร์ DH ให้ตรงกับข้อมูลที่ให้มา
  robot = rtb.DHRobot(
      [
          rtb.RevoluteMDH(d= d1 , offset = pi),
          rtb.RevoluteMDH(alpha = pi/2 ),
          rtb.RevoluteMDH(a=a2),
      ],
      name = "RRR_Robot"
      )


  tool_frame = SE3((a3-d6),-d5,d4) @ SE3.RPY(0.0,-pi/2,0.0) #Transformation Matrix from last joint to end-effector
  robot.tool = tool_frame #add End-effector to robot
  # คำนวณ Jacobian Matrix ใน Base Frame
  J_base = robot.jacob0(q)  # ใช้มุมข้อต่อ 3 ข้อที่กำหนดเท่านั้น
  J_base[abs(J_base) < 0.0001] = 0
  return J_base

q = [0, 0, 0]
print("Jacobian Matrix at ToolBox:")
print(testscript_1(q))
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here

#==============================================================================================================#