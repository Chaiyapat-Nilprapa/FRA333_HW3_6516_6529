## **FRA333 HW3 จัดทำโดย**
1. ชัยภัทร 65340500016
2. ธรา 65340500029

## **โปรดลง Library พวกนี้ก่อนรัน**
  - numpy
  - math
  - roboticstoolbox

**ลงโดยใช้ Command พวกนี้**
```bash
  - pip install numpy==1.24.4
```
```bash
  - pip install math
```
```bash
  - pip install roboticstoolbox
```

**Clone project**

**ลงโดยใช้ Command นี้**
```bash
  gh repo clone Chaiyapat-Nilprapa/FRA333_HW3_6516_6529
```

**หลังจากเข้าไปเราจะใช้**

  1. FRA333_HW3_6516_6529.py >> ไฟล์คำตอบ
  2. testScript.py >> ไฟล์ตรวจคำตอบ
  3. HW3_utils.py >> Forward Kinematic function ที่บริษัทให้มา

## **เมื่อรันไฟล์คำตอบ (FRA333_HW3_6516_6529.py)**

  Output ที่ได้ออกมา

![Answer]("Answer.png")

  - เมทริกซ์ Jacobian ที่ได้จากการเทียบจาก Base
  - Det ของ Jacobian เพื่อใช้หา Singularity
  - จากภาพเนื่องจาก Det ของ Jacobian มีค่ามากกว่า 0.01 จึงไม่เกิด Singularity
  - Effort ที่เกิดขึ้น (ค่า Tau ที่แต่ละข้อต่อ)

  Input ที่ใส่เข้าไป
  



