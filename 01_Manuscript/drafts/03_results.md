# 3. ผลการทดลอง

> หัวข้อนี้ยังไม่มีข้อมูลจากการทดลองจริง โครงสร้างด้านล่างเป็นกรอบที่วางไว้ตามแผนการวิเคราะห์
> เมื่อได้ข้อมูล CSV จากการทดลอง จะเติมค่าตัวเลขและกราฟเข้ามา

## 3.1 การติดตามมุมมอเตอร์ (Motor Angle Tracking)

รูปที่ [N] แสดงผลการเปรียบเทียบมุมมอเตอร์ระหว่างค่าเป้าหมาย (Target Setpoint) กับค่าจริงที่วัดจาก Encoder (Real Actual) ของมอเตอร์ทั้ง 8 ตัว ในช่วง [TODO: ระบุ] วินาทีแรกของการเดิน

[TODO: รูปที่ N -- กราฟ 4x2 (4 ขา x 2 มอเตอร์) แสดง Target Setpoint vs Real Actual จากสคริปต์ Sim-to-real_Motor-Tracking.py]

จากกราฟ พบว่ามอเตอร์ทุกตัวสามารถติดตามค่าเป้าหมายได้ [TODO: ระบุ -- ดี/มีความล่าช้า/มี overshoot] โดยค่า RMSE ของแต่ละมอเตอร์สรุปในตารางที่ [N]

ตารางที่ [N] ค่า RMSE ของมุมมอเตอร์ (Setpoint vs Actual)

| Leg | Motor   | RMSE (deg) | Max Error (deg) | Dominant Freq (Hz) |
|-----|---------|------------|-----------------|---------------------|
| FL  | theta1  | [TODO]     | [TODO]          | [TODO]              |
| FL  | theta2  | [TODO]     | [TODO]          | [TODO]              |
| FR  | theta1  | [TODO]     | [TODO]          | [TODO]              |
| FR  | theta2  | [TODO]     | [TODO]          | [TODO]              |
| RL  | theta1  | [TODO]     | [TODO]          | [TODO]              |
| RL  | theta2  | [TODO]     | [TODO]          | [TODO]              |
| RR  | theta1  | [TODO]     | [TODO]          | [TODO]              |
| RR  | theta2  | [TODO]     | [TODO]          | [TODO]              |

[TODO: วิเคราะห์ -- มอเตอร์ตัวใดมี RMSE สูงสุด/ต่ำสุด, สาเหตุที่เป็นไปได้]

## 3.2 การเปรียบเทียบเส้นทางปลายขา (Foot Trajectory Comparison)

### 3.2.1 เส้นทางจาก Encoder (Forward Kinematics) เทียบกับ Vision (ArUco)

รูปที่ [N] แสดงเส้นทางปลายขา (Foot Trajectory) เปรียบเทียบระหว่างตำแหน่งที่คำนวณจาก Encoder ผ่าน Forward Kinematics กับตำแหน่งจริงที่วัดจากระบบ ArUco Vision

[TODO: รูปที่ N -- กราฟ 2D Trajectory (X vs Y) เปรียบเทียบ Encoder FK vs Vision Actual จากสคริปต์ trajectory_analysis.py]

ค่า RMSE ของตำแหน่งปลายขาสรุปในตารางที่ [N]

ตารางที่ [N] ค่า RMSE ของตำแหน่งปลายขา

| Metric              | Value (mm) |
|---------------------|------------|
| RMSE X-axis         | [TODO]     |
| RMSE Y-axis         | [TODO]     |
| RMSE Combined (2D)  | [TODO]     |
| Max Error           | [TODO]     |
| X Overlap Range     | [TODO]     |
| Number of Points    | [TODO]     |

[TODO: วิเคราะห์ -- ทิศทางที่มีความคลาดเคลื่อนมากกว่า (X หรือ Y), ช่วง Trajectory ที่คลาดเคลื่อนมากที่สุด (ช่วงยกขา/ลงพื้น)]

### 3.2.2 เปรียบเทียบกับเส้นทางเป้าหมาย

รูปที่ [N] แสดงเส้นทางปลายขาจาก Vision เทียบกับเส้นทางเป้าหมาย (Target Trajectory) ที่สั่งจาก Gait Generator

[TODO: รูปที่ N -- กราฟ Target vs Vision Actual]

| Metric                        | Value (mm) |
|-------------------------------|------------|
| RMSE vs Target (Vision)       | [TODO]     |
| RMSE vs Target (Encoder FK)   | [TODO]     |
| Lift Height (Target)          | [TODO]     |
| Lift Height (Vision Actual)   | [TODO]     |
| Lift Height Difference        | [TODO]     |

## 3.3 การวิเคราะห์ความถี่ (Frequency Analysis)

การวิเคราะห์ FFT ของสัญญาณมุมมอเตอร์ช่วยยืนยันว่าการเดินมีความถี่ตรงกับที่ออกแบบไว้

[TODO: รูปที่ N -- กราฟ FFT Magnitude Spectrum ของมอเตอร์ตัวอย่าง]

ความถี่หลัก (Dominant Frequency) ของการเดินจาก Setpoint คือ [TODO] Hz ขณะที่ค่าจริงจาก Encoder คือ [TODO] Hz ซึ่ง [TODO: ตรงกัน/ต่างกัน]

## 3.4 ผลการชดเชยความคลาดเคลื่อน

> ส่วนนี้รอผลจากการพัฒนาแบบจำลองชดเชย (Data-Driven Compensation Model)

### 3.4.1 เปรียบเทียบก่อนและหลังชดเชย

[TODO: รูปที่ N -- กราฟ Trajectory ซ้อนทับ: Target / Before Compensation / After Compensation]

ตารางที่ [N] เปรียบเทียบ RMSE ก่อนและหลังชดเชย

| Metric              | Before (mm) | After (mm) | Improvement (%) |
|---------------------|-------------|------------|-----------------|
| RMSE X-axis         | [TODO]      | [TODO]     | [TODO]          |
| RMSE Y-axis         | [TODO]      | [TODO]     | [TODO]          |
| RMSE Combined       | [TODO]      | [TODO]     | [TODO]          |
| Max Error           | [TODO]      | [TODO]     | [TODO]          |

### 3.4.2 สถิติเชิงสรุป

[TODO: ตารางสรุปเชิงสถิติ -- ค่าเฉลี่ย, ส่วนเบี่ยงเบนมาตรฐาน, ค่าต่ำสุด, ค่าสูงสุด ของความคลาดเคลื่อนก่อน/หลังชดเชย]

## 3.5 การวิเคราะห์ Torque-to-Weight Ratio

ตารางที่ [N] เปรียบเทียบค่า TWR ของหุ่นยนต์ BLEGS กับหุ่นยนต์สี่ขาจากงานวิจัยอื่น

ตารางที่ [N] เปรียบเทียบ TWR กับหุ่นยนต์สี่ขาอื่น

| Robot           | Mass (kg) | Joint Torque (Nm) | TWR (Nm/kg) | DOF/Leg | Ref.   |
|-----------------|-----------|--------------------|-------------|---------|--------|
| BLEGS           | [TODO]    | [TODO]             | [TODO]      | 2       | --     |
| MIT Mini Cheetah| 9.0       | 17.0               | 1.89        | 3       | [TODO] |
| Stanford Doggo  | 4.8       | [TODO]             | [TODO]      | 2       | [TODO] |
| ODRI Solo       | 2.2       | [TODO]             | [TODO]      | 3       | [TODO] |
| ANYmal          | 30.0      | 40.0               | 1.33        | 3       | [TODO] |
| Unitree A1      | 12.0      | 33.5               | 2.79        | 3       | [TODO] |

[TODO: รูปที่ N -- กราฟ Bar Chart หรือ Scatter Plot เปรียบเทียบ TWR]

[TODO: ตรวจสอบค่า TWR ของหุ่นยนต์อ้างอิงให้ถูกต้องจากเอกสารต้นฉบับ ค่าในตารางเป็นค่าประมาณ]

---

## สิ่งที่ต้องเพิ่มเติมในหัวข้อนี้

- [TODO: ข้อมูล CSV] นำข้อมูลจากการทดลองจริงใส่ใน 03_Data/real/ และ 03_Data/vision/
- [TODO: รันสคริปต์] รัน trajectory_analysis.py และสคริปต์ใน 04_Code/analysis/ เพื่อได้ค่าตัวเลขจริง
- [TODO: กราฟทั้งหมด] สร้างกราฟจากข้อมูลจริงและบันทึกใน 02_Figures/
- [TODO: Compensation] ทำการทดลอง Data-Driven Compensation และบันทึกผล
- [TODO: TWR] คำนวณ TWR ของ BLEGS และตรวจสอบค่าอ้างอิงของหุ่นยนต์อื่นจากเอกสารจริง
- [TODO: ลำดับรูปและตาราง] กำหนดหมายเลขรูปและตารางให้ต่อเนื่องจากหัวข้อที่ 2
