# VisualServo
VisualServo
Copyright (C) 20018 - 2020 by XieMingyou. All rights reserved.

这是本人自主开发的机器人视觉伺服程序，本人保留版权。

程序功能简介：  
- 实现基于位置(PBVS)和基于图像(IBVS)的视觉伺服：利用ViSP库，进行相机畸变校正、机器人手眼标定，通过识别AprilTag进行位姿估计，建立机器人模型，计算雅克比矩阵和控制速度，采用自主研发的运动控制器实现机器人跟随AprilTag在六个位姿分量的伺服，收敛快，伺服精度达0.1mm级别；发表论文：RGB-D Image Processing Algorithm for Target Recognition and Pose Estimation of Visual Servo System（SCI三区）。

打开PBVS程序：  
servoKawasaki\servoKawasakiPBVS\servoKawasakiPBVS.sln

打开IBVS程序：  
servoKawasaki\servoKawasakiIBVS\servoKawasakiIBVS.sln
