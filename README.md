# FootTracer

一个基于薄膜压力传感器和IMU模块的运动姿态监测系统，能够测量人体足底压力实时动态分布和足部空间姿态，并在上位机中可视化显示。采用16路压阻式压力传感器和MPU6050模块作为主要传感器件。以模拟开关选通扫描的方式采集压力传感器数据，调用DMP库读取MPU6050姿态数据（[参考此处](https://github.com/Huffer342-WSH/MPU6050_I2C/tree/master/Middlewares/MPU6050_Motion_Driver)）。Python上位机采用差值法绘制压力分布热图，并用3D模型展示空间姿态数据。


## 项目结构
```
.
├── Host
│   └── 上位机程序
│
├── MEMSdesign
│   └── MCU程序
│
└── PCB
    └── PCB文件
```



