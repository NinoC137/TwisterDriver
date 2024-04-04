# Twister_Driver

一款双路FOC控制板, 额外配置了两个舵机驱动口与一系列外设.



在软件编写部分, 实现面向对象式编程架构, 建立FOC电机对象, 提高代码复用能力

尽可能实现了硬件抽象, 留出硬件API接口, 提高代码的可移植性



在代码上, 用户仅需自行填充

1. PWM的写入api, 形参为0~100%的参数(即0~1的float变量).
2. 角度的读取api, 将读取到的角度存入FOC对象中的angle_pi即可.
3. 电流的读取api, 将读取到的电流与零点偏置电流存入FOC对象中的current数组即可.



## 目前的功能实现

- [x] 开环速度控制 openLoop control
- [x] 角度环单环闭环控制 angular single PID feed back control
- [x] 速度环单环闭环控制 speed single PID feed back control
- [x] 电流环单环闭环控制 current single PID feed back control
- [x] 角度-速度环串级PID控制 angular-speed serial PID feed back control
- [x] 速度-电流环串级PID控制 speed-current serial PID feed back control
- [x] 角度-电流环串级PID控制 angular-current serial PID feed back control
- [x] **角度-速度-电流环串级PID控制** angular-speed-current serial PID feed back control

----

- [x] 分别使用SPWM与SVPWM完成上述控制 use SPWM and SVPWM complete these control
- [x] 双路FOC驱动 2-channel FOC control
- [x] 双路FOC完成上述控制 let 2-channel FOC complete these control
- [ ] GUI绘制与显示系统状态
- [x] 串口/485总线控制协议 serial/485 communication
- [ ] CAN总线控制协议 CAN communication



## 硬件功能清单

### FOC控制

使用drv8313驱动IC, 可以驱动8~24V的电机, 工作电流最大为2.5A.

使用ina199芯片, 配置了两路三相的电流采样, 直接输出模拟量.

配置了编码器输入接口, 采用IIC通信的方式读取数据, 主要适配于MT6701编码器.

### 通信

预留了CAN总线接口, TypeC-CH340-USART1接口, USART3接口, 其中USART3可自由配置使用场景. 

### 外设

配置了一个EC11编码器, 接入定时器3的编码器通道, 及当做按键Key6.

配置了5个物理按键, 可自由配置触发情景.

配置了一个SPI-LCD 0.96寸屏幕, 屏幕的驱动芯片为st7735.

配置了一个可编程的LED灯, 用于指示运行状态等等

### 舵机驱动

舵机连接了定时器的两个通道, 5V电压驱动.



## 物料清单

主控芯片	---	STM32G431

电机驱动	---	drv8313PWPR

电流采样	---	INA199A

通信芯片	---	CH340N(串口)	TJA1050(CAN)

电源管理	---	8~24V DC-DC压降	TPS54202DDCR

显示屏	---		ips 0.96寸TFT-LCD显示屏 ST7735驱动



连接器, 电容电阻电感及保护二极管等等此处不列举



电机型号:		2804云台电机, 极对数7, 320KV, 额定电压12V 默认连接线为SH1.0

编码器型号:	MT6701