# UselessTank prototype v2
The second prototype of UselessTank.

# 实测参数
## HM-GM37-3429_12V_1:50_ENCODER
Encoder data of run 10 meters:
Left   : 40924 + (65536 * 2) = 171996
Right  : 42496 + (65536 * 2) = 173568
Average: 172782
Distance per encoder count: 0.057876mm

# 组装参数

## 外部设备电路连接

### 坦克控制板 -> 电机控制板
P12 / PB4  / TIM3_CH1(Remap) -> Motor2(Left) PWM
P11 / PB5  / TIM3_CH2(Remap) -> Motor1(Right) PWM
P10 / PA8                    -> Motor2(Left) IN1
P9  / PB15                   -> Motor2(Left) IN2
P8  / PC5                    -> Motor1(Right) IN1
P7  / PB12                   -> Motor1(Right) IN2

### 坦克控制板 -> 拉力传感器
P1 / PC8 -> 拉力AD DOUT
P2 / PC9 -> 拉力AD PD_SCK

## 连接线参数

电源线BAT: 
规格: 16AWG
长度(分电板->开关): 12cm

电源线12V: 
规格: 18AWG
长度(分电板->开关): 12cm
长度(开关->12V电源): 18cm
长度(电源输入负->电源输入地): 3cm
长度(电源输入地->电源输出负): 22cm
备注：电源输入负、输入地、输出负需要连接在一起，否则控制板GND和树莓派GND不共地，会造成串口通讯故障

电源线5V: 
规格: 18AWG
长度(分电板->开关): 12cm
长度(开关->5V电源): 25cm
长度(电源输入负->电源输入地): 3cm
长度(电源输入地->电源输出负): 20cm
备注：电源输入负、输入地、输出负需要连接在一起，否则控制板GND和树莓派GND不共地，会造成串口通讯故障

树莓派电源线: 
规格: 22AWG
长度: 30cm

树莓派串口线:
规格: 26AWG
长度: 25cm
颜色:
    树莓派TX->坦克控制板RX: 蓝
    树莓派RX->坦克控制板TX: 绿
	树莓派GND->坦克控制板GND: 黑

坦克控制板电源线: 
规格: 22AWG
长度: 15cm

电机控制板电源线: 
规格: 16AWG+18AWG
长度: 22cm(16AWG)+8cm(双股18AWG)

电机控制板数据线: 
规格: 26AWG
长度: 25cm
颜色: 
    VCC->Encoder 3.3V: 红
    PWM1->PB5(P11): 蓝
    INA1->PA8(P10): 绿
    INB1->PB15(P9): 白
    GND: 黑
    PWM2->PB4(P12): 蓝
    INA2->PA15(P14): 绿
    INB2->PB3(P13): 白

电机控制板->左右电机电源线: 
规格: 18AWG
长度: 15cm
备注：左电机电源线正负极反接

电机电源线：
规格: 18AWG
长度: 5cm

电机编码器数据线: 
规格: 26AWG
长度(左电机): 40cm
长度(右电机): 50cm

拉力传感器数据线: 
规格: 26AWG
长度: 25cm
颜色: 
    VCC: 红
    DOUT->PC8(P1): 蓝
    SCK->PC9(P2): 绿
    GND: 黑

功放板电源线: 
规格: DC5.5-2.1电源线
长度: 20cm

功放板喇叭线: 
规格: 22AWG
长度: 8cm

喇叭左: 
规格: 自带线
长度: 21cm

喇叭右: 
规格: 自带线
长度: 6cm
