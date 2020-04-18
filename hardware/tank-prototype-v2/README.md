# UselessTank prototype v2
The second prototype of UselessTank.

# 实测参数

## HM-GM37-3429-12V-50-ENCODER
Encoder count of run 10 meters:
Left   : 40924 + (65536 * 2) = 171996
Right  : 42496 + (65536 * 2) = 173568
Average: 172782
Average per meter: 17278.2
Distance per encoder count: 0.057876mm

Encoder diff of 10 full left turn:
Left : -134712
Right: 141688
Total(-Left+Right): 276400
Diff per full turn: 27640

## CHR-GM37-3429-12V-30-ENCODER
Encoder count of run 10 meters:
Left   : 39541 + (65536 * 1) = 105077
Right  : 39490 + (65536 * 1) = 105026
Average: 105051.5
Average per meter: 10505.15
Distance per encoder count: 0.095191mm

Encoder diff of 10 full left turn:
Left : -69265
Right: 78666
Total(-Left+Right): 147931
Diff per full turn: 14793.1

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

电源线BAT(分电板VBAT-IN -> 主开关):
规格: 16AWG
长度: 12cm
颜色:
    分电板VBAT-IN(红) -> 主开关(红)
    分电板VBAT-IN(黑) -> 主开关(黑)
备注: 主开关一端焊接XT60-Male插头，分电板VBAT-IN端焊接XT60-Female插头

电源线12V(分电板VBAT-OUT -> 12V开关):
规格: 18AWG
长度: 12cm
颜色:
    分电板VBAT-OUT(红) -> 12V开关(蓝)
    分电板VBAT-OUT(黑) -> 12V开关(黑)
备注: 分电板VBAT-OUT端焊接XT60-Male插头

电源线12V(12V开关 -> 12V电源):
规格: 18AWG
长度: 18cm
颜色:
    12V开关(绿) -> 12V电源(红)
    12V开关(黑) -> 12V电源(黑)
备注: VBAT-OUT黑、开关黑、电源黑焊接在一起

电源线12V(12V电源 -> 分电板12V-IN):
规格: 18AWG
长度: 16cm
备注: 分电板12V-IN端焊接XT60-Female插头

电源线12V(12V电源输入负 -> 12V电源输入地):
规格: 18AWG
长度: 3cm

电源线12V(12V电源输入地 -> 12V电源输出负):
规格: 18AWG
长度: 22cm
备注: 电源输入负、输入地、输出负需要连接在一起，否则控制板GND和树莓派GND不共地，会造成串口通讯故障

电源线5V(分电板VBAT-OUT -> 5V开关):
规格: 18AWG
长度: 12cm
颜色:
    分电板VBAT-OUT(红) -> 5V开关(蓝)
    分电板VBAT-OUT(黑) -> 5V开关(黑)
备注: 分电板VBAT-OUT端焊接XT60-Male插头

电源线5V(5V开关 -> 5V电源):
规格: 18AWG
长度: 25cm
颜色:
    5V开关(绿) -> 12V电源(红)
    5V开关(黑) -> 12V电源(黑)
备注: VBAT-OUT黑、开关黑、电源黑焊接在一起

电源线5V(5V电源 -> 分电板5V-IN):
规格: 18AWG
长度: 22cm
备注: 分电板5V-IN端焊接XT60-Female插头

电源线5V(5V电源输入负 -> 5V电源输入地):
规格: 18AWG
长度: 3cm

电源线5V(5V电源输入地 -> 5V电源输出负):
规格: 18AWG
长度: 20cm
备注: 电源输入负、输入地、输出负需要连接在一起，否则控制板GND和树莓派GND不共地，会造成串口通讯故障

树莓派电源线(分电板5V-OUT -> 树莓派):
规格: 22AWG
长度: 30cm
备注: 分电板5V-OUT端焊接XT30-Male插头

树莓派串口线(树莓派 -> 坦克控制板):
规格: 26AWG
长度: 25cm
颜色:
    树莓派TX -> 坦克控制板RX: 蓝
    树莓派RX -> 坦克控制板TX: 绿
	树莓派GND -> 坦克控制板GND: 黑

坦克控制板数据线(分电板CTRL -> 坦克控制板):
规格: 22AWG
长度: 20cm
颜色:
    1(GND) -> GND: 黑
    2(VCC_MCU) -> 5V: 红
    3: NC
    4(PWR_CTRL) -> PC10: 蓝
    5(PWR_BTN_DET) -> PC11: 绿
    6(PWR_LED_CTRL) -> PC12: 白
    7(STOP_BTN_DET) -> PB11: 橙
    8(STOP_BTN_G_CTRL) -> PD2: 黄
    9: NC
    10(VCC_BAT) -> PWR+: 红(15cm)
    11(GND_BAT) -> PWR-: 黑(15cm)

坦克控制板电源线(坦克控制板 -> 分电板VBAT-OUT):
规格: 22AWG
长度: 15cm
备注: 分电板VBAT-OUT端焊接XT30-Male插头

电机控制板电源线(电机控制板 -> 分电板VBAT-OUT):
规格: 16AWG+18AWG
长度: 22cm(16AWG)+8cm(双股18AWG)
备注: 分电板VBAT-OUT端焊接XT60-Male插头

电机控制板数据线(电机控制板 -> 坦克控制板):
规格: 26AWG
长度: 25cm
颜色:
    VCC -> Encoder 3.3V: 红
    PWM1 -> PB5(P11): 蓝
    INA1 -> PA8(P10): 绿
    INB1 -> PB15(P9): 白
    GND: 黑
    PWM2 -> PB4(P12): 蓝
    INA2 -> PA15(P14): 绿
    INB2 -> PB3(P13): 白

电机电源线(电机控制板 -> 左电机):
规格: 18AWG
长度: 15cm
备注:
    一端焊接XT30-Female插头
    左电机电源线连接到控制板接线柱时，正负极反接

电机电源线(电机控制板 -> 右电机):
规格: 18AWG
长度: 15cm
备注: 一端焊接XT30-Female插头

电机电源线(左电机):
规格: 18AWG
长度: 8cm
备注:
    电机金属壳有缺口处对应电极为正极
    一端焊接XT30-Male插头

电机电源线(右电机):
规格: 18AWG
长度: 8cm
备注:
    电机金属壳有缺口处对应电极为正极
    一端焊接XT30-Male插头

电机编码器数据线(电机控制板 -> 左右电机):
规格: 26AWG
长度(左电机): 40cm
长度(右电机): 50cm
备注: 左右电机编码器数据线缠绕在一起

拉力传感器数据线(拉力传感器 -> 坦克控制板):
规格: 26AWG
长度: 25cm
颜色:
    VCC: 红
    DOUT -> PC8(P1): 蓝
    SCK -> PC9(P2): 绿
    GND: 黑

功放板电源线(功放板 -> 分电板12V-OUT):
规格: DC5.5-2.1电源线
长度: 20cm
备注: 分电板12V-OUT端焊接XT30-Male插头

功放板喇叭线(功放板 -> 左右喇叭):
规格: 22AWG
长度: 8cm
颜色（从靠近DC插座的接线柱开始数）:
    1 -> 左喇叭红
    2 -> 左喇叭黑
    3 -> 右喇叭黑
    4 -> 右喇叭红
备注: 功放板上画的喇叭正负极似乎有问题，按照功放板标注的喇叭正负极接的话，两个喇叭相位是反的，需要调换一个喇叭正负极后声音才正常

喇叭左:
规格: 自带线
长度: 21cm

喇叭右:
规格: 自带线
长度: 6cm
