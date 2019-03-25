单片机资源分配

Timer3 用于舵机定时器
Timer4 用于两个车轮的PWM输出
Timer2和Timer8 用于两个车轮的编码器计数


// 测试 Test
MotorDriver_L_Turn_Reverse();
MotorDriver_R_Turn_Reverse();

TIM_SetCompare3(TIM4,780); 
TIM_SetCompare4(TIM4,780); 





方法简要文件介绍

delay.c 有关延时的方法
