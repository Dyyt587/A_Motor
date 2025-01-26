#include "main.h"
#include "delay.h"
#include "mcpwm.h"

void Init_Driver_State(void)
{
	control_word.all=0x06;
	target_position=0;
	profile_target_position_b=profile_target_position;
}
void Init_System_Parameter(void)
{	
	software_version = SOFTWARE_VERSION;
	Modbus_Addr_Base=0;
	RS485_Addr=1;
	RS485_Protocol=1;
	RS485_Baudrate=38400;
	
	over_voltage=500;     //高压报警值：单位0.1V，500表示50V
	under_voltage=100;		//低压报警值：单位0.1V，100表示10V
	chop_voltage=400;
	
	over_temperature=750;
	
	Driver_IIt_Filter=2;
	Driver_IIt_Current=15000;
	
	Driver_IIt_Filter_DC=16;
	Driver_IIt_Current_DC=25000;
	
}
void Init_Motor_Parameter(void)
{
	commutation_current=2000; //找磁场零位电流，2000表示2A，用于给到Q轴目标电流对齐磁场的。
	commutation_mode=1;					//找磁场零位模式，0表示抖动找零位，1表示通过单圈绝对值位置找零位
	commutation_time=1000;			//抖动找零位时的保持时间，1000表示1秒
	feedback_resolution=16384;    //编码器分辨率
	poles_num=2;                  //电机极对数
	motor_peak_current=4000;      //电机最大电流
	motor_code=0;
	hall_phase[1]=2260;         //霍尔相位数据，用于霍尔反馈闭环时的，不同电机需要校正这个数据才能正常运行
	hall_phase[2]=64;
	hall_phase[3]=1138;
	hall_phase[4]=4260;
	hall_phase[5]=3247;
	hall_phase[6]=5355;
	ENC_Z_Offset=2680;
	hall_phase_offset=708;
	phase_dir=1;            //矢量控制的相位方向，1或-1，由于调整适应电机方向的。
	vel_dir=-1;
	tamagawa_dir=1;        //通讯编码器方向，1或-1
	tamagawa_offset=0;      //通讯编码器位置偏移
	feedback_type=4;       	//编码器反馈类型，1表示增量式编码器，4表示多摩川通讯式编码器

}

void Init_Control_Parameter(void)
{
	operation_mode=2;      //工作模式：1表示有加减速的位置模式，2表示速度模式，3表示有加减速的速度模式，4表示力矩模式，0表示开环速度模式
	control_word.all=0x06;  //控制字：0x06表示去使能；0x0F表示使能；0x86表示清错
	Iq_demand=0;          	//内部Q轴电流指令，单位是mA，只读
	speed_demand=0;					//内部速度指令，单位是0.001转每秒，只读
	position_demand=0;			//内部位置指令，单位是count，只读
	target_Iq=0;						//Q轴电流指令，单位是mA，读写
	target_speed=0;					//速度指令，单位是0.001转每秒，读写
	target_position=0;			//位置指令，单位是count，读写
	
	kcp=70;      //电流环比例增益kp
	kci=1;       //电流环积分增益ki
	kci_sum_limit=10000000;   //电流环积分限制
	current_in_lpf_a=1000;
	current_out_lpf_a=600;
	Ilim=3000;     //输出电流限制，单位是mA
	
	kvp=200;      //速度环比例增益kp
	kvi=1;					//速度环积分增益ki
	kvi_sum_limit=5000;  //速度环积分限制
	low_pass_filter_on=1;
	real_speed_filter_num=4;
	speed_in_lpf_a=500;
	speed_out_lpf_a=300;
	vel_lim=50000;   //速度限制，单位是0.001转每秒
	
	kpp=10;		//位置环比例增益kp
	kpi=1;		//位置环积分增益ki
	kpi_sum_limit=100;  //位置环积分限制
	position_in_lpf_a=1000;
	position_out_lpf_a=1000;
	
	profile_target_position=0;  //带加减速的目标位置
	profile_speed=30000;    //带加减速的最大速度，，单位0.001转每秒
	profile_acce=100000;     //减速度，单位0.001转每秒平方
	profile_dece=100000;     //减速度，单位0.001转每秒平方
	
	searching_speed=1000;
	motion_out_lpf_a=1000;
	
	auto_reverse_p_time=0;    //自动正反转的正转时间，单位毫秒
	auto_reverse_n_time=0;		//自动正反转的反转时间
	auto_p_pos=0;  //自动正反转的正向目标值
	auto_n_pos=0;	//自动正反转的反向目标值
	auto_switch_on=0; //上电自动锁轴，1表示上电自动锁轴
	
	
}
void Exchange_motor_code(void)
{
	char fb,pp,re,cr;
	fb = motor_code/10000;
	pp = (motor_code%10000)/1000;
	re = (motor_code%1000)/10;
	cr = motor_code%10;
	if(fb!=0)
	{
		feedback_type=fb;
	}
	if(pp!=0)
	{
		poles_num=pp;
	}
	if(cr!=0)
	{
		motor_rated_current=cr*2000;
		motor_peak_current=cr*4000;
	}
	switch(re)
	{
		case 1:
			feedback_resolution=1000;
			break;
		case 2:
			feedback_resolution=2000;
			break;
		case 3:
			feedback_resolution=2048;
			break;
		case 4:
			feedback_resolution=4000;
			break;
		case 5:
			feedback_resolution=4096;
			break;
		case 8:
			feedback_resolution=8000;
			break;
		case 9:
			feedback_resolution=8192;
			break;
		case 10:
			feedback_resolution=10000;
			break;
		case 16:
			feedback_resolution=16384;
			break;
		case 32:
			feedback_resolution=32768;
			break;
		case 65:
			feedback_resolution=65536;
			break;
		case 13:
			feedback_resolution=131072;
			break;
		case 26:
			feedback_resolution=262144;
			break;
	}
	
}
