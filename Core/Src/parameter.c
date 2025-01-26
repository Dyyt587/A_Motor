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
	
	over_voltage=500;     //��ѹ����ֵ����λ0.1V��500��ʾ50V
	under_voltage=100;		//��ѹ����ֵ����λ0.1V��100��ʾ10V
	chop_voltage=400;
	
	over_temperature=750;
	
	Driver_IIt_Filter=2;
	Driver_IIt_Current=15000;
	
	Driver_IIt_Filter_DC=16;
	Driver_IIt_Current_DC=25000;
	
}
void Init_Motor_Parameter(void)
{
	commutation_current=2000; //�Ҵų���λ������2000��ʾ2A�����ڸ���Q��Ŀ���������ų��ġ�
	commutation_mode=1;					//�Ҵų���λģʽ��0��ʾ��������λ��1��ʾͨ����Ȧ����ֵλ������λ
	commutation_time=1000;			//��������λʱ�ı���ʱ�䣬1000��ʾ1��
	feedback_resolution=16384;    //�������ֱ���
	poles_num=2;                  //���������
	motor_peak_current=4000;      //���������
	motor_code=0;
	hall_phase[1]=2260;         //������λ���ݣ����ڻ��������ջ�ʱ�ģ���ͬ�����ҪУ��������ݲ�����������
	hall_phase[2]=64;
	hall_phase[3]=1138;
	hall_phase[4]=4260;
	hall_phase[5]=3247;
	hall_phase[6]=5355;
	ENC_Z_Offset=2680;
	hall_phase_offset=708;
	phase_dir=1;            //ʸ�����Ƶ���λ����1��-1�����ڵ�����Ӧ�������ġ�
	vel_dir=-1;
	tamagawa_dir=1;        //ͨѶ����������1��-1
	tamagawa_offset=0;      //ͨѶ������λ��ƫ��
	feedback_type=4;       	//�������������ͣ�1��ʾ����ʽ��������4��ʾ��Ħ��ͨѶʽ������

}

void Init_Control_Parameter(void)
{
	operation_mode=2;      //����ģʽ��1��ʾ�мӼ��ٵ�λ��ģʽ��2��ʾ�ٶ�ģʽ��3��ʾ�мӼ��ٵ��ٶ�ģʽ��4��ʾ����ģʽ��0��ʾ�����ٶ�ģʽ
	control_word.all=0x06;  //�����֣�0x06��ʾȥʹ�ܣ�0x0F��ʾʹ�ܣ�0x86��ʾ���
	Iq_demand=0;          	//�ڲ�Q�����ָ���λ��mA��ֻ��
	speed_demand=0;					//�ڲ��ٶ�ָ���λ��0.001תÿ�룬ֻ��
	position_demand=0;			//�ڲ�λ��ָ���λ��count��ֻ��
	target_Iq=0;						//Q�����ָ���λ��mA����д
	target_speed=0;					//�ٶ�ָ���λ��0.001תÿ�룬��д
	target_position=0;			//λ��ָ���λ��count����д
	
	kcp=70;      //��������������kp
	kci=1;       //��������������ki
	kci_sum_limit=10000000;   //��������������
	current_in_lpf_a=1000;
	current_out_lpf_a=600;
	Ilim=3000;     //����������ƣ���λ��mA
	
	kvp=200;      //�ٶȻ���������kp
	kvi=1;					//�ٶȻ���������ki
	kvi_sum_limit=5000;  //�ٶȻ���������
	low_pass_filter_on=1;
	real_speed_filter_num=4;
	speed_in_lpf_a=500;
	speed_out_lpf_a=300;
	vel_lim=50000;   //�ٶ����ƣ���λ��0.001תÿ��
	
	kpp=10;		//λ�û���������kp
	kpi=1;		//λ�û���������ki
	kpi_sum_limit=100;  //λ�û���������
	position_in_lpf_a=1000;
	position_out_lpf_a=1000;
	
	profile_target_position=0;  //���Ӽ��ٵ�Ŀ��λ��
	profile_speed=30000;    //���Ӽ��ٵ�����ٶȣ�����λ0.001תÿ��
	profile_acce=100000;     //���ٶȣ���λ0.001תÿ��ƽ��
	profile_dece=100000;     //���ٶȣ���λ0.001תÿ��ƽ��
	
	searching_speed=1000;
	motion_out_lpf_a=1000;
	
	auto_reverse_p_time=0;    //�Զ�����ת����תʱ�䣬��λ����
	auto_reverse_n_time=0;		//�Զ�����ת�ķ�תʱ��
	auto_p_pos=0;  //�Զ�����ת������Ŀ��ֵ
	auto_n_pos=0;	//�Զ�����ת�ķ���Ŀ��ֵ
	auto_switch_on=0; //�ϵ��Զ����ᣬ1��ʾ�ϵ��Զ�����
	
	
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
