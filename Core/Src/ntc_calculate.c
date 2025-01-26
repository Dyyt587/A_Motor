#include "mcpwm.h"
// B=3380K, 10k , from 0 degree to 100 degree, table, the step is 5 degree
int NTC_Table[21]={2749,2218,1802,1472,1210,1000,831,694,583,491,416,354,302,259,223,193,167,146,127,111,98};
int NTC_R_Value;
short Get_NTC_Temperature(int NTC_R_Value)
{
	int i;
	int table_p=0;
	short temperature;
	for(i=0;i<21;i++)
	{
		if(NTC_R_Value>NTC_Table[i])
		{
			table_p=i;
			break;
		}
	}
	if(table_p<20)
	{
		if(table_p<1)
		{
			temperature=0;
		}
		else
		{
			temperature=(table_p-1)*50+50*(NTC_Table[table_p-1]-NTC_R_Value)/(NTC_Table[table_p-1]-NTC_Table[table_p]);
		}
			
	}
	else
	{
		temperature=1000;
	}
	return temperature;
}

