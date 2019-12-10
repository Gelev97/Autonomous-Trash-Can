#include "show.h"

#include "sys.h"
  /**************************************************************************
作者：墨比斯科技
我的淘宝小店：https://moebius.taobao.com/
**************************************************************************/
unsigned char i;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
float Vol;
/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{			
//	//=============第1行显示3轴角度===============//	
	OLED_ShowString(0,0,(u8*)"X:");
	if(Pitch<0)		OLED_ShowNumber(15,0,Pitch+360,3,12);
	else			OLED_ShowNumber(15,0,Pitch,3,12);	

	OLED_ShowString(40,0,(u8*)"Y:");
	if(Roll<0)		OLED_ShowNumber(55,0,Roll+360,3,12);
	else			OLED_ShowNumber(55,0,Roll,3,12);	

	OLED_ShowString(80,0,(u8*)"Z:");
	if(Yaw<0)		OLED_ShowNumber(95,0,Yaw+360,3,12);
	else			OLED_ShowNumber(95,0,Yaw,3,12);		

	if(Run_Flag==0)//速度模式
	{		
		//=============显示电机A的状态=======================//	
		if( Target_A<0)			OLED_ShowString(00,10,(u8*)"-"),
								OLED_ShowNumber(15,10,-Target_A,5,12);
		else					OLED_ShowString(0,10,(u8*)"+"),
								OLED_ShowNumber(15,10, Target_A,5,12); 
		
		if( Encoder_A<0)		OLED_ShowString(80,10,(u8*)"-"),
								OLED_ShowNumber(95,10,-Encoder_A,5,12);
		else					OLED_ShowString(80,10,(u8*)"+"),
								OLED_ShowNumber(95,10, Encoder_A,5,12);
		//=============显示电机B的状态=======================//	
		if( Target_B<0)			OLED_ShowString(00,20,(u8*)"-"),
								OLED_ShowNumber(15,20,-Target_B,5,12);
		else					OLED_ShowString(0,20,(u8*)"+"),
								OLED_ShowNumber(15,20, Target_B,5,12); 
		
		if( Encoder_B<0)		OLED_ShowString(80,20,(u8*)"-"),
								OLED_ShowNumber(95,20,-Encoder_B,5,12);
		else					OLED_ShowString(80,20,(u8*)"+"),
								OLED_ShowNumber(95,20, Encoder_B,5,12);
		//=============显示电机C的状态=======================//	
		if( Target_C<0)			OLED_ShowString(00,30,(u8*)"-"),
								OLED_ShowNumber(15,30,-Target_C,5,12);
		else					OLED_ShowString(0,30,(u8*)"+"),
								OLED_ShowNumber(15,30, Target_C,5,12); 

		if( Encoder_C<0)		OLED_ShowString(80,30,(u8*)"-"),
								OLED_ShowNumber(95,30,-Encoder_C,5,12);
		else					OLED_ShowString(80,30,(u8*)"+"),
								OLED_ShowNumber(95,30, Encoder_C,5,12);	
		//=============显示电机D的状态=======================//	
		if( Target_D<0)			OLED_ShowString(00,40,(u8*)"-"),
								OLED_ShowNumber(15,40,-Target_D,5,12);
		else					OLED_ShowString(0,40,(u8*)"+"),
								OLED_ShowNumber(15,40, Target_D,5,12); 

		if( Encoder_D<0)		OLED_ShowString(80,40,(u8*)"-"),
								OLED_ShowNumber(95,40,-Encoder_D,5,12);
		else                 	OLED_ShowString(80,40,(u8*)"+"),
								OLED_ShowNumber(95,40, Encoder_D,5,12);
	}
	else if(Run_Flag==1)//位置模式
	{		
		//=============显示电机A的状态=======================//	
		if( Target_A<0)			OLED_ShowString(00,10,(u8*)"-"),
								OLED_ShowNumber(15,10,-Target_A,5,12);
		else                 	OLED_ShowString(0,10,(u8*)"+"),
								OLED_ShowNumber(15,10, Target_A,5,12); 

		if( Position_A<0)		OLED_ShowString(80,10,(u8*)"-"),
								OLED_ShowNumber(95,10,-Position_A,5,12);
		else					OLED_ShowString(80,10,(u8*)"+"),
								OLED_ShowNumber(95,10, Position_A,5,12);
		//=============显示电机B的状态=======================//	
		if( Target_B<0)			OLED_ShowString(00,20,(u8*)"-"),
								OLED_ShowNumber(15,20,-Target_B,5,12);
		else					OLED_ShowString(0,20,(u8*)"+"),
								OLED_ShowNumber(15,20, Target_B,5,12); 

		if( Position_B<0)		OLED_ShowString(80,20,(u8*)"-"),
								OLED_ShowNumber(95,20,-Position_B,5,12);
		else                 	OLED_ShowString(80,20,(u8*)"+"),
								OLED_ShowNumber(95,20, Position_B,5,12);
		//=============显示电机C的状态=======================//	
		if( Target_C<0)			OLED_ShowString(00,30,(u8*)"-"),
								OLED_ShowNumber(15,30,-Target_C,5,12);
		else					OLED_ShowString(0,30,(u8*)"+"),
								OLED_ShowNumber(15,30, Target_C,5,12); 

		if( Position_C<0)		OLED_ShowString(80,30,(u8*)"-"),
								OLED_ShowNumber(95,30,-Position_C,5,12);
		else                 	OLED_ShowString(80,30,(u8*)"+"),
								OLED_ShowNumber(95,30, Position_C,5,12);	
		//=============显示电机D的状态=======================//	
		if( Target_D<0)			OLED_ShowString(00,40,(u8*)"-"),
								OLED_ShowNumber(15,40,-Target_D,5,12);
		else                 	OLED_ShowString(0,40,(u8*)"+"),
								OLED_ShowNumber(15,40, Target_D,5,12); 

		if( Position_D<0)		OLED_ShowString(80,40,(u8*)"-"),
								OLED_ShowNumber(95,40,-Position_D,5,12);
		else                 	OLED_ShowString(80,40,(u8*)"+"),
								OLED_ShowNumber(95,40, Position_D,5,12);
	}
	//=============第五行显示电压=======================//
	if(Run_Flag==0)				OLED_ShowString(00,50,(u8*)"VELOCITY");
	else if(Run_Flag==1)		OLED_ShowString(00,50,(u8*)"POSITION");

	
	OLED_ShowString(88,50,(u8*)".");
	OLED_ShowString(110,50,(u8*)"V");
	OLED_ShowNumber(75,50,Voltage/100,2,12);
	OLED_ShowNumber(98,50,Voltage%100,2,12);
	if(Voltage%100<10) 	OLED_ShowNumber(92,50,0,2,12);
	//=============刷新=======================//
	OLED_Refresh_Gram();	
}
/**************************************************************************
函数功能：向APP发送数据
入口参数：无
返回  值：无
作    者：墨比斯科技
**************************************************************************/
void APP_Show(void)
{    
//	static u8 flag;
//	int app_2,app_3,app_4;
//	app_4=(Voltage/2-1110)*2/3;	
//	if(app_4>100)app_4=100;   //对电压数据进行处理
//	app_2=Move_X*0.7;  if(app_2<0)app_2=-app_2;			                   //对编码器数据就行数据处理便于图形化
//	app_3=Move_Y*0.7;  if(app_3<0)app_3=-app_3;
//	if(Run_Flag==1)		app_2=0,app_3=0;
//	flag=!flag;
//	if(PID_Send==1)//发送PID参数
//	{
//		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)RC_Velocity,(int)RC_Position,(int)Position_KP,(int)Position_KI,(int)Position_KD,(int)Velocity_KP,(int)Velocity_KI,0,0);//打印到APP上面	
//		PID_Send=0;	
//	}	
//	else	if(flag==0)// 
//	printf("{A%d:%d:%d:%d}$",(u8)app_2,(u8)app_3,app_4,0); //打印到APP上面
//	else
//	printf("{B%d:%d:%d:%d}$",(int)Pitch,(int)Roll,(int)Yaw,app_4);//打印到APP上面 显示波形
}

