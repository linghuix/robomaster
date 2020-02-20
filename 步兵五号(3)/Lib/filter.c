#include "filter.h"
//使用时直接在下面定义结构体
//Filter_t MPUz50Hz, MPUy50Hz, MPUx50Hz;
//Filter_t PIDOUTPUT50Hz;
//Filter_t TX2filter;

double NUM[5] = {
	0.0001298496353869,0.0005193985415477,0.0007790978123215,0.0005193985415477,
	0.0001298496353869
};
double DEN[5] = {
	1,   -3.607896169129,    4.979470803751,   -3.110763682983,
	0.7415201473558
};

//**********************************************************************
//* 功能： 切比雪夫五阶截止频率50hz滤波器                              
//* 参数： Filter_t：滤波器结构体                                      
//* 返回值：无                                                         
//* 说明;对于噪声的滤波表现良好，但是存在明显的相移,导致控制有延时     
//**********************************************************************
void Chebyshev50HzLPF(Filter_t *F)
{
	int i;
	for (i = 4; i>0; i--)
	{
		F->ybuf[i] = F->ybuf[i - 1];
		F->xbuf[i] = F->xbuf[i - 1];
	}
	F->xbuf[0] = F->raw_value;
	F->ybuf[0] = NUM[0] * F->xbuf[0];
	for (i = 1; i<5; i++)
	{
		F->ybuf[0] = F->ybuf[0] + NUM[i] * F->xbuf[i] - DEN[i] * F->ybuf[i];
	}
	F->filtered_value = F->ybuf[0];
}

//**********************************************************************
//* 功能： 卡尔曼一阶滤波器                                            
//* 参数：rawdata：未处理数据                                          
//* 参数： Process_Noise_Q：过程噪声系数                               
//* 参数： Measure_Noise_R：测量噪声系数                               
//* 返回值：处理后数据                                                 
//* 说明：对于毛刺信号的处理良好，相移不明显，但是幅值损失比较大,
//        可通过增加控制功率来弥补这部分损失
//**********************************************************************
double KalmanFilter2(double rawdata, double Process_Noise_Q, double Measure_Noise_R)
{
	double R = Measure_Noise_R;
	double Q = Process_Noise_Q;

	static double x_last;
	double x_mid = x_last;
	double x_now;

	static double p_last;
	double p_mid;
	double p_now;

	double kg;

	x_mid = x_last;                          //x_last=x(k-1|k-1),x_mid=x(k|k-1)
	p_mid = p_last + Q;                      //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声

	//卡尔曼滤波的五个重要公式
	kg = p_mid / (p_mid + R);                //kg为kalman filter，R 为噪声
	x_now = x_mid + kg * (rawdata - x_mid);  //估计出的最优值
	p_now = (1 - kg)*p_mid;                  //最优值对应的covariance
	p_last = p_now;                          //更新covariance 值
	x_last = x_now;                          //更新系统状态值

	return x_now;
}
