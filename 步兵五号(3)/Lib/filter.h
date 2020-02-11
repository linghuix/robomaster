#ifndef __FILTER_H
#define __FILTER_H	 
//ÂË²¨Æ÷½á¹¹Ìå
typedef struct {
	double raw_value;
	double xbuf[5];
	double ybuf[5];
	double filtered_value;
}Filter_t;

//extern Filter_t MPUz50Hz, MPUy50Hz, MPUx50Hz;
//extern Filter_t PIDOUTPUT50Hz;
void Chebyshev50HzLPF(Filter_t *F);

//Ò»½×¿¨¶ûÂüÂË²¨
#define KALMAN_Q 0.02
#define KALMAN_R 7.000
double KalmanFilter2(double rawdata, double Process_Noise_Q, double Measure_Noise_R);



#endif




