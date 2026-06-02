#ifndef __FIT_CURVE__
#define __FIT_CURVE__
#include "AD5940.h"
#define rows 10
#define columns 10



void golay(int window_size, float buffer_size, float *poriginal_current, float *pgolay_current);
void moving_window(int window_size, int iteration, float buffer_size,float *pgolay_current, float *pAverage);
void fitcurve_fun(int len,float *px_val, float *py_val, float *pfit_line, float *psub_val, float pPeak_val);
void fit_cubic_fun(int len, float *px_val, float *py_val, float *pfit_line, float *psub_val);

#endif
