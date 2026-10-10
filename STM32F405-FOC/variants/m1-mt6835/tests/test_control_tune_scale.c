#include "control.h"
#include "foc.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
uint32_t bsp_uart_millis(void){return 0;}
static float run(float ratio,int direction)
{
    foc.state=FOC_IDLE;foc.calibrated=foc.zero_ready=true;foc.voltage_scale=1;
    foc.calibration.direction=direction;foc.rpm=foc.iq=0;
    control_stop();assert(control_zero());control_step(100,0);
    foc_parameters_active=ratio>0;
    foc_parameters.kp_q=FOC_CURRENT_KP*ratio;
    assert(control_speed(30));foc.state=FOC_RUN;
    for(unsigned n=1;n<=20;++n)control_step(100+n*1000,0);
    return control_iq_ref();
}
int main(void)
{
    float normal=run(0,1),scaled=run(.25f,1),reverse=run(.25f,-1),larger=run(2,1);
    assert(normal>1 && fabsf(scaled-normal)<.001f);
    assert(fabsf(scaled+reverse)<.001f && fabsf(normal-larger)<.001f);
    puts("PASS: outer PI independent of electrical Kp, calibrated direction preserved");
}
