#include "foc.h"
#include "control.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>
volatile uint32_t motor_sample_us;
volatile float motor_duty[3];
uint32_t bsp_uart_millis(void) { return motor_sample_us/1000u; }
static void sample(unsigned code)
{
    motor_sample_us=(motor_sample_us+100)&0xffffff;
    float angle=(code&32767u)*(360.0f/32768);
    foc_step(angle,12,1.65f,1.65f,0);
    foc_outer_step();
}
int main(void)
{
    foc_calibration_t calibration={0,1,7};
    foc_init(&calibration);control_stop();
    foc.zero_ready=true;foc.b_offset=foc.c_offset=1.65f;foc.state=FOC_IDLE;
    /* Fifteen-bit TLE steps stop fitting exactly into a float accumulated
     * past 4096 degrees. Twelve full turns then single-code motion reproduces
     * the real long-run indexing failure independently of the compensation. */
    for(unsigned n=0;n<=12*2048;++n)sample(n*16);
    double mean_speed=0;
    for(unsigned n=1;n<=4096;++n) {
        sample(n);
        if(n>3096)mean_speed+=control_speed_rpm()/1000;
    }
    for(unsigned n=0;n<10;++n)sample(4096); /* Latch the final 1 kHz position. */
    float raw=45, angle=control_absolute_deg();
    float error=fmodf(angle-raw+180,360)-180;
    printf("after 12 turns: angular error %.6f deg, speed %.6f RPM, position %.6f deg\n",error,mean_speed,control_position_deg());
    assert(fabsf(error)<.001f && fabsf(control_position_deg()-4365)<.001f);
    assert(fabs(mean_speed-18.310546875)<.02);
}
