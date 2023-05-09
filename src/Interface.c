#include <stdio.h>
#include <rc/adc.h>
#include <rc/time.h>

void initADC()
{
	if(rc_adc_init()){
        fprintf(stderr,"ERROR: failed to run rc_init_adc()\n");
        return;
    }
}

void ADC_Test()
{
	rc_adc_read_volt(i)
}