#include <stdio.h>
#include <robotcontrol.h>

void initADC()
{
	if(rc_adc_init()){
        fprintf(stderr,"ERROR: failed to run rc_init_adc()\n");
        return;
    }
}

void ADC_Test()
{
	rc_adc_read_volt(0);
}

void ReadWrite()
{
	return;
}