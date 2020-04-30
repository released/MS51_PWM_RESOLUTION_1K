# MS51_PWM_RESOLUTION_1K
 MS51_PWM_RESOLUTION_1K

update @ 2020/04/30

- Set P0.0 (PWM0_CH3) Freq as 375Hz , change duty with 0.1% (resolution 1000) per 100 ms

- Due to MS51 unable to set customize PWM CNR , in order to have exact division  , Freq set to 375

	- PWM_CNR formula : (SYS_CLK/PWM_FREQ)/PWM_PSC - 1
	
	- PWM_CMR formula : PWM_DUTY x (PWM_CNR + 1)/1000	(with duty from 0.1% to 100%)

- reverse duty increase or decrease when reach 0% or 100%

- Toggle P1.2 (MS51FB EVM LED) to monitor as channel 2 in below scope waveform

![image](https://github.com/released/MS51_PWM_RESOLUTION_1K/blob/master/FREQ_375Hz_RESOLUTION_1K.jpg)

