#ifndef DEFINE_H
#define DEFINE_H

#define _FREQ_PWM   (10000.0f)
#define _TS         (1.0f/_FREQ_PWM)
#define _REG2VOLT   (3.25f/4096.0f)
#define _R_SHUNT    (0.33f)
#define _AMP_GAIN   ( 680.0f*3.3f*2.0f/(2200.0f+680.0f) )
#define _IX2VOLT    (-_R_SHUNT*_AMP_GAIN)
#define _VOLT2IX    (1.0f/_IX2VOLT)

#define _VOLT2VBUS  (192.0f/12.0f)

#define _MOTOR_POLE               (14)
#define _MOTOR_RA                 (0.453f)
#define _MOTOR_LD                 (0.0009477f)
#define _MOTOR_LQ                 (0.0009477f)
#define _MOTOR_PSIA               (0.006198f)
#define _MOTOR_JM                 (0.00001612f)
#define _ENC_PULSE_MECHA_CYCLE    (1200)

#define _LIMIT_HZ                 (300.0f)
#define _LIMIT_VOLT               (15.0f)
#define _LIMIT_AMP                (1.5f)
#define _OC_AMP                   (2.0f)

#define _ASR_W                    (100.0f)
#define _ASR_XI                   (0.5f)

#define _ACR_W                    (2000.0f)
#define _ACR_XI                   (0.5f)

#define _APR_W                    (500.0f)
#define _APR_XI                   (0.5f)

#endif