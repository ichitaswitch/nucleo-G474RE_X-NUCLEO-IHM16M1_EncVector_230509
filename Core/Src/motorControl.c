#include "stm32g4xx_hal.h"
#include "tim.h"
#include "dac.h"
#include "comp.h"
#include "define.h"
#include <math.h>

typedef struct
{
	float sum;
	float out;
	float max;
	float min;
	float kp;
	float ki;
} pi_t;

static pi_t asr =
{
  .max = _LIMIT_AMP,
  .min = -_LIMIT_AMP,
  .kp = 2.0f * _ASR_XI * _ASR_W * _MOTOR_JM / (_MOTOR_POLE>>1) / (_MOTOR_POLE>>1) / _MOTOR_PSIA,
  .ki = _ASR_W * _ASR_W * _MOTOR_JM / (_MOTOR_POLE>>1) / (_MOTOR_POLE>>1) / _MOTOR_PSIA * _TS
};

static pi_t adr =
{
  .max = _LIMIT_VOLT,
  .min = -_LIMIT_VOLT,
  .kp = 2.0f * _ACR_XI * _ACR_W * _MOTOR_LD - _MOTOR_RA,
  .ki = _ACR_W * _ACR_W * _MOTOR_LD * _TS
};

static pi_t aqr =
{
  .max = _LIMIT_VOLT,
  .min = -_LIMIT_VOLT,
  .kp = 2.0f * _ACR_XI * _ACR_W * _MOTOR_LQ - _MOTOR_RA,
  .ki = _ACR_W * _ACR_W * _MOTOR_LQ * _TS
};

static pi_t apr =
{
  .max = _LIMIT_HZ * _RAD(360.0f),
  .min = -_LIMIT_HZ * _RAD(360.0f),
  .kp = 2.0f * _APR_XI * _APR_W,
  .ki = _APR_W * _APR_W * _TS
};

void pwmEnable(void);
void pwmDisable(void);
static void _TIM_CCxChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelState);
static void _TIM_CCxNChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelNState);
float encpls2Phase( uint16_t encPls );
void uvw2ab( float u, float v, float w, float *a, float *b );
void ab2dq( float a, float b, float *d, float *q, float phs );
void dq2ab( float d, float q, float *a, float *b, float phs );
void ab2uvw( float a, float b, float *u, float *v, float *w );
void svm( float *u, float *v, float *w);
float phaseFix(float phs);
uint8_t pi( pi_t *hdl, float err );

float phase, phaseErr, phaseEnc;
float iU,iV,iW,vBus;
float iA,iB,iD,iQ;
float iDRef, iQRef;
float vDRef, vQRef, vARef, vBRef, vURef, vVRef, vWRef;
float omgDst, omgRef, omgFlt, omg;

uint8_t posiMatch;

extern uint16_t adc1Buf;
extern uint16_t adc2Buf[2];
extern uint8_t encEdgeDetect;
extern uint8_t onOff;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  static int16_t u,v,w;
  static int16_t oU,oV,oW;
  static uint8_t adNum;
  static uint32_t step;
  static uint16_t encCnt;

  if( hadc->Instance == ADC1 )
  {
    u = HAL_ADCEx_InjectedGetValue(hadc, 1);
    adNum++;
  }
  else if( hadc->Instance == ADC2 )
  {
    v = HAL_ADCEx_InjectedGetValue(hadc, 1);
    adNum++;
  }
  else if( hadc->Instance == ADC3 )
  {
    w = HAL_ADCEx_InjectedGetValue(hadc, 1);
    adNum++;
  }

  if( adNum < 3 ) return;
	adNum = 0;

  if( !onOff )
  {
    oU += (u-oU)>>1;
    oV += (v-oV)>>1;
    oW += (w-oW)>>1;
    HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, (oU*_REG2VOLT-_IX2VOLT*_OC_AMP)* (1.0f/3.25f*0xFFF) );
    HAL_DAC_SetValue(&hdac2, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, (oV*_REG2VOLT-_IX2VOLT*_OC_AMP)* (1.0f/3.25f*0xFFF) );
    HAL_DAC_SetValue(&hdac3, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, (oW*_REG2VOLT-_IX2VOLT*_OC_AMP)* (1.0f/3.25f*0xFFF) );
    pwmDisable();

    step = 0;
    adr.sum = aqr.sum = asr.sum = 0.0f;
    iU = iV = iW = iA = iB = iD = iQ = 0.0f;
    iDRef = iQRef = 0.0f;
    vDRef = vQRef = vARef = vBRef = vURef = vVRef = vWRef = 0.0f;
    encCnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    if( posiMatch != 0xFF )
    {
      phase = 0.0f;
      phaseErr = 0.0f;
      omg = omgFlt = omgRef = 0.0f;
      posiMatch = 0;
    }
    else
    {
      phase = phaseFix(phase+omg*_TS);
      if(encEdgeDetect)
      {
        encEdgeDetect = 0;
        phaseEnc = phaseFix( encpls2Phase((uint16_t)__HAL_TIM_GET_COUNTER(&htim3)) );
      }
      phaseErr = phaseFix( phaseEnc - phase );
      pi( &apr, phaseErr );
      omg = apr.out;
      omgFlt += (omg - omgFlt) * (100.0f) * _TS;
      omgRef = omgFlt;
    }
    return;
  }

  pwmEnable();
  float iu = (u-oU)*_REG2VOLT*_VOLT2IX;
  float iv = (v-oV)*_REG2VOLT*_VOLT2IX;
  float iw = (w-oW)*_REG2VOLT*_VOLT2IX;
  float *min = &vWRef;
  if( *min > vURef ) min = &vURef;
  if( *min > vVRef ) min = &vVRef;
  if( min == &vURef ) iu = -(iv+iw);
  else if( min == &vVRef ) iv = -(iw+iu);
  else if( min == &vWRef ) iw = -(iu+iv);
  iU += ( iu - iU ) * (3000.0f) * _TS;
  iV += ( iv - iV ) * (3000.0f) * _TS;
  iW += ( iw - iW ) * (3000.0f) * _TS;
  vBus += ( adc1Buf*_REG2VOLT*_VOLT2VBUS - vBus ) * (3000.0f) * _TS;

  // UVW->dq
  uvw2ab(iU,iV,iW,&iA,&iB);
  ab2dq(iA,iB,&iD,&iQ,phase);

  if( !posiMatch )
  {
    encEdgeDetect = 0;
    float fBuf = omg;
    fBuf += _RAD(360.0f)/0.001f*_TS;
    if( fBuf > _RAD(360.0f) )
    {
      fBuf = _RAD(360.0f);
    }
    omg = fBuf;
    phase = phaseFix(phase+omg*_TS);

    fBuf = iDRef;
    fBuf += 0.5f/0.0001f*_TS;
    if( fBuf > 0.5f )
    {
      fBuf = 0.5f;
    }
    iDRef = fBuf;
    iQRef = 0.0f;

    uint16_t deltaBuf = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3) - encCnt;
    if( *(int16_t *)&deltaBuf > 30 || *(int16_t *)&deltaBuf < -30 )
    {
      if( phase < _RAD(5.0f) && phase > _RAD(-5.0f) )
      {
        phase = omg = 0.0f;
        posiMatch = 1;
      }
    }
  }
  else if( posiMatch == 1 )
  {
    encEdgeDetect = 0;
    if( ++step > _FREQ_PWM/10.0f )
    {
      __HAL_TIM_SET_COUNTER(&htim3, 0);
      posiMatch = 0xFF;
    }
  }
  else
  {
    phase = phaseFix(phase+omg*_TS);
    if(encEdgeDetect)
    {
      encEdgeDetect = 0;
      phaseEnc = phaseFix( encpls2Phase((uint16_t)__HAL_TIM_GET_COUNTER(&htim3)) );
    }
    phaseErr = phaseFix( phaseEnc - phase );
    pi( &apr, phaseErr );
    omg = apr.out;
    omgFlt += (omg - omgFlt) * (100.0f) * _TS;
    
    if( omgDst - omgRef > 100.0f*_RAD(360.0f)/0.1f*_TS ) omgRef += 100.0f*_RAD(360.0f)/0.1f*_TS;
    else if( omgRef - omgDst > 100.0f*_RAD(360.0f)/0.1f*_TS ) omgRef -= 100.0f*_RAD(360.0f)/0.1f*_TS;
    else omgRef = omgDst;

    pi( &asr, omgRef-omg );
    iQRef = asr.out;

    if( iDRef > 0.0f )
    {
      float fBuf = iDRef;
      fBuf -= 0.5f/0.0001f*_TS;
      if( fBuf < 0.0f )
      {
        fBuf = 0.0f;
      }
      iDRef = fBuf;
    }
    else
    {
      iDRef = 0.0f;
    }
  }

  // ACR->VdRef,VqRef更新
  pi(&adr, iDRef-iD);
  vDRef = adr.out;
  pi(&aqr, iQRef-iQ);
  vQRef = aqr.out;

  // dq->UVW
  dq2ab(vDRef,vQRef,&vARef,&vBRef,phase);
  ab2uvw(vARef,vBRef,&vURef,&vVRef,&vWRef);

  // PWM Duty更新
  svm(&vURef,&vVRef,&vWRef);

  float rVBus = 1.41421356f/vBus;
  uint16_t pwmRef = htim1.Init.Period>>1;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwmRef - pwmRef*vURef*rVBus );
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwmRef - pwmRef*vVRef*rVBus );
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwmRef - pwmRef*vWRef*rVBus );
}

float encpls2Phase( uint16_t encPls )
{
  return phaseFix( (float)encPls/(float)_ENC_PULSE_MECHA_CYCLE * (float)(_MOTOR_POLE>>1) * _RAD(360.0f) );
}

void pwmEnable(void)
{
  HAL_COMP_Start(&hcomp1);
  HAL_COMP_Start(&hcomp4);
  HAL_COMP_Start(&hcomp6);
  _TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
  _TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
  _TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);
  _TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCxN_ENABLE);
  _TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCxN_ENABLE);
  _TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_ENABLE);
}

void pwmDisable(void)
{
  _TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_DISABLE);
  _TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCx_DISABLE);
  _TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCx_DISABLE);
  _TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCxN_DISABLE);
  _TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCxN_DISABLE);
  _TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_DISABLE);
  HAL_COMP_Stop(&hcomp1);
  HAL_COMP_Stop(&hcomp4);
  HAL_COMP_Stop(&hcomp6);
}

static void _TIM_CCxChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelState)
{
  uint32_t tmp;

  /* Check the parameters */
  assert_param(IS_TIM_CC1_INSTANCE(TIMx));
  assert_param(IS_TIM_CHANNELS(Channel));

  tmp = TIM_CCER_CC1E << (Channel & 0x1FU); /* 0x1FU = 31 bits max shift */

  /* Reset the CCxE Bit */
  TIMx->CCER &= ~tmp;

  /* Set or reset the CCxE Bit */
  TIMx->CCER |= (uint32_t)(ChannelState << (Channel & 0x1FU)); /* 0x1FU = 31 bits max shift */
}

static void _TIM_CCxNChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelNState)
{
  uint32_t tmp;

  tmp = TIM_CCER_CC1NE << (Channel & 0x1FU); /* 0x1FU = 31 bits max shift */

  /* Reset the CCxNE Bit */
  TIMx->CCER &=  ~tmp;

  /* Set or reset the CCxNE Bit */
  TIMx->CCER |= (uint32_t)(ChannelNState << (Channel & 0x1FU)); /* 0x1FU = 31 bits max shift */
}

void uvw2ab( float u, float v, float w, float *a, float *b )
{
	*a = ( 0.816496580928f ) * ( u - ( ( v + w ) * (0.5f) ) );
  *b = ( 0.7071067811866f ) * ( v - w );
}

void ab2dq( float a, float b, float *d, float *q, float phs )
{
	float sin, cos;

	sin = sinf(phs);
	cos = cosf(phs);

	*d = cos*a + sin*b;
	*q = cos*b - sin*a;
}

void dq2ab( float d, float q, float *a, float *b, float phs )
{
	float sin, cos;

	sin = sinf(phs);
	cos = cosf(phs);

	*a = cos*d - sin*q;
	*b = cos*q + sin*d;
}

void ab2uvw( float a, float b, float *u, float *v, float *w )
{
	a *= (0.40824892046f);
  b *= (0.70710678118f);
  *u = 2.0f * a;
  *v = -a + b;
  *w = -a - b;
}

void svm( float *u, float *v, float *w)
{
	float max, min, typ;

	max = min = *u;
	if(*v > max) max = *v;
	if(*w > max) max = *w;
	if(*v < min) min = *v;
	if(*w < min) min = *w;
	typ = (max + min) * 0.5;

	*u -= typ;
	*v -= typ;
	*w -= typ;
}

float phaseFix(float phs)
{
	while(phs > _RAD(180.0f) )
		phs -= _RAD(360.0f);
	while(phs < _RAD(-180.0f)  )
		phs += _RAD(360.0f);
	return phs;
}

uint8_t pi( pi_t *hdl, float err )
{
	uint8_t ret = 0;
	float out;

	hdl->sum = hdl->sum + err * hdl->ki;

	if(hdl->sum > hdl->max)
	{
		hdl->sum = hdl->max;
		ret = 1;
	}
	else if(hdl->sum < hdl->min)
	{
		hdl->sum = hdl->min;
		ret = 1;
	}

	out = hdl->sum + err * hdl->kp;

	if(out > hdl->max)
	{
		out = hdl->max;
		ret = 1;
	}
	else if(out < hdl->min)
	{
		out = hdl->min;
		ret = 1;
	}

	hdl->out = out;

	return ret;
}