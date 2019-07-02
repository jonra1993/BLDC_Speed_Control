/*************************************************************
 *
 * $Workfile: PID.cpp $
 *
 * $Creator: Jonathan Vargas $
 *
 * $Description: Discrete motor control PID algorithm $
 *************************************************************/

#include "PID.h"

/***************************************************************************
 *
 * PID Constructor
 *
 ***************************************************************************/

//! PID initial variables, sampleT is in ms
PID::PID( float Kp, float Ki, float Kd, int sampleT=100, byte outMin=0, byte outMax=100){
	PID::Kp= Kp;
 	PID::Ki = Ki;
	PID::Kd = Kd;
	PID::Set_Sample_Time(sampleT);
	Set_Output_Limits(outMin, outMax);         
}

/*****************************************************************************
 *
 * functions
 *
 ****************************************************************************/

//! Calculates PID control law according saturation levels
float PID::PID_ProcessIteration (float Input, float Setpoint)
{
	unsigned long currentMillis = millis();
	unsigned long timeChange = (currentMillis - PID::lastTime);

	PID::m_desiredLevel = Setpoint;
	PID::m_currentLevel = Input;
	PID::m_sampleTime = PID::Get_Sample_Time();

	if (timeChange >= PID::m_sampleTime){
		// Calculate actual error
		PID::m_errorLevel = PID::m_desiredLevel - PID::m_currentLevel;
		// Calculate P component P[n] = kp*e[n]
		PID::m_proportional = PID::m_errorLevel * PID::Kp;
		// Calculate I component I[n] = I[n-1]+ki*e[n]*ts
		PID::m_integral = PID::m_integral + PID::m_errorLevel * PID::Ki * (float)PID::m_sampleTime;
		// Calculate D component D[n] = ((e[n]*e[n-1])*kd)/ts
		PID::m_derivative = ((PID::m_errorLevel - PID::m_previousErrorLevel) * PID::Kd) / (float)PID::m_sampleTime;
		// Calculate PID control law k[n] = P[n] + I[n] +D[n]
		PID::m_pwmOutput = PID::m_proportional + PID::m_integral + PID::m_derivative;
		// Saturate upper limit of output
		if(PID::m_pwmOutput > (float)PID::m_Outputmax){
				PID::m_pwmOutput = PID::m_Outputmax;
		}
		// Saturate lower limit of output
		if(PID::m_pwmOutput < (float)PID::m_Outputmin){
				PID::m_pwmOutput = PID::m_Outputmin;
		}
		// Saves actual error as previous error		
		PID::m_previousErrorLevel = PID::m_errorLevel;
		PID::lastTime = currentMillis;
		
		return PID::m_pwmOutput;
		}
}

//! Sets maximum allowable PWM duty cycle of PID.
void PID::Set_Output_Limits(byte outMin, byte outMax) // * clamps the output to a specific range.
{
	PID::m_Outputmin = outMin;
	PID::m_Outputmax = outMax;

}

//! Gets sample time
uint16_t PID::Get_Sample_Time(){
	return PID::sampleTime;
}

//! Sets sample time
void PID::Set_Sample_Time(uint16_t sampleT){
	PID::sampleTime = sampleT;
}

//! Sets PID constants
void PID::Set_PID_Constants( float Kp, float Ki, float Kd){
	PID::Kp= Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;
}

//! Clears PID variables 
void PID::PID_Clear(){
	PID::m_proportional = 0.0f;
	PID::m_integral = 0.0f;
	PID::m_derivative = 0.0f;
}