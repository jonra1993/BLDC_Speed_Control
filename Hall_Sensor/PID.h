/*************************************************************
 *
 * $Workfile: PID.h $
 *
 * $Creator: Jonathan Vargas $
 *
 * $Description: Discrete motor control PID algorithm $
 *************************************************************/

#include <Arduino.h>

#ifndef PID_H_
#define PID_H_

class PID {
protected:

private:
	uint16_t sampleTime;                         // Sampling Time in ms

public:
	
	float m_currentLevel;               // Current measured
	float m_desiredLevel;               // Setpoint
	float m_errorLevel;                 // Error
	float m_previousErrorLevel;         // Previous Error		
	float m_proportional;               // Proportional Term
	float m_integral;                   // Integral Term
	float m_derivative;                 // Derivative Term
	float m_pwmOutput;                  // Signal Output
	uint16_t m_sampleTime;            	// Sample timeted
	int m_Outputmin; 
	int m_Outputmax;
	unsigned long lastTime;
	float Kp; float Ki; float Kd;     	//PID constants

	
	PID( float Kp, float Ki, float Kd, int sampleT=100, int outMin=0, int outMax=100); 
	float PID_ProcessIteration(float Input, float Setpoint);       
	void Set_Output_Limits(int outMin, int outMax); // * clamps the output to a specific range.
	uint16_t Get_Sample_Time();
	void Set_Sample_Time(uint16_t sampleT);
	void Set_PID_Constants( float Kp, float Ki, float Kd);
	void PID_Clear();
};
			 
#endif /* PID_H_*/


/*****************************************************************************
 *
 * End of File: PID.h
 *
 ****************************************************************************/
