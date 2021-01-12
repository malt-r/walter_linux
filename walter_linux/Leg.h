#pragma once
#include "Servo.h"
#include "MathEngine.h"
#include <string>

class Leg
{
private:
    Servo* m_pHipJoint;
    Servo* m_pUpperLegJoint;
    Servo* m_pLowerLegJoint;

    //MathEngine::CartesianVector m_currentVector;
    
    double m_radius;
    double m_theta0;

    double m_femurLength;	// "Oberschenkel"
    double m_thighLength;	// "Unterschenkel"
    double m_footLength;	// "Fuﬂ"

    std::string m_lastCommand;
public: 
    Leg();
    Leg
    (
        Servo* hipJoint,
        Servo* upperLegJoint,
        Servo* lowerLegJoint,
        unsigned int legIndex
    );
    ~Leg();

    Servo* GetHipJointServo();
    Servo* GetUpperLegJointServo();
    Servo* GetLowerLegJointServo();
    double GetRadius();
    double GetTheta0inDegree();
    double GetFemurLength();
    double GetThighLength();
    double GetFootLength();
    void SetLastCommand(std::string cmd);
    std::string GetLastCommand();
   // MathEngine::CartesianVector GetCurrentCartesianVectorOfFoot();
};

