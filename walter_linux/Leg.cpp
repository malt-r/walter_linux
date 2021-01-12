#include "Leg.h"
#include "LegConstants.h"

Leg::Leg()
{
    m_pHipJoint = nullptr;
    m_pUpperLegJoint = nullptr;
    m_pLowerLegJoint = nullptr;
    
    m_theta0 = 0;
    
    m_femurLength = legConst_femurLength;
    m_thighLength = legConst_thighLength;
    m_footLength = legConst_footLength;
    
    m_radius = legConst_radius;
}

Leg::Leg(Servo* hipJoint, Servo* upperLegJoint, Servo* lowerLegJoint, unsigned int legIndex)
{
    m_pHipJoint = hipJoint;
    m_pUpperLegJoint = upperLegJoint;
    m_pLowerLegJoint = lowerLegJoint;

    const unsigned int numOfLegs = 6;
    if (legIndex < numOfLegs)
    {
        m_theta0 = legConst_afTheta0s[legIndex];
    }
    else
    {
        m_theta0 = 0;
    }

    m_femurLength = legConst_femurLength;
    m_thighLength = legConst_thighLength;
    m_footLength = legConst_footLength;

    m_radius = legConst_radius;
}

Leg::~Leg()
{
}

Servo* Leg::GetHipJointServo()
{
    return m_pHipJoint;
}

Servo* Leg::GetUpperLegJointServo()
{
    return m_pUpperLegJoint;
}

Servo* Leg::GetLowerLegJointServo()
{
    return m_pLowerLegJoint;
}

double Leg::GetRadius()
{
    return m_radius;
}

double Leg::GetTheta0inDegree()
{
    return m_theta0;
}

double Leg::GetFemurLength()
{
    return m_femurLength;
}

double Leg::GetThighLength()
{
    return m_thighLength;
}

double Leg::GetFootLength()
{
    return m_footLength;
}

void Leg::SetLastCommand(std::string cmd)
{
    m_lastCommand = cmd;
}

std::string Leg::GetLastCommand()
{
    return m_lastCommand;
}
