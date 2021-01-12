#include "MotionEngine.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
//#include <Windows.h>
#include <cstring>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <string>
#include "PerformanceTimer.h"
#include "LegConstants.h"
#include "defines.h"

#define _USE_MATH_DEFINES // for C++
#include <math.h>

//#ifndef _ARM_
//#include <Windows.h>
//#endif

MotionEngine::MotionEngine
(
    std::vector<Leg*> legPtrs,
    std::vector<int>* buttons,
    std::vector<std::vector<int>>* legAnglesRaw,
    std::vector<std::string>* notifys,
    std::vector<std::string>* errors
)
{
    //copy the vector
    for (int i = 0; i < 6; i++)
    {
        m_pLegPtrs[i] = legPtrs[i];
    }
    m_bShiftGroupIsOdds = false;
    m_bShiftGroupsSwitched = false;
    
    memset(m_afDeltaX, 0, sizeof(*m_afDeltaX) * 6);
    memset(m_afDeltaY, 0, sizeof(*m_afDeltaY) * 6);
    memset(m_afDeltaZ, 0, sizeof(*m_afDeltaZ) * 6);
    
    memset(m_afStartDeltaX, 0, sizeof(*m_afStartDeltaX) * 6);
    memset(m_afStartDeltaY, 0, sizeof(*m_afStartDeltaY) * 6);
    memset(m_afStartDeltaZ, 0, sizeof(*m_afStartDeltaZ) * 6);
    memset(m_afMiddleZ, 0, sizeof(*m_afMiddleZ) * 6);    

    m_nLastDirectionX = 0;
    m_nLastDirectionY = 0;
    m_bIsWalking = false;
    m_bIsFinishedWalking = true;
    /*m_fCurrentDirectionAngle = 0.0;
    m_fBlockedDirectionAngle = 0.0;
    m_bDirectionBlocked = false;
    */
    m_currentState = MotionEngine::RobotState::init;
    memset(m_currentTraversalTargetVector, 0, sizeof(*m_currentTraversalTargetVector) * 6);
    m_fCurrentTimeForTraversalms = 0;
    m_traversalStartPoint = std::chrono::high_resolution_clock::now();
    m_bIsFinishedTraversing = true;
    m_homingState = MotionEngine::HomingState::isAtHome;

    m_buttons = buttons;
    m_legAnglesRawAnalogue = legAnglesRaw;

    m_notifys = notifys;
    m_errors = errors;
}

MotionEngine::~MotionEngine()
{
}

bool MotionEngine::MoveServoToPulseWidth
(
	int hComm, 
	Servo* pServo, 
	int nPulse, 
	unsigned int nTime,
	bool bOutputMessageToConsole,
	bool bOverrideBoundaries //overrides the min / max position
)
{
	bool bRet = true;
	
	if (nPulse < 10 || nPulse > 3000)
	{
		/*if (bOutputMessageToConsole)
		{
			std::cout << "nPulse was out of range: " << nPulse << std::endl;
		}*/
		bRet = false;
	}
	else
	{
		unsigned int nSetPulseWidth;
		std::string sCommand 
			= pServo->GetServoPulseString
			(
				nPulse, 
				&nSetPulseWidth, 
				bOverrideBoundaries);

		if (nTime != 0)
		{
			//append time
			sCommand.append("T");
			sCommand.append(std::to_string(nTime));
		}

		//add carriage-return (ASCII 13)
		sCommand += char(13);

		//info
		if (bOutputMessageToConsole)
		{
			//std::cout << "Sending command: " << sCommand.c_str() << std::endl;
			//std::cout << "Moving servo to position " << nPulse;
			/*if (nTime != 0)
			{
				std::cout << " with time " << nTime;
			}
			std::cout << std::endl;*/
		}
		
		//DWORD nBytesWritten = 0;
		//send command
		if (hComm != -1)
		{
			/*bool bSuccess
				= WriteFile
				(
					hComm,
					sCommand.c_str(),
					sCommand.length(),
					&nBytesWritten,
					NULL
				);*/
            if (UNLOCK_MOVEMENT)
            {
                serialPuts(hComm, sCommand.c_str());
            }
            pServo->SetPulse(nSetPulseWidth, bOverrideBoundaries);
			/*if (!bSuccess)
			{
				if (bOutputMessageToConsole)
				{
					std::cout << "An error occured while sending the command! Errorcode: "
						<< GetLastError() << std::endl;
				}
				bRet = false;
			}
			else
			{
				if (bOutputMessageToConsole)
				{
					std::cout << "Sent " << nBytesWritten << " Bytes" << std::endl;
				}
				
			}*/
		}
		else
		{
			pServo->SetPulse(nSetPulseWidth, bOverrideBoundaries);
		}
	}
	return bRet;
}

void MotionEngine::MoveGroup
(
    int hComm, 
    std::vector<Servo*> servos, 
    std::vector<unsigned int> positions, 
    unsigned int nTime,
    bool bPrintToConsole
)
{
	//# <ch> P <pw> S ?<spd>? ... # <ch> P <pw> S ?<spd>? T <time> <cr>
	std::string sCommand;
	unsigned int nCurrentServoIndex = 0;
	//concatenate all servo position commands
	for (Servo* servo : servos)
	{
		unsigned int nSetPosition;
		sCommand.append
		(
			servo->GetServoPulseString
			(
				positions.at
				(
					nCurrentServoIndex
				),
				&nSetPosition
			)
		);
		servo->SetPulse(nSetPosition);
		nCurrentServoIndex++;
	}

	// append time and carriage return
	if (nTime != 0)
	{
		//append time
		sCommand.append("T");
		sCommand.append(std::to_string(nTime));
	}

	//add carriage-return (ASCII 13)
	sCommand += char(13);
    //if (bPrintToConsole)
    //{
    //    std::cout << "Sending command: " << sCommand.c_str() << std::endl;
    //}

	//add sending
	//DWORD nBytesWritten = 0;
	if (hComm != -1)
	{
		//WriteFile(hComm, sCommand.c_str(), sCommand.length(), &nBytesWritten, NULL);
        if (UNLOCK_MOVEMENT)
        {
            serialPuts(hComm, sCommand.c_str());
        }
        //if (bPrintToConsole)
        //{
        //    std::cout << "Sent " << nBytesWritten << " bytes" << std::endl;
        //}
	}
	//else
	//{
 //       //if (bPrintToConsole)
 //       //{
 //       //    std::cout << "hComm is invalid, no bytes sent..." << std::endl;
 //       //}
	//}
}

//TODO: modify, so that the command 
void MotionEngine::MoveGroupWithAngles
(
    int hComm,
    std::vector<Servo*> servos,
    std::vector<double> positionsAsDegree,
    unsigned int nTime,
    bool bPrintToConsole,
    std::string* pCmd
)
{
    bool bDebug = false;
    do
    {
        //# <ch> P <pw> S ?<spd>? ... # <ch> P <pw> S ?<spd>? T <time> <cr>
        std::string sCommand;
        unsigned int nCurrentServoIndex = 0;
        //concatenate all servo position commands
        for (Servo* servo : servos)
        {
            double fSetAngle;
            sCommand.append
            (
                servo->GetServoPulseStringWithAngle
                (
                    positionsAsDegree.at
                    (
                        nCurrentServoIndex
                    ),
                    &fSetAngle
                )
            );
            servo->SetAngle(fSetAngle);
            nCurrentServoIndex++;
        }

        // append time and carriage return
        if (nTime != 0)
        {
            //append time
            sCommand.append("T");
            sCommand.append(std::to_string(nTime));
        }
        sCommand.append(";");
        //add carriage-return (ASCII 13)
        sCommand += char(13);

//#if DEBUG_OUTPUT
//        if (true)
//        {
//            std::cout << "Sending command: " << sCommand.c_str() << std::endl;
//        }
//#endif

        //add sending
        long nBytesWritten = 0;
        if (hComm != -1)
        {
            //WriteFile(hComm, sCommand.c_str(), sCommand.length(), &nBytesWritten, NULL);
            if (UNLOCK_MOVEMENT)
            {
                serialPuts(hComm, sCommand.c_str());
            }
            /*if (bPrintToConsole)
            {
                std::cout << "Sent " << nBytesWritten << " bytes" << std::endl;
            }*/
        }
        if (nullptr != pCmd)
        {
            *pCmd = sCommand;
        }
        //else
        //{
        //    /*if (bPrintToConsole)
        //    {
        //        std::cout << "hComm is invalid, no bytes sent..." << std::endl;
        //    }*/
        //}

        if (isnan(servos[0]->GetCurrentAngle()) ||
            isnan(servos[1]->GetCurrentAngle()) ||
            isnan(servos[2]->GetCurrentAngle()))
        {
            //bool bBreak = true;
        }
    } while (bDebug);
}

MathEngine::CartesianVector MotionEngine::ConvertFromGlobalToLocalVector
(
    Leg leg,
    MathEngine::CartesianVector globalVector
)
{
    MathEngine::CartesianVector returnVector = MathEngine::CartesianVector();
    double radius = leg.GetRadius();
    double theta0 = MathEngine::FromDegToRad(leg.GetTheta0inDegree());

    double fRotationMatrixComToS[3][3]
    {
        {cos(-theta0), -sin(-theta0), 0}, //row 0
        {sin(-theta0), cos(-theta0), 0 }, //row 1
        {0, 0, 1}                         //row 2
    };
    double fTranslationVector[3]
    {
        -radius * cos(theta0),
        -radius * sin(theta0),
        0
    };
    double fTargetVectorLocalCopy[3]
    {
        globalVector.x,
        globalVector.y,
        globalVector.z
    };

    double targetVectorInHipJointCoord[3];

    //translate the endpoint of the targetvector from the 
    //COM-Coordinatesystem to the Coordinatesystem of the hipjoint
    //does not work correctly
    for (int i = 0; i < 3; i++)
    {
        targetVectorInHipJointCoord[i] = //order of operations?
            (fTargetVectorLocalCopy[0] + fTranslationVector[0]) * fRotationMatrixComToS[i][0] +
            (fTargetVectorLocalCopy[1] + fTranslationVector[1]) * fRotationMatrixComToS[i][1] +
            (fTargetVectorLocalCopy[2] + fTranslationVector[2]) * fRotationMatrixComToS[i][2];
    }
    returnVector =
        MathEngine::CartesianVector
        (
            targetVectorInHipJointCoord[0],
            targetVectorInHipJointCoord[1],
            targetVectorInHipJointCoord[2]
        );
    return returnVector;
}

//faulty
MathEngine::CartesianVector MotionEngine::ConvertFromLocalToGlobalVector
(
    Leg leg,
    MathEngine::CartesianVector localVector
)
{
    MathEngine::CartesianVector returnVector = MathEngine::CartesianVector();

    double theta0inDegree = leg.GetTheta0inDegree();
    double theta0 = MathEngine::FromDegToRad(theta0inDegree);
    double radius = leg.GetRadius();
    double fRotationMatrixSToCoM[3][3]
    {
        {cos(theta0), -sin(theta0), 0}, //row 0
        {sin(theta0), cos(theta0), 0 }, //row 1
        {0, 0, 1}                         //row 2
    };
    double fTranslationVector[3]
    {
        radius * cos(theta0),
        radius * sin(theta0),
        0
    };
    double fTargetVectorLocalCopy[3]
    {
        localVector.x,
        localVector.y,
        localVector.z
    };

    double targetVectorInCoMCoord[3];

    //translate the endpoint of the targetvector from the 
    //hipjoint-Coordinatesystem to the CoM-System
    /*for (int i = 0; i < 3; i++)
    {
        targetVectorInCoMCoord[i] =
            (fTargetVectorLocalCopy[0] + fTranslationVector[0]) * fRotationMatrixSToCoM[i][0] +
            (fTargetVectorLocalCopy[1] + fTranslationVector[1]) * fRotationMatrixSToCoM[i][1] +
            (fTargetVectorLocalCopy[2] + fTranslationVector[2]) * fRotationMatrixSToCoM[i][2];
    }*/
    for (int i = 0; i < 3; i++)
    {
        targetVectorInCoMCoord[i] =
            (fTargetVectorLocalCopy[0] * fRotationMatrixSToCoM[i][0]) +
            (fTargetVectorLocalCopy[1] * fRotationMatrixSToCoM[i][1]) +
            (fTargetVectorLocalCopy[2] * fRotationMatrixSToCoM[i][2]);
    }
    returnVector =
        MathEngine::CartesianVector
        (
            targetVectorInCoMCoord[0] + fTranslationVector[0],
            targetVectorInCoMCoord[1] + fTranslationVector[1],
            targetVectorInCoMCoord[2] + fTranslationVector[2]
        );

    bool breakFlag;
    if (isnan(returnVector.x) ||
        isnan(returnVector.y) ||
        isnan(returnVector.z))
    {
        //breakFlag = true;
    }
    return returnVector;
}

MathEngine::CartesianVector MotionEngine::GetCurrentCartesianVectorOfFootGlobal
(
    Leg leg,
    MathEngine::CartesianVector* hipJointRelativeVector,
    MathEngine::CartesianVector* pLocalFootVector
)
{
    MathEngine::CartesianVector returnVector = MathEngine::CartesianVector();
    bool bDebug = false;
    do
    {
        double fFootVectorLengthxy = 0;
        MathEngine::CartesianVector localFootVector = 
            MotionEngine::GetCurrentCartesianVectorOfFootLocal(leg, &fFootVectorLengthxy);
        if (pLocalFootVector != nullptr)
        {
            pLocalFootVector->x = localFootVector.x;
            pLocalFootVector->y = localFootVector.y;
            pLocalFootVector->z = localFootVector.z;
        }
        if (hipJointRelativeVector != nullptr)
        {
            hipJointRelativeVector->x = fFootVectorLengthxy;
            hipJointRelativeVector->z = localFootVector.z;//relative z needs improvement --> perhaps the foot length and food angle are a little bit off
        }

        //TODO: add call to ConversionMethod here
        returnVector = ConvertFromLocalToGlobalVector(leg, localFootVector);

        bool breakFlag;
        if (isnan(returnVector.x) ||
            isnan(returnVector.y) ||
            isnan(returnVector.z))
        {
            //breakFlag = true;
        }
    } while (bDebug);
    return returnVector;
}

MathEngine::CartesianVector MotionEngine::GetCurrentCartesianVectorOfFootLocal
(
    Leg leg,
    double* lengthOfFootVectorxy
)
{
    MathEngine::CartesianVector returnVector = MathEngine::CartesianVector();
    bool bDebug = false;
    //theta 1
    double fHipJointAngle = leg.GetHipJointServo()->GetCurrentAngle(false);

    //theta 2
    double fUpperLegJointAngle = leg.GetUpperLegJointServo()->GetCurrentAngle(false);

    //theta 3
    double fLowerLegJointAngle = leg.GetLowerLegJointServo()->GetCurrentAngle(false);

    double fFemurLength = leg.GetFemurLength();
    double fThighLength = leg.GetThighLength();
    double fFootLength = leg.GetFootLength();
    double legRadius = leg.GetRadius();

    double L2x = cos(fUpperLegJointAngle) * fThighLength;
    double theta32 = M_PI_2 - fUpperLegJointAngle;
    double theta33 = fLowerLegJointAngle - theta32;
    double L3x = sin(theta33) * fFootLength;

    double fFootVectorLengthxy = L2x + L3x + fFemurLength; // this is is the length of the footVector in x/y-plane
    double fz = cos(theta33) * fFootLength - sin(fUpperLegJointAngle) * fThighLength;
    double fx = cos(fHipJointAngle) * fFootVectorLengthxy;
    double fy = sin(fHipJointAngle) * fFootVectorLengthxy;
    if (lengthOfFootVectorxy != nullptr)
    {
        *lengthOfFootVectorxy = fFootVectorLengthxy;
    }
    returnVector.x = fx;
    returnVector.y = fy;
    returnVector.z = fz;
    
    return returnVector;
}

MotionEngine::LegAngles MotionEngine::CalculateLegAnglesWithLocalVector
(
    Leg leg, 
    MathEngine::CartesianVector targetVectorLocal,
    int* errorRet
)
{
    LegAngles returnAngles = LegAngles();
    {
        //PerformanceTimer timer = PerformanceTimer("local kinematics");
        bool bDebug = false;
        do
        {
            double footLength = legConst_footLength;//TODO: einfach die konstanten nehmen
            double femurLength = legConst_femurLength;
            double ThighLength = legConst_thighLength;


            double theta1 = atan(targetVectorLocal.y / targetVectorLocal.x);
            double L1 = sqrt
            (
                targetVectorLocal.x * targetVectorLocal.x +
                targetVectorLocal.y * targetVectorLocal.y
            );
            double fx = L1 * cos(theta1);
            double fy = L1 * sin(theta1);
            double fz = targetVectorLocal.z;

            double fL1x = L1 - femurLength;//this does seem to be wrong

            double theta4fL1 = atan(fz / fL1x); // works

            //page2
            //works
            double fLength = sqrt(fz * fz + fL1x * fL1x);
            double theta2numerator
                = (ThighLength * ThighLength)
                + (fLength * fLength)
                - (footLength * footLength);
            double theta2denumenator = 2 * ThighLength * fLength;

            //TODO: fix problem, were numerator / denumenator gets smaller than -1 --> foot way up high...
            double theta2and4 = acos(theta2numerator / theta2denumenator);
            double theta2 = theta2and4 - theta4fL1;

            double theta3numerator = 
                pow(ThighLength, 2) + 
                pow(footLength, 2) - 
                pow(fLength, 2);
            double theta3denumenator = 2 * ThighLength * footLength;
            double theta3;
            theta3 = acos((theta3numerator / theta3denumenator));

            returnAngles
                = MotionEngine::LegAngles
                (
                    MathEngine::FromRadToDeg(theta1),
                    MathEngine::FromRadToDeg(theta2),
                    MathEngine::FromRadToDeg(theta3)
                );

            bool breakFlag;
            if (isnan(returnAngles.hipJoint) ||
                isnan(returnAngles.lowerLegJoint) ||
                isnan(returnAngles.upperLegJoint))
            {
                //breakFlag = true;
                /*std::cout << "targetVec: x: " << targetVectorLocal.x << " y: " << targetVectorLocal.y << " z: " << targetVectorLocal.z << std::endl;
                throw std::out_of_range("nan in kinematics");*/ 
                if (errorRet != nullptr)
                {
                    *errorRet = -1;
                }
            }
        } while (bDebug);
        /*std::stringstream outputstream;
        outputstream << "Local vector passed:";
        outputstream << " x: " << targetVectorLocal.x;
        outputstream << " y: " << targetVectorLocal.y;
        outputstream << " z: " << targetVectorLocal.z;
        outputstream << std::endl;
        OutputDebugString(outputstream.str().c_str());*/
    }
    if (errorRet != nullptr)
    {
        errorRet = 0;
    }
    return returnAngles;
}


MotionEngine::LegAngles MotionEngine::CalculateLegAnglesWithGlobalVector
(
    Leg leg,
    MathEngine::CartesianVector targetVectorGlobal,
    MathEngine::CartesianVector* targetVectorLocal //out
)
{
    LegAngles returnAngles = LegAngles();
    {
        //PerformanceTimer timer = PerformanceTimer();
        bool bDebug = false;
        do
        {
            //TODO: add boundaries for angles

            //convert the global target vector to a lge-local one...
            double radius = leg.GetRadius();
            double theta0 = MathEngine::FromDegToRad(leg.GetTheta0inDegree());
            double thetaF = atan(targetVectorGlobal.y / targetVectorGlobal.x);
            double thetaF0 = thetaF - MathEngine::FromDegToRad(leg.GetTheta0inDegree());
            double LF = sqrt(targetVectorGlobal.x * targetVectorGlobal.x + targetVectorGlobal.y * targetVectorGlobal.y);
            double L1 = sqrt(radius * radius + LF * LF - 2 * radius * LF * cos(thetaF0));

            double fRotationMatrixComToS[3][3]
            {
                {cos(-theta0), -sin(-theta0), 0}, //row 0
                {sin(-theta0), cos(-theta0), 0 }, //row 1
                {0, 0, 1}                         //row 2
            };
            double fTranslationVector[3]
            {
                -radius * cos(theta0),
                -radius * sin(theta0),
                0
            };
            double fTargetVectorLocalCopy[3]
            {
                targetVectorGlobal.x,
                targetVectorGlobal.y,
                targetVectorGlobal.z
            };

            double targetVectorInHipJointCoord[3];

            //translate the endpoint of the targetvector from the 
            //COM-Coordinatesystem to the Coordinatesystem of the hipjoint
            //does not work correctly
            for (int i = 0; i < 3; i++)
            {
                targetVectorInHipJointCoord[i] =
                    fRotationMatrixComToS[i][0] * (fTargetVectorLocalCopy[0] + fTranslationVector[0]) +
                    fRotationMatrixComToS[i][1] * (fTargetVectorLocalCopy[1] + fTranslationVector[1]) +
                    fRotationMatrixComToS[i][2] * (fTargetVectorLocalCopy[2] + fTranslationVector[2]);
            }
            MathEngine::CartesianVector localTargetVector =
                MathEngine::CartesianVector
                (
                    targetVectorInHipJointCoord[0],
                    targetVectorInHipJointCoord[1],
                    targetVectorInHipJointCoord[2]
                );
            if (targetVectorLocal != nullptr)
            {
                *targetVectorLocal = localTargetVector;
            }
            //from here on its the local hip joint leg coordinates...
            //double theta1 = atan(targetVectorInHipJointCoord[1] / targetVectorInHipJointCoord[0]);
            //L1 = sqrt
            //(
            //    targetVectorInHipJointCoord[0] * targetVectorInHipJointCoord[0] +
            //    targetVectorInHipJointCoord[1] * targetVectorInHipJointCoord[1]
            //);
            //double fx = L1 * cos(theta1);
            //double fy = L1 * sin(theta1);
            //double fz = targetVectorGlobal.z;
            //double fL1x = L1 - leg.GetFemurLength();//this does seem to be wrong
            //
            //double theta4fL1 = atan(fz / fL1x); // works
            ////page2
            ////works
            //double fLength = sqrt(fz * fz + fL1x * fL1x);
            //double theta2numerator
            //    = (leg.GetThighLength() * leg.GetThighLength())
            //    + (fLength * fLength)
            //    - (leg.GetFootLength() * leg.GetFootLength());
            //double theta2denumenator = 2 * leg.GetThighLength() * fLength;
            ////TODO: fix problem, were numerator / denumenator gets smaller than -1 --> foot way up high...
            //double theta2and4 = acos(theta2numerator / theta2denumenator);
            //double theta2 = theta2and4 - theta4fL1;
            //double theta3numerator = (sin(theta2and4) * fLength);
            //double theta3denumenator = leg.GetFootLength();
            //double theta3;
            //if (abs(theta3numerator - theta3denumenator) < 0.00001)
            //{
            //    theta3 = asin(1);
            //}
            //else
            //{
            //    theta3 = asin((theta3numerator / theta3denumenator));
            //}
            //returnAngles
            //    = MotionEngine::LegAngles
            //    (
            //        MathEngine::FromRadToDeg(theta1),
            //        MathEngine::FromRadToDeg(theta2),
            //        MathEngine::FromRadToDeg(theta3)
            //    );
            returnAngles = MotionEngine::CalculateLegAnglesWithLocalVector(leg, localTargetVector);

            bool breakFlag;
            if (isnan(returnAngles.hipJoint) ||
                isnan(returnAngles.lowerLegJoint) ||
                isnan(returnAngles.upperLegJoint))
            {
                //breakFlag = true;
            }
        } while (bDebug);
    }
    return returnAngles;
}

bool MotionEngine::MoveLegWithGlobalVector
(
    int hComm, 
    Leg* legPtr, 
    MathEngine::CartesianVector targetVectorLocal, 
    unsigned int nTime
)
{
    bool bRet = true;
    double currentHipAngle = legPtr->GetHipJointServo()->GetCurrentAngle();
    double currentUpperLegAngle = legPtr->GetUpperLegJointServo()->GetCurrentAngle();
    double currentLowerLegAngle = legPtr->GetLowerLegJointServo()->GetCurrentAngle();
    MotionEngine::LegAngles angles = CalculateLegAnglesWithGlobalVector(*legPtr, targetVectorLocal);

    double maxHipAngle = legPtr->GetHipJointServo()->GetMaxAngle();
    double minHipAngle = legPtr->GetHipJointServo()->GetMinAngle();
    double maxUpperLegJointpAngle = legPtr->GetUpperLegJointServo()->GetMaxAngle();
    double minUpperLegJointAngle = legPtr->GetUpperLegJointServo()->GetMinAngle();
    double maxLowerLegJointAngle = legPtr->GetLowerLegJointServo()->GetMaxAngle();
    double minLowerLegJointAngle = legPtr->GetLowerLegJointServo()->GetMinAngle();

    bool hipAngleOutOfBounds 
        = angles.hipJoint >= maxHipAngle ||
        angles.hipJoint <= minHipAngle;
    bool upperLegAngleOutOfBounds 
        = angles.upperLegJoint >= maxUpperLegJointpAngle ||
        angles.upperLegJoint <= minUpperLegJointAngle;
    bool lowerLegAngleOutOfBounds 
        = angles.lowerLegJoint >= maxLowerLegJointAngle ||
        angles.lowerLegJoint <= minLowerLegJointAngle;

    bool bAngleOutOfBounds
        = hipAngleOutOfBounds ||
        upperLegAngleOutOfBounds ||
        lowerLegAngleOutOfBounds;

    if (bAngleOutOfBounds)
    {
        bRet = false;
    }
    else
    {
        auto servoPtrs
            = std::vector<Servo*>
        {
            legPtr->GetHipJointServo(),
            legPtr->GetUpperLegJointServo(),
            legPtr->GetLowerLegJointServo()
        };

        auto positions
            = std::vector<double>
        {
            angles.hipJoint,
            angles.upperLegJoint,
            angles.lowerLegJoint
        };
        MoveGroupWithAngles(hComm, servoPtrs, positions, 0, false);
    }
    return bRet;
}

int MotionEngine::MoveLegsWithLocalVector
(
    int hComm,
    std::vector<Leg*> legPtrs,
    MathEngine::CartesianVector targetVectorLocal,
    unsigned int nTime
)
{
    int nRet = 0;
    auto servoPtrs
        = std::vector<Servo*>();
    auto positions
        = std::vector<double>();
    for (int i = 0; i< 6; i++)
    {
        double currentHipAngle = legPtrs[i]->GetHipJointServo()->GetCurrentAngle();
        double currentUpperLegAngle = legPtrs[i]->GetUpperLegJointServo()->GetCurrentAngle();
        double currentLowerLegAngle = legPtrs[i]->GetLowerLegJointServo()->GetCurrentAngle();
        int nLocalRet = 0;
        MotionEngine::LegAngles angles 
            = CalculateLegAnglesWithLocalVector
            (
                *legPtrs[i], 
                targetVectorLocal, 
                &nLocalRet
            );
        if (-1 == nLocalRet)
        {
            nRet = -1;
            break;
        }
        else
        {
            double maxHipAngle = legPtrs[i]->GetHipJointServo()->GetMaxAngle();
            double minHipAngle = legPtrs[i]->GetHipJointServo()->GetMinAngle();
            double maxUpperLegJointpAngle = legPtrs[i]->GetUpperLegJointServo()->GetMaxAngle();
            double minUpperLegJointAngle = legPtrs[i]->GetUpperLegJointServo()->GetMinAngle();
            double maxLowerLegJointAngle = legPtrs[i]->GetLowerLegJointServo()->GetMaxAngle();
            double minLowerLegJointAngle = legPtrs[i]->GetLowerLegJointServo()->GetMinAngle();

            bool hipAngleOutOfBounds
                = angles.hipJoint >= maxHipAngle ||
                angles.hipJoint <= minHipAngle;
            bool upperLegAngleOutOfBounds
                = angles.upperLegJoint >= maxUpperLegJointpAngle ||
                angles.upperLegJoint <= minUpperLegJointAngle;
            bool lowerLegAngleOutOfBounds
                = angles.lowerLegJoint >= maxLowerLegJointAngle ||
                angles.lowerLegJoint <= minLowerLegJointAngle;

            bool bAngleOutOfBounds
                = hipAngleOutOfBounds ||
                upperLegAngleOutOfBounds ||
                lowerLegAngleOutOfBounds;

            if (bAngleOutOfBounds)
            {
                nRet = -2;
            }
            else
            {
                servoPtrs.push_back(legPtrs[i]->GetHipJointServo());
                servoPtrs.push_back(legPtrs[i]->GetUpperLegJointServo());
                servoPtrs.push_back(legPtrs[i]->GetLowerLegJointServo());

                positions.push_back(angles.hipJoint);
                positions.push_back(angles.upperLegJoint);
                positions.push_back(angles.lowerLegJoint);
            }
            /*if (i % 3 == 2)
            {
                MoveGroupWithAngles(hComm, servoPtrs, positions, 0, false);
                servoPtrs.clear();
                positions.clear();
                Sleep(50);
            }*/
        }
    }
    if (nRet == 0)
    {
        MoveGroupWithAngles(hComm, servoPtrs, positions, nTime, false);
    }
    return nRet;
}

int MotionEngine::MoveLegWithLocalVector
(
    int hComm, 
    Leg* legPtr, 
    MathEngine::CartesianVector targetVectorLocal, 
    unsigned int nTime
)
{
    int nRet = 0;
    bool debug = false;
    do
    {
        double currentHipAngle = legPtr->GetHipJointServo()->GetCurrentAngle();
        double currentUpperLegAngle = legPtr->GetUpperLegJointServo()->GetCurrentAngle();
        double currentLowerLegAngle = legPtr->GetLowerLegJointServo()->GetCurrentAngle();
        MotionEngine::LegAngles angles;
        /*try
        {*/
        int nRetLocal = 0;
        angles = CalculateLegAnglesWithLocalVector(*legPtr, targetVectorLocal, &nRet);
        if (nRet == -1)
        {
            nRet = -1;
        }
        else
        {
            double maxHipAngle = legPtr->GetHipJointServo()->GetMaxAngle();
            double minHipAngle = legPtr->GetHipJointServo()->GetMinAngle();
            double maxUpperLegJointpAngle = legPtr->GetUpperLegJointServo()->GetMaxAngle();
            double minUpperLegJointAngle = legPtr->GetUpperLegJointServo()->GetMinAngle();
            double maxLowerLegJointAngle = legPtr->GetLowerLegJointServo()->GetMaxAngle();
            double minLowerLegJointAngle = legPtr->GetLowerLegJointServo()->GetMinAngle();

            bool hipAngleOutOfBounds
                = angles.hipJoint >= maxHipAngle ||
                angles.hipJoint <= minHipAngle;
            bool upperLegAngleOutOfBounds
                = angles.upperLegJoint >= maxUpperLegJointpAngle ||
                angles.upperLegJoint <= minUpperLegJointAngle;
            bool lowerLegAngleOutOfBounds
                = angles.lowerLegJoint >= maxLowerLegJointAngle ||
                angles.lowerLegJoint <= minLowerLegJointAngle;

            bool bAngleOutOfBounds
                = hipAngleOutOfBounds ||
                upperLegAngleOutOfBounds ||
                lowerLegAngleOutOfBounds;

            if (bAngleOutOfBounds)
            {
                nRet = -2;
            }
            else
            {
                auto servoPtrs
                    = std::vector<Servo*>
                {
                    legPtr->GetHipJointServo(),
                    legPtr->GetUpperLegJointServo(),
                    legPtr->GetLowerLegJointServo()
                };

                auto positions
                    = std::vector<double>
                {
                    angles.hipJoint,
                    angles.upperLegJoint,
                    angles.lowerLegJoint
                };
                if (isnan(angles.hipJoint) ||
                    isnan(angles.upperLegJoint) ||
                    isnan(angles.lowerLegJoint))
                {
                    //debug = true;
                }
                std::string cmd;
                MoveGroupWithAngles(hComm, servoPtrs, positions, 0, false, &cmd);
                legPtr->SetLastCommand(cmd);

            }
        }
        //}
        /*
        catch (const std::exception& e)
        {
            if (e.what() == "nan in kinematics")
            {
                throw std::out_of_range("nan in kinematics");
            }
        }*/
    } while (debug);
    return nRet;
}

int MotionEngine::DoWalk(int nDirectionX, int nDirectionY) //both are at max 100...
{
    int nRet = 0;
    
#pragma region make these constants
    float fScalingForDirection = 0.2;
    int nMaxValueForDirection = 100;
    float fMaxValueForSpeed = 1.2; //m/s
    float fSclaingForSpeed = fMaxValueForSpeed / (float)nMaxValueForDirection;
#pragma endregion

    float fSpeed = fScalingForDirection * sqrtf((float)(pow(nDirectionX, 2) + pow(nDirectionY, 2)));
    //pseudocode:
    //(find out, which part of this could possibly be executed asynchronously)

    //if directionvector (fDirectionX, fDirectionY) has changed to previous one (save internally)
    //calculate new targetVectors for legs
    //  define neutral standing point for legs (done in LegConstans.h)
    //  apply the directionVector to neutral standing point --> that is P2
    if (nDirectionX != m_nLastDirectionX || nDirectionY != m_nLastDirectionY)
    {
        //do something clever with the groups here, if the pod was moving, idk...
        
        //scale the direction to apply it to the neutral standing point
        float fDirectionX = fScalingForDirection * (float)nDirectionX;
        float fDirectionY = fScalingForDirection * (float)nDirectionY;
        
        //calculate P2 for all legs
        double fP2XTravel = legConst_neutralPointX + fDirectionX;
        double fP2YTravel = legConst_neutralPointY + fDirectionY;

        double fP2XShift = legConst_neutralPointX - fDirectionX;
        double fP2YShift = legConst_neutralPointY - fDirectionY;

        MathEngine::CartesianVector currentFootVec;

        //calculate traversal function for each leg
        //calculate deltas
        for (int idxLeg = 0; idxLeg < 6; idxLeg++)
        {
            currentFootVec =
                MotionEngine::GetCurrentCartesianVectorOfFootLocal(*m_pLegPtrs[idxLeg]);
            if ((idxLeg % 2) == m_bShiftGroupIsOdds) //index is in shiftgroup
            {
                m_afDeltaX[idxLeg] = fP2XShift - currentFootVec.x;
                m_afDeltaY[idxLeg] = fP2YShift - currentFootVec.y;
            }
            else if ((idxLeg % 2) == !m_bShiftGroupIsOdds) //index is in travelGroup
            {
                m_afDeltaX[idxLeg] = fP2XTravel - currentFootVec.x;
                m_afDeltaY[idxLeg] = fP2YTravel - currentFootVec.y;
            }
        }
        //calculate max length of traversal-vectors


        m_nLastDirectionX = nDirectionX;
        m_nLastDirectionY = nDirectionY;
    }
    
    //calculate functions (or rather the factors of the functions) 
    //between current point (P1) and P2, the target point
    //  two different cases:
    //      1: "shift" --> the foot is "shifted" over the floor, along a linear
    //          function, this actually moves the robot
    //      2: "travel" --> the foot is repositioned through the air
    //          --> use cubic spline-interpolation to calculate the z-function,
    //              the calculation of x(t) and y(t) stays the same
    //--> define shift-group and travel-group and swap them, if the current phase is over

    //--> currently not sure, how to be able to guarantee the desired speed in every constellation of the legs...
    //  --> first shift the three legs, which can move the farest

    //add calculation of boundaries for legs... 
    //define sector delimited by two lines, which extend from the hipjoint 
    //and two lines, which extend from the center of the pod...


    //set up chrono-loop
    //set cycle-time for chrono loop
    //set cycle-time for writing to hardware, may have limitations

    //calculate the current x, y & z values for each leg, generate local target vectors for the legs
    //use inverse kinematics to move the legs (MotionEngine::MoveLeg)

    return nRet;
}

int MotionEngine::TestWalk(int hComm, int nDirectionX, int nDirectionY)
{
    int nRet = 0;

#pragma region make these constants
    double fTestSpeed = 0.1;
    /*double fScalingForDirection = 0.2;
    int nMaxValueForDirection = 100;
    double fMaxValueForSpeed = 1.2;*/ //m/s5
    //double fSclaingForSpeed = fMaxValueForSpeed / (float)nMaxValueForDirection;
#pragma endregion

    //double fSpeed = fScalingForDirection * sqrtf((float)(pow(nDirectionX, 2) + pow(nDirectionY, 2)));
    
    //TODO: find out, which leg should be shifted first...
    if (nDirectionX != m_nLastDirectionX || nDirectionY != m_nLastDirectionY)
    {
        if (m_nLastDirectionX == 0 && m_nLastDirectionY == 0)
        {
            double fDirectionAngle = 0;
            if (nDirectionY != 0)
            {
                if (nDirectionX == 0 && nDirectionY > 0)
                {
                    fDirectionAngle = M_PI_2;
                }
                else if (nDirectionX == 0 && nDirectionY < 0)
                {
                    fDirectionAngle = M_PI_2 * 3;
                }
                else
                {
                    fDirectionAngle = atan((double)nDirectionX / (double)nDirectionY);
                }

            }
            else
            {
                if (nDirectionX > 0)
                {
                    fDirectionAngle = 0;
                }
                else if (nDirectionX < 0)
                {
                    fDirectionAngle = M_PI;
                }
                else
                {
                    fDirectionAngle = 0;
                }
            }
            double fShiftedAngle = fmod(fDirectionAngle + M_PI, M_PI * 2);
            int shiftDeterminatorIdx = 0;
            //find the leg, thats theta0 is nearest to fShiftedAngle...
            for (int idxLeg = 0; idxLeg < 6; idxLeg++)
            {
                double currentTheta0 = MathEngine::FromDegToRad(m_pLegPtrs[idxLeg]->GetTheta0inDegree());
                double check = M_PI_2 * (1.0 / 3.0);
                double minus = fShiftedAngle - currentTheta0;
                if (abs(minus) < check)
                {
                    shiftDeterminatorIdx = idxLeg;
                    break;
                }
            }
            m_bShiftGroupsSwitched = true;
            if (shiftDeterminatorIdx % 2)
            {
                m_bShiftGroupIsOdds = true;
            }
            else
            {
                m_bShiftGroupIsOdds = false;
            }
        }

        m_nLastDirectionX = nDirectionX;
        m_nLastDirectionY = nDirectionY;
    }

    if (m_bShiftGroupsSwitched)
    {
        //do something clever with the groups here, if the pod was moving, idk...

    //calculate P2 for all legs --> TODO: dont do this with fixed values
    //these are local vectors

    //calculate the global targetvectors
        MathEngine::CartesianVector localNeutralVector
            = MathEngine::CartesianVector
            (
                legConst_neutralPointX,
                legConst_neutralPointY,
                legConst_neutralPointZ
            );
            //#ifndef _ARM_
            //    std::stringstream outputstream;
            //    outputstream << "shiftgroupisodd: " << m_bShiftGroupIsOdds;
            //    outputstream << std::endl;
            //    OutputDebugString(outputstream.str().c_str());
            //#endif // !_ARM_

        for (int idxLeg = 0; idxLeg < 6; idxLeg++)
        {
            MathEngine::CartesianVector tempGlobalTarget = MathEngine::CartesianVector();
            if ((idxLeg % 2) == m_bShiftGroupIsOdds) //index is in shiftgroup
            {
                tempGlobalTarget
                    = MotionEngine::ConvertFromLocalToGlobalVector
                    (
                        *m_pLegPtrs[idxLeg],
                        localNeutralVector
                    );
                tempGlobalTarget.x += 50;//TODO: add something clever here, this is for testing
                tempGlobalTarget.y += 0;
            }
            else if ((idxLeg % 2) == !m_bShiftGroupIsOdds)
            {
                tempGlobalTarget
                    = MotionEngine::ConvertFromLocalToGlobalVector
                    (
                        *m_pLegPtrs[idxLeg],
                        localNeutralVector
                    );
                tempGlobalTarget.x -= 50;
                tempGlobalTarget.y -= 0;
            }
            m_targetVectors[idxLeg]
                = MotionEngine::ConvertFromGlobalToLocalVector
                (
                    *m_pLegPtrs[idxLeg],
                    tempGlobalTarget
                );
            //#ifndef _ARM_
            //std::stringstream outputstream;
            //outputstream << "Leg" << idxLeg << ": x: " << tempGlobalTarget.x << " y: " << tempGlobalTarget.y << " z: " << tempGlobalTarget.z << std::endl;
            //OutputDebugString(outputstream.str().c_str());
            //#endif // !_ARM_

        }
        m_bShiftGroupsSwitched = false;
    }

    int nResult = 0;
    for (int idxLeg = 0; idxLeg < 6; idxLeg++)
    {
        if ((idxLeg % 2) == m_bShiftGroupIsOdds) //index is in shiftgroup
        {
            nResult += TraverseTo
            (
                hComm,
                idxLeg,
                m_targetVectors[idxLeg],
                false,
                fTestSpeed//,
                //currentTimePoint
            );
        }
        else if ((idxLeg % 2) == !m_bShiftGroupIsOdds)
        {
            nResult += TraverseTo
            (
                hComm,
                idxLeg,
                m_targetVectors[idxLeg],
                true,
                fTestSpeed//,
                //currentTimePoint
            );
        }
    }
    if (0 == nResult)
    {
        //switch
        //bTravel = !bTravel;
        //nIterationCounter++;
        m_bShiftGroupIsOdds = !m_bShiftGroupIsOdds;
        m_bShiftGroupsSwitched = true;
    }
    return nRet;
}

//bool MotionEngine::AngleIsInBlockingRange(double fAngleToCheck, double fAngleWithRange)
//{
//    double fAngleToCheckMod = fmod(fAngleToCheck + 2 * M_PI, 2 * M_PI);
//    double fAngleWithRangeMaxMod = fmod(fAngleWithRange + m_blockingAngleRange + 2 * M_PI, 2 * M_PI);
//    double fAngleWithRangeMinMod = fmod(fAngleWithRange - m_blockingAngleRange + 2 * M_PI, 2 * M_PI);
//    bool bIsInRange = fAngleToCheckMod < fAngleWithRangeMaxMod && fAngleToCheckMod > fAngleWithRangeMinMod;
//    return bIsInRange;
//}

int MotionEngine::Walk(int hComm, int nDirectionX, int nDirectionY)
{
    int nRet = 0;
    int nRadiusToApplyToNeutralPoint = 35;
    bool bMove = false;
    double fMaxSpeed = 0.15;
    //double fMaxSpeed = 0.03;
    double fSpeedToSet = fMaxSpeed;
    double fMaxVelocity = 100;
    double fMinVelocity = 25;
    double fScalingForDirection = 0.35;
    int nMaxValueForDirection = 100;
    int nMinValueForDirection = 25;

    //apply boundaries for X
    if (nDirectionX > 0)
    {
        if (nDirectionX > nMaxValueForDirection)
        {
            nDirectionX = nMaxValueForDirection;
        }
        else if (nDirectionX < nMinValueForDirection)
        {
            nDirectionX = nMinValueForDirection;
        }
    }
    else if (nDirectionX < 0)
    {
        if (nDirectionX < -nMaxValueForDirection)
        {
            nDirectionX = -nMaxValueForDirection;
        }
        else if (nDirectionX > -nMinValueForDirection)
        {
            nDirectionX = -nMinValueForDirection;
        }
    }
    
    //apply boundaries for Y
    if (nDirectionY > 0)
    {
        if (nDirectionY > nMaxValueForDirection)
        {
            nDirectionY = nMaxValueForDirection;
        }
        else if (nDirectionY < nMinValueForDirection)
        {
            nDirectionY = nMinValueForDirection;
        }
    }
    else if (nDirectionY < 0)
    {
        if (nDirectionY < -nMaxValueForDirection)
        {
            nDirectionY = -nMaxValueForDirection;
        }
        else if (nDirectionY > -nMinValueForDirection)
        {
            nDirectionY = -nMinValueForDirection;
        }
    }

    //find out, which leg should be shifted first...
    if ((nDirectionX != m_nLastDirectionX || nDirectionY != m_nLastDirectionY) && m_bIsFinishedTraversing)
    {
        StopTraversal();//testing
        if (m_nLastDirectionX == 0 && m_nLastDirectionY == 0)
        {
            double fDirectionAngle = 0;
            if (nDirectionY != 0)
            {
                if (nDirectionX == 0 && nDirectionY > 0)
                {
                    fDirectionAngle = M_PI_2;
                }
                else if (nDirectionX == 0 && nDirectionY < 0)
                {
                    fDirectionAngle = M_PI_2 * 3;
                }
                else
                {
                    fDirectionAngle = atan((double)nDirectionY / (double)nDirectionX);
                }
            }
            else
            {
                if (nDirectionX > 0)
                {
                    fDirectionAngle = 0;
                }
                else if (nDirectionX < 0)
                {
                    fDirectionAngle = M_PI;
                }
                else
                {
                    fDirectionAngle = 0;
                }
            }
            if (nDirectionX < 0)
            {
                fDirectionAngle += M_PI;
            }
            //m_fCurrentDirectionAngle = fDirectionAngle;

            double fShiftedAngle = fmod(fDirectionAngle + M_PI, M_PI * 2);
            int shiftDeterminatorIdx = 0;
            //find the leg, thats theta0 is nearest to fShiftedAngle...
            for (int idxLeg = 0; idxLeg < 6; idxLeg++)
            {
                double currentTheta0 = MathEngine::FromDegToRad(m_pLegPtrs[idxLeg]->GetTheta0inDegree());
                double check = M_PI_2 * (1.0 / 3.0);
                double minus = fShiftedAngle - currentTheta0;
                if (abs(minus) < check)
                {
                    shiftDeterminatorIdx = idxLeg;
                    break;
                }
            }
            m_bShiftGroupsSwitched = true;
            if (shiftDeterminatorIdx % 2)
            {
                m_bShiftGroupIsOdds = true;
            }
            else
            {
                m_bShiftGroupIsOdds = false;
            }
        }

        m_nLastDirectionX = nDirectionX;
        m_nLastDirectionY = nDirectionY;
    }

    MathEngine::CartesianVector localNeutralVector
        = MathEngine::CartesianVector
        (
            legConst_neutralPointX,
            legConst_neutralPointY,
            legConst_neutralPointZ
        );
    if (nDirectionX == 0 && nDirectionY == 0)
    {
        int result = MoveHome(hComm, 0.08);
        nRet = result;
        /*if (-1 == result)
        {
            throw std::out_of_range("error in traversal");
        }*/
    }
    else if ((nDirectionX != 0 || nDirectionY != 0) && m_bShiftGroupsSwitched)
    {
        //m_bMove = true;

        //TODO: test this
        /*if (m_bDirectionBlocked)
        {
            std::cout << "reset direction blocked!" << std::endl;
            m_bDirectionBlocked = false;
            m_fBlockedDirectionAngle = 0.0;
        }*/
        double fDirectionAngle = 0;
        if (nDirectionY != 0)
        {
            if (nDirectionX == 0 && nDirectionY > 0)
            {
                fDirectionAngle = M_PI_2;
            }
            else if (nDirectionX == 0 && nDirectionY < 0)
            {
                fDirectionAngle = M_PI_2 * 3;
            }
            else
            {
                fDirectionAngle = atan((double)nDirectionY / (double)nDirectionX);
            }
        }
        else
        {
            if (nDirectionX > 0)
            {
                fDirectionAngle = 0;
            }
            else if (nDirectionX < 0)
            {
                fDirectionAngle = M_PI;
            }
            else
            {
                fDirectionAngle = 0;
            }
        }
        if (nDirectionX < 0)
        {
            fDirectionAngle += M_PI;
        }

        //calculate scaled x & y to apply to neutral point
        double fVelocity = sqrt(pow(nDirectionX, 2) + pow(nDirectionY, 2));
        if (fVelocity > fMaxVelocity)
        {
            fVelocity = fMaxVelocity;
        }
        else if (fVelocity < fMinVelocity)
        {
            fVelocity = fMinVelocity;
        }
        fSpeedToSet = fMaxSpeed * (fVelocity / fMaxVelocity);
        //m_fCurrentSpeed = fSpeedToSet;

        //apply scaling for direction
        int nScaledX = /*fScalingForDirection * (double)nDirectionX;*/
            nRadiusToApplyToNeutralPoint * cos(fDirectionAngle);

        int nScaledY = /*fScalingForDirection * (double)nDirectionY;*/ 
            nRadiusToApplyToNeutralPoint * sin(fDirectionAngle);

        //calculating the target-vectors here:
        //1. get the global equivalent of the local neutral vector for each leg
        //2. apply offset (movement-direction) to the neutral point globally
        //3. convert the global target-vectors to local ones
        for (int idxLeg = 0; idxLeg < 6; idxLeg++)
        {
            MathEngine::CartesianVector tempGlobalTarget = MathEngine::CartesianVector();
            if ((idxLeg % 2) == m_bShiftGroupIsOdds) //index is in shiftgroup
            {
                tempGlobalTarget
                    = MotionEngine::ConvertFromLocalToGlobalVector
                    (
                        *m_pLegPtrs[idxLeg],
                        localNeutralVector
                    );
                tempGlobalTarget.x += nScaledX; //TODO: this is problematic, because the legs dont have the same degree of freedom in every direction
                tempGlobalTarget.y += nScaledY;
            }
            else if ((idxLeg % 2) == !m_bShiftGroupIsOdds)
            {
                tempGlobalTarget
                    = MotionEngine::ConvertFromLocalToGlobalVector
                    (
                        *m_pLegPtrs[idxLeg],
                        localNeutralVector
                    );
                tempGlobalTarget.x -= nScaledX; 
                tempGlobalTarget.y -= nScaledY;
            }
            m_targetVectors[idxLeg]
                = MotionEngine::ConvertFromGlobalToLocalVector
                (
                    *m_pLegPtrs[idxLeg],
                    tempGlobalTarget
                );

        }
        m_bShiftGroupsSwitched = false;
        m_bMove = true;
    }

    if (m_bMove)
    {
        int nResult = 0;
        int nLocalResult = 0;
        m_homingState = MotionEngine::HomingState::isNotAtHome;
        for (int idxLeg = 0; idxLeg < 6; idxLeg++)
        {
            if ((idxLeg % 2) == m_bShiftGroupIsOdds) //index is in shiftgroup
            {
                nLocalResult = TraverseTo
                (
                    hComm,
                    idxLeg,
                    m_targetVectors[idxLeg],
                    false,
                    fSpeedToSet
                );
                if (-1 == nLocalResult)
                {
                    nRet = -1;
                    break;
                }
                nResult += nLocalResult;
            }
            else if ((idxLeg % 2) == !m_bShiftGroupIsOdds)
            {
                nLocalResult = TraverseTo
                (
                    hComm,
                    idxLeg,
                    m_targetVectors[idxLeg],
                    true,
                    fSpeedToSet
                );
                if (-1 == nLocalResult)
                {
                    /*throw std::out_of_range("error in traversal");*/
                    nRet = -1;
                    break;
                }
                nResult += nLocalResult;
            }
        }
        if (0 == nResult)
        {
            //switch shiftgroup
            m_bShiftGroupIsOdds = !m_bShiftGroupIsOdds;
            m_bShiftGroupsSwitched = true;
        }
    }

    m_lastHomingState = m_homingState;
    return nRet;
}

int MotionEngine::CalculateLocalTurnVectors
(
    double turnAngleDegree, 
    MathEngine::CartesianVector* pTurnLeft, 
    MathEngine::CartesianVector* pTurnRight)
{
    //generate TargetVectors on circle
    MathEngine::CartesianVector turnRightTarget;
    MathEngine::CartesianVector turnLeftTarget;

    MathEngine::CartesianVector neutralLocalVector =
        MathEngine::CartesianVector
        (
            legConst_neutralPointX,
            legConst_neutralPointY,
            legConst_neutralPointZ
        );

    auto globalneutralVector = ConvertFromLocalToGlobalVector(*m_pLegPtrs[0], neutralLocalVector);
    double turnAngle = M_PI * 2 * (turnAngleDegree / 360); // turns left
    double turnedX = globalneutralVector.x * cos(turnAngle) - globalneutralVector.y * sin(turnAngle);//global
    double turnedY = globalneutralVector.x * sin(turnAngle) + globalneutralVector.y * cos(turnAngle);//global
    MathEngine::CartesianVector turnedGlobalVectorLeft =
        MathEngine::CartesianVector
        (
            turnedX,
            turnedY,
            globalneutralVector.z
        );
    m_localTurnLeft = ConvertFromGlobalToLocalVector(*m_pLegPtrs[0], turnedGlobalVectorLeft);
    if (nullptr != pTurnLeft)
    {
        *pTurnLeft = m_localTurnLeft;
    }

    turnedX = globalneutralVector.x * cos(-turnAngle) - globalneutralVector.y * sin(-turnAngle);//global
    turnedY = globalneutralVector.x * sin(-turnAngle) + globalneutralVector.y * cos(-turnAngle);//global
    MathEngine::CartesianVector turnedGlobalVectorRight =
        MathEngine::CartesianVector
        (
            turnedX,
            turnedY,
            globalneutralVector.z
        );
    m_localTurnRight = ConvertFromGlobalToLocalVector(*m_pLegPtrs[0], turnedGlobalVectorRight);
    if (nullptr != pTurnRight)
    {
        *pTurnRight = m_localTurnRight;
    }
    return 0;
}

int MotionEngine::Turn(int hComm, bool moveLeft)
{
    /*if (m_bDirectionBlocked)
    {
        m_errors->push_back("direction was blocked, turning disabled");
    }
    else
    {*/
    int nRet = 0;
    bool bShiftGroupIsOdds = true;
    double turnSpeed = 0.12;
    if ((m_localTurnLeft.x == 0 && m_localTurnLeft.y == 0)
        || (m_localTurnRight.x == 0 && m_localTurnRight.y == 0))
    {
        CalculateLocalTurnVectors(7);
    }
    //call traversal-method...

    int nResult = 0;
    int nLocalResult = 0;
    m_homingState = MotionEngine::HomingState::isNotAtHome;
    for (int idxLeg = 0; idxLeg < 6; idxLeg++)
    {
        if ((idxLeg % 2) == m_bShiftGroupIsOdds) //index is in shiftgroup
        {
            if (moveLeft)
            {
                nLocalResult = TraverseTo//shift and move leg right
                (
                    hComm,
                    idxLeg,
                    m_localTurnRight,
                    false,
                    turnSpeed,
                    true,
                    false
                ); 
                if (-1 == nLocalResult)
                {
                    //throw std::out_of_range("error in traversal");
                    nRet = -1;
                    break;
                }
                nResult += nLocalResult;
            }
            else
            {
                nLocalResult = TraverseTo//shift and move leg left
                (
                    hComm,
                    idxLeg,
                    m_localTurnLeft,
                    false,
                    turnSpeed,
                    true,
                    true
                );
                if (-1 == nLocalResult)
                {
                    //throw std::out_of_range("error in traversal");
                    nRet = -1;
                    break;
                }
                nResult += nLocalResult;
            }

        }
        else if ((idxLeg % 2) == !m_bShiftGroupIsOdds)
        {
            if (moveLeft)
            {
                nLocalResult = TraverseTo//travel and move leg left
                (
                    hComm,
                    idxLeg,
                    m_localTurnLeft,
                    true,
                    turnSpeed,
                    true,
                    true
                );
                if (-1 == nLocalResult)
                {
                    //throw std::out_of_range("error in traversal");
                    nRet = -1;
                    break;
                }
                nResult += nLocalResult;
            }
            else
            {
                nLocalResult = TraverseTo//travel and move leg right
                (
                    hComm,
                    idxLeg,
                    m_localTurnRight,
                    true,
                    turnSpeed,
                    true,
                    false
                );
                if (-1 == nLocalResult)
                {
                    //throw std::out_of_range("error in traversal");
                    nRet = -1;
                    break;
                }
                nResult += nLocalResult;
            }
        }
    }
    if (0 == nResult)
    {
        //switch shiftgroup
        m_bShiftGroupIsOdds = !m_bShiftGroupIsOdds;
        m_bShiftGroupsSwitched = true;
        std::cout << "switched shiftgroup" << std::endl;
    }
    
    return nRet;
}

int MotionEngine::TestTraversalOfOneLeg(int hComm, int idxLeg, double fSpeed)
{
    bool bBreak = false;
    bool bTravel = false; // if false --> shift, else travel
    int nIterationCounter = 0;
    MathEngine::CartesianVector targetVector;
    std::chrono::high_resolution_clock::time_point currentTimePoint;

    while (nIterationCounter < 10)
    {
        if (!bTravel)
        {
            //shift
            targetVector.x = legConst_neutralPointX - 10;
            targetVector.y = legConst_neutralPointY + 50;
            targetVector.z = legConst_neutralPointZ;
        }
        else
        {
            //travel
            targetVector.x = legConst_neutralPointX + 30;
            targetVector.y = legConst_neutralPointY - 60;
            targetVector.z = legConst_neutralPointZ;
        }
        int nResult = TraverseTo
        (
            hComm, 
            idxLeg, 
            targetVector, 
            bTravel, 
            fSpeed
        );
        if (0 == nResult)
        {
            //switch
            bTravel = !bTravel;
            nIterationCounter++;
        }
    }
    return 0;
}

int MotionEngine::MoveHome(int hComm, double fSpeed)
{
    int nRet = 1;
    MathEngine::CartesianVector neutralVector =
    {
        legConst_neutralPointX,
        legConst_neutralPointY,
        legConst_neutralPointZ
    };
    m_bMove = false;
    if (m_homingState != MotionEngine::HomingState::isAtHome)
    {
        if (m_homingState != MotionEngine::HomingState::moveFirstGroupToHome &&
            m_homingState != MotionEngine::HomingState::moveSecondGroupToHome)
            m_homingState = MotionEngine::HomingState::stopTraversal;
        //move to homeposish

        if (m_homingState == MotionEngine::HomingState::stopTraversal)
        {
            StopTraversal();
            m_homingState = MotionEngine::HomingState::moveFirstGroupToHome;
        }
        int nResult = 0;
        int nLocalResult = 0;
        for (int idxLeg = 0; idxLeg < 6; idxLeg++)
        {
            if ((idxLeg % 2) == !m_bShiftGroupIsOdds)
            {
                nLocalResult = TraverseTo
                (
                    hComm,
                    idxLeg,
                    neutralVector,
                    true,
                    fSpeed
                );
                if (-1 == nLocalResult)
                {
                    //throw std::out_of_range("error in traversal");
                    nRet = -1;
                    break;
                }
                nResult += nLocalResult;
            }
        }
        if (0 == nResult)
        {
            //switch shiftgroup
            m_bShiftGroupIsOdds = !m_bShiftGroupIsOdds;
            m_bShiftGroupsSwitched = true;
            if (m_homingState == MotionEngine::HomingState::moveFirstGroupToHome)
            {
                m_homingState = MotionEngine::HomingState::moveSecondGroupToHome;
            }
            else if (m_homingState == MotionEngine::HomingState::moveSecondGroupToHome)
            {
                m_homingState = MotionEngine::HomingState::isAtHome;
                nRet = 0;
            }
        }
    }
    return nRet;
}

//int MotionEngine::TraverseOnCircle
//(
//    int hComm, 
//    int idxLeg, 
//    double angleInDegree, 
//    bool moveUp, 
//    double fSpeed
//)
//{
//    bool moveLeft = angleInDegree > 0;
//    //calculate spline for movement through x/y-plane
//    //calculate angle between current vector and targetvector
//    auto currentVectorGlobal = MotionEngine::GetCurrentCartesianVectorOfFootGlobal(*(m_pLegPtrs[idxLeg]));
//    auto targetVectorGlobal = ConvertFromLocalToGlobalVector(*m_pLegPtrs[idxLeg], targetVectorLocal);
//    double acosOperandNum = currentVectorGlobal.x * targetVectorGlobal.x + currentVectorGlobal.y * targetVectorGlobal.y;
//    double acosOperandDen = sqrt(pow(currentVectorGlobal.x, 2) + pow(currentVectorGlobal.y, 2)) * sqrt(pow(targetVectorGlobal.x, 2) + pow(targetVectorGlobal.y, 2));
//    double acosOperand = acosOperandNum / acosOperandDen;
//    if (acosOperand > 1.0)
//    {
//        acosOperand = 1;
//    }
//    double alpha = acos(acosOperand);
//    double neutralPointRadius = legConst_radius + legConst_neutralPointX;
//    double lengthOfSegment = (neutralPointRadius)*alpha;
//    //m_fCurrentTimeForTraversalms = lengthOfSegment / fSpeedInMPerS;
//
//    double turnAngle = MathEngine::FromDegToRad(angleInDegree);
//    double neutralPointRadius = legConst_radius + legConst_neutralPointX;
//
//    double lengthOfSegment = (neutralPointRadius)*turnAngle;
//    m_fCurrentTimeForTraversalms = lengthOfSegment / fSpeed;
//
//    int nNumberOfSupPoints = 10;
//    std::vector<double> fSegmentsX;
//    std::vector<double> fSegmentsY;
//    std::vector<double> fSegmentsTime;
//    double fSegmentX = 0;
//    double fSegmentY = 0;
//    double angleIncrement = turnAngle / nNumberOfSupPoints;
//    MathEngine::CartesianVector globalSegmentVector;
//    for (int i = 1; i <= nNumberOfSupPoints; i++)
//    {
//        double turnAngle;
//        //if left --> apply positive angle
//        if (moveLeft)
//        {
//            turnAngle = angleIncrement * i;
//        }
//        else
//        {
//            turnAngle = angleIncrement * -i;
//        }
//        fSegmentX = legConst_neutralPointX * cos(turnAngle) - legConst_neutralPointY * sin(turnAngle);//global
//        fSegmentY = legConst_neutralPointX * sin(turnAngle) + legConst_neutralPointY * cos(turnAngle);//global
//
//        globalSegmentVector.x = fSegmentX;
//        globalSegmentVector.y = fSegmentY;
//
//        auto localSegmentVector = ConvertFromGlobalToLocalVector(*m_pLegPtrs[idxLeg], globalSegmentVector);
//
//        fSegmentsX.push_back(localSegmentVector.x);
//        fSegmentsY.push_back(localSegmentVector.y);
//
//        fSegmentsTime.push_back((m_fCurrentTimeForTraversalms / nNumberOfSupPoints) * i);
//
//        if (nNumberOfSupPoints == i)
//        {
//            localSegmentVector.z = legConst_neutralPointZ; //temporary
//            m_currentTraversalTargetVector[idxLeg] = localSegmentVector;
//        }
//    }
//    tk::spline xSpline;
//    tk::spline ySpline;
//    xSpline.set_points(fSegmentsX, fSegmentsTime);
//    ySpline.set_points(fSegmentsY, fSegmentsTime);
//    m_splinesTurnX[idxLeg] = xSpline;
//    m_splinesTurnY[idxLeg] = ySpline;
//    
//    return 0;
//}

//use internal variables to check, if traversal works
//returns 0, if finished...
int MotionEngine::TraverseTo
(
    int hComm,
    int idxLeg, 
    MathEngine::CartesianVector targetVectorLocal, 
    bool moveUp, 
    double fSpeedInMPerS,
    bool onCircle,
    bool onCircleLeft
)
{
    int nRet = 1; 
    MathEngine::CartesianVector currentVector;
    bool debug = false;
    do
    {
        currentVector = MotionEngine::GetCurrentCartesianVectorOfFootLocal(*(m_pLegPtrs[idxLeg]));
        if (isnan(currentVector.x) ||
            isnan(currentVector.y) ||
            isnan(currentVector.z))
        {
            //debug = true;
        }
    } while (debug);
    
    auto currentTimePoint = std::chrono::high_resolution_clock::now();
    //calculate traversal function --> just, if new targetVector is set

    if (idxLeg < 6 && (targetVectorLocal != m_currentTraversalTargetVector[idxLeg]))
    {
        //calcuate deltas
        m_afDeltaX[idxLeg] = targetVectorLocal.x - currentVector.x;
        m_afDeltaY[idxLeg] = targetVectorLocal.y - currentVector.y;
        m_afDeltaZ[idxLeg] = targetVectorLocal.z - currentVector.z;
        
        //set startpos
        m_afStartDeltaX[idxLeg] = currentVector.x;
        m_afStartDeltaY[idxLeg] = currentVector.y;
        m_afStartDeltaZ[idxLeg] = currentVector.z;

        m_currentTraversalTargetVector[idxLeg] = targetVectorLocal;
        if (onCircle)
        {
            //calculate spline for movement through x/y-plane
            //calculate angle between current vector and targetvector
            //auto currentVectorGlobal = MotionEngine::GetCurrentCartesianVectorOfFootGlobal(*(m_pLegPtrs[idxLeg]));
            //auto targetVectorGlobal = ConvertFromLocalToGlobalVector(*m_pLegPtrs[idxLeg], targetVectorLocal);
            //double acosOperandNum = currentVectorGlobal.x * targetVectorGlobal.x + currentVectorGlobal.y * targetVectorGlobal.y;
            //double powFunction = pow(currentVectorGlobal.x, 2);
            //double powManual = currentVectorGlobal.x * currentVectorGlobal.x;
            //double opOfFirstSqrt = pow(currentVectorGlobal.x, 2) + pow(currentVectorGlobal.y, 2);
            //double opOfSecondSqrt = pow(targetVectorGlobal.x, 2) + pow(targetVectorGlobal.y, 2);
            //double acosOperandDen = sqrt(opOfFirstSqrt) * sqrt(opOfSecondSqrt);
            //double acosOperand = acosOperandNum / acosOperandDen;
            //if (acosOperand > 1.0)
            //{
            //    acosOperand = 1;
            //}
            //double alpha = acos(acosOperand);
            //double neutralPointRadius = legConst_radius + legConst_neutralPointX;
            //double lengthOfSegment = (neutralPointRadius)*alpha;
            //m_fCurrentTimeForTraversalms = lengthOfSegment / fSpeedInMPerS;
            //int nNumberOfSupPoints = 10;//TODO: change this to constant density of supPoints per length
            //std::vector<double> fSegmentsX;
            //std::vector<double> fSegmentsY;
            //std::vector<double> fSegmentsTime;
            //double fSegmentX = 0;
            //double fSegmentY = 0;
            //double angleIncrement = alpha / nNumberOfSupPoints;
            //MathEngine::CartesianVector globalSegmentVector;
            //for (int i = 1; i <= nNumberOfSupPoints; i++)
            //{
            //    double turnAngle;
            //    //if left --> apply positive angle
            //    if (onCircleLeft)
            //    {
            //        turnAngle = angleIncrement * i;
            //    }
            //    else
            //    {
            //        turnAngle = angleIncrement * -i;
            //    }
            //    fSegmentX = legConst_neutralPointX * cos(turnAngle) - legConst_neutralPointY * sin(turnAngle);//global
            //    fSegmentY = legConst_neutralPointX * sin(turnAngle) + legConst_neutralPointY * cos(turnAngle);//global
            //    globalSegmentVector.x = fSegmentX;
            //    globalSegmentVector.y = fSegmentY;
            //    auto localSegmentVector = ConvertFromGlobalToLocalVector(*m_pLegPtrs[idxLeg], globalSegmentVector);
            //    fSegmentsX.push_back(localSegmentVector.x);
            //    fSegmentsY.push_back(localSegmentVector.y);
            //    fSegmentsTime.push_back((m_fCurrentTimeForTraversalms / nNumberOfSupPoints) * i);
            //    if (nNumberOfSupPoints == i)
            //    {
            //        localSegmentVector.z = legConst_neutralPointZ; //temporary
            //        m_currentTraversalTargetVector[idxLeg] = localSegmentVector;
            //    }
            //}
            //tk::spline xSpline;
            //tk::spline ySpline;
            //xSpline.set_points(fSegmentsX, fSegmentsTime);
            //ySpline.set_points(fSegmentsY, fSegmentsTime);
            tk::spline xSpline;
            tk::spline ySpline;
            /*std::cout << "calculating circle-spline for leg " << idxLeg;
            std::cout << "vec x: " << targetVectorLocal.x << " y: " << targetVectorLocal.y << " z: " << targetVectorLocal.z << std::endl;
            std::cout << "currentVec x: " << m_currentTraversalTargetVector[idxLeg].x << " y: " << m_currentTraversalTargetVector[idxLeg].y << " z: " << m_currentTraversalTargetVector[idxLeg].z << std::endl;
            */CalculateXYSplinesForTraversal//   TODO: 
            (
                idxLeg, 
                targetVectorLocal, 
                fSpeedInMPerS, 
                onCircleLeft, 
                &xSpline, 
                &ySpline
            );
            m_splinesTurnX[idxLeg] = xSpline;
            m_splinesTurnY[idxLeg] = ySpline;
        }
        else
        {
            //calculate traversal length in mm TODO: FIx error , where traversalLength is nan(ind), because the deltas are nan(ind)
            double traversalLengthXY =
                sqrtf
                ((float)
                (
                    pow(m_afDeltaX[idxLeg], 2) +
                    pow(m_afDeltaY[idxLeg], 2) +
                    pow(m_afDeltaZ[idxLeg], 2)
                    )
                );
            m_currentTraversalTargetVector[idxLeg] = targetVectorLocal;
            m_fCurrentTimeForTraversalms = ((traversalLengthXY) / fSpeedInMPerS); //TODO: fix this
            //maybe add a traversaltime for each leg...
        }
        if (moveUp)
        {
            //calculate spline for z-traversal through air
            
            //test slowing down the last bit of travel
            /*double fTimeForFastTravel = m_fCurrentTimeForTraversalms * 0.6;
            double fZForEndOfFastTravel = targetVectorLocal.z - 15;*/
            double fZOnHalfWay = targetVectorLocal.z - 45; //TODO: add subtraction from neutral vectoror something
            double fTimeForHalfWay = /*fTimeForFastTravel*/ m_fCurrentTimeForTraversalms/ 2;
            
            std::vector<double> X = std::vector<double>{ 0, fTimeForHalfWay,/* fTimeForFastTravel,*/ m_fCurrentTimeForTraversalms };
            std::vector<double> Z = std::vector<double>{ currentVector.z, fZOnHalfWay, /*fZForEndOfFastTravel,*/ targetVectorLocal.z };
            tk::spline spline;
            spline.set_points(X, Z);//TODO: think about using "addBoundries"
            m_splinesForTravel[idxLeg] = spline;
        }
        
        m_bIsFinishedTraversing = false;
        m_traversalStartPoint = std::chrono::high_resolution_clock::now();
        currentTimePoint = std::chrono::high_resolution_clock::now();
    }
    currentTimePoint = std::chrono::high_resolution_clock::now();
    
    //calculate timedifference
    auto chronoMilliseconds 
        = std::chrono::duration_cast<std::chrono::milliseconds>
        (
            currentTimePoint - m_traversalStartPoint
            );
    double fDifferenceMs = chronoMilliseconds.count();

    bool xFinished = false;
    bool yFinished = false;
    bool zFinished = false;

    //caluclate values of the current traversal vector for current timepoint
    double fCurrentX = currentVector.x;
    if (m_afDeltaX[idxLeg] > 0 ? //check, if this is also viable, if turning
        currentVector.x + 0.05 < targetVectorLocal.x :
        currentVector.x - 0.05 > targetVectorLocal.x)
    {
        if (onCircle)
        {
            fCurrentX = m_splinesTurnX[idxLeg](fDifferenceMs);
        }
        else
        {
            fCurrentX =
                (1 / m_fCurrentTimeForTraversalms) *
                m_afDeltaX[idxLeg] *
                fDifferenceMs
                +
                m_afStartDeltaX[idxLeg];
        }
    }
    else if (onCircle && fDifferenceMs > m_fCurrentTimeForTraversalms)
    {
        xFinished = true;
        std::cout << "xFinished on cicle, leg: " << idxLeg << std::endl;
    }
    else
    {
        xFinished = true;
    }
    double fCurrentY = currentVector.y;
    if (m_afDeltaY[idxLeg] > 0 ?
        currentVector.y + 0.05 < targetVectorLocal.y :
        currentVector.y - 0.05 > targetVectorLocal.y)
    {
        if (onCircle)
        {
            fCurrentY = m_splinesTurnY[idxLeg](fDifferenceMs);
        }
        else
        {
            fCurrentY =
                (1 / m_fCurrentTimeForTraversalms) *
                m_afDeltaY[idxLeg] *
                fDifferenceMs
                +
                m_afStartDeltaY[idxLeg];
        }
    }
    else if (onCircle && fDifferenceMs > m_fCurrentTimeForTraversalms)
    {
        yFinished = true;
        std::cout << "yFinished on cicle, leg: " << idxLeg << std::endl;
    }
    else
    {
        yFinished = true;
    }

    double fCurrentZ = currentVector.z;
    if (!moveUp && (m_afDeltaZ[idxLeg] > 0 ?
        currentVector.z + 0.05 < targetVectorLocal.z :
        currentVector.z - 0.05 > targetVectorLocal.z))
    {
        //fCurrentZ = m_currentTraversalTargetVector[idxLeg].z;
        fCurrentZ =
            (1 / m_fCurrentTimeForTraversalms) *
            m_afDeltaZ[idxLeg] *
            fDifferenceMs
            +
            m_afStartDeltaZ[idxLeg];
    }
    else if (moveUp && fDifferenceMs < m_fCurrentTimeForTraversalms)
    {
        fCurrentZ = m_splinesForTravel[idxLeg](fDifferenceMs);
    }
    else if (onCircle && fDifferenceMs > m_fCurrentTimeForTraversalms)
    {
        zFinished = true;
        std::cout << "zFinished on cicle, leg: " << idxLeg << std::endl;
    }
    else
    {
        zFinished = true;
    }

    //check, if all coordinates are finished
    if (xFinished && yFinished && zFinished || m_bIsFinishedTraversing)//TODO: redo this in MathEngine::CartesianVector
    {
        if (onCircle)
        {
            std::cout << "finished traversal on circle, idxLeg: " << idxLeg << std::endl;
        }
        m_bIsFinishedTraversing = true;
        nRet = 0;
    }
    else
    {
        //if (m_fCurrentSpeed > 0.0001 && fCurrentZ >= m_targetVectors[idxLeg].z && (*m_buttons)[idxLeg] == 0)
        //{
        //    //std::cout << "Houston, we have got a problem..." << std::endl;
        //    m_bDirectionBlocked = true;
        //    m_fBlockedDirectionAngle = m_fCurrentDirectionAngle;
        //    std::stringstream msg;
        //    msg << "btn on leg " << idxLeg << " not pressed, blocking angle " << m_fBlockedDirectionAngle << std::endl;
        //    std::cout << msg.str().c_str() << std::endl;
        //    m_errors->push_back(msg.str().c_str());
        //}
        //move the leg
        MathEngine::CartesianVector currentTargetVector =
            MathEngine::CartesianVector
            (
                fCurrentX,
                fCurrentY,
                fCurrentZ
            );
        {
            try
            {
                //PerformanceTimer("MoveLegWithLocalVector");
                bool result = MotionEngine::MoveLegWithLocalVector
                (
                    hComm,
                    m_pLegPtrs[idxLeg],
                    currentTargetVector
                );
            }
            catch (std::exception & e)
            {
                if (e.what() == "nan in kinematics")
                {
                    nRet = -1;
                }
            }
            //if (!result)
            //{
            //    bool bDebug = false;
            //    if (bDebug)
            //    {
            //        bool result = MotionEngine::MoveLegWithLocalVector
            //        (
            //            hComm,
            //            m_pLegPtrs[idxLeg],
            //            currentTargetVector
            //        );
            //    }
            //    nRet = 0;
            //    m_bIsFinishedTraversing = true; // not sure about that..
            //}
        }
    }
    return nRet;
}

int MotionEngine::StandUp(int hComm)
{
    MathEngine::CartesianVector neutralPointLocal =
        MathEngine::CartesianVector
        (
            legConst_neutralPointX,
            legConst_neutralPointY,
            legConst_neutralPointZ
        );
    int nResult = 0;
    //do
    //{
        for (int idxLeg = 0; idxLeg < 6; idxLeg++)
        {
            nResult = TraverseTo
            (
                hComm,
                idxLeg,
                neutralPointLocal,
                false,
                0.1
            );
        }
   // } while (nResult != 0);
    return nResult;
}

int MotionEngine::LayDown(int hComm)
{
    MathEngine::CartesianVector startPosLocal =
        MathEngine::CartesianVector
        (
            legConst_neutralPointX + 60,
            legConst_neutralPointY,
            legConst_neutralPointZ - 110
        );
    int nResult = 0;

    for (int idxLeg = 0; idxLeg < 6; idxLeg++)
    {
        nResult = TraverseTo
        (
            hComm,
            idxLeg,
            startPosLocal,
            false,
            0.1
        );
    }
    return nResult;
}

std::string GenerateCommandHelper(int channel, int pulse)
{
    std::string cmd;
    cmd = "#";
    cmd.append(std::to_string(channel));
    cmd.append("P");
    cmd.append(std::to_string(pulse));
    cmd += (char)13;
    return cmd;
}

int MotionEngine::KeepLegsAlive(int hComm, bool addToPulse)
{
    std::string cmd;
    unsigned int currentPulse;
    unsigned int pulseToAdd = 70;
    int channel;
    for (int idxLeg = 0; idxLeg < 6; idxLeg++)
    {
        //currentPulse = m_pLegPtrs[idxLeg]->GetHipJointServo()->GetCurrentPulse();
        //channel = m_pLegPtrs[idxLeg]->GetHipJointServo()->GetChannel();
        //if (addToPulse)
        //{
        //    currentPulse += pulseToAdd;
        //}
        //cmd = GenerateCommandHelper(channel, currentPulse);
        //serialPuts(hComm, cmd.c_str());

        currentPulse = m_pLegPtrs[idxLeg]->GetUpperLegJointServo()->GetCurrentPulse();
        channel = m_pLegPtrs[idxLeg]->GetUpperLegJointServo()->GetChannel();
        if (addToPulse)
        {
            currentPulse += pulseToAdd;
        }
        cmd = GenerateCommandHelper(channel, currentPulse);
        serialPuts(hComm, cmd.c_str());

        /*currentPulse = m_pLegPtrs[idxLeg]->GetLowerLegJointServo()->GetCurrentPulse();
        channel = m_pLegPtrs[idxLeg]->GetLowerLegJointServo()->GetChannel();
        if (addToPulse)
        {
            currentPulse += pulseToAdd;
        }
        cmd = GenerateCommandHelper(channel, currentPulse);
        serialPuts(hComm, cmd.c_str());*/
    }
    return 0;
}



int MotionEngine::CalculateXYSplinesForTraversal
(
    int idxLeg, 
    MathEngine::CartesianVector targetVectorLocal, 
    double fSpeedInMPerS, 
    bool onCircleLeft, 
    tk::spline* pSplineX, 
    tk::spline* pSplineY
)
{
    //TODO: fix issue with skipping to neutral pos...
    if (nullptr != pSplineX && nullptr != pSplineY)
    {
        //calculate spline for movement through x/y-plane
            //calculate angle between current vector and targetvector
        auto currentVectorGlobal = MotionEngine::GetCurrentCartesianVectorOfFootGlobal(*(m_pLegPtrs[idxLeg]));
        auto targetVectorGlobal = ConvertFromLocalToGlobalVector(*m_pLegPtrs[idxLeg], targetVectorLocal);
        auto globalNeutralVector = ConvertFromLocalToGlobalVector(*m_pLegPtrs[idxLeg], MathEngine::CartesianVector(legConst_neutralPointX, legConst_neutralPointY, legConst_neutralPointZ));
        double acosOperandNum = currentVectorGlobal.x * targetVectorGlobal.x + currentVectorGlobal.y * targetVectorGlobal.y;
        double powFunction = pow(currentVectorGlobal.x, 2);
        double powManual = currentVectorGlobal.x * currentVectorGlobal.x;

        double opOfFirstSqrt = pow(currentVectorGlobal.x, 2) + pow(currentVectorGlobal.y, 2);
        double opOfSecondSqrt = pow(targetVectorGlobal.x, 2) + pow(targetVectorGlobal.y, 2);
        double acosOperandDen = sqrt(opOfFirstSqrt) * sqrt(opOfSecondSqrt);
        double acosOperand = acosOperandNum / acosOperandDen;
        if (acosOperand > 1.0)
        {
            acosOperand = 1;
        }
        double alpha = acos(acosOperand);

        double neutralPointRadius = legConst_radius + legConst_neutralPointX;
        double lengthOfSegment = (neutralPointRadius)*alpha;
        m_fCurrentTimeForTraversalms = lengthOfSegment / fSpeedInMPerS;

        const int nNumberOfSupPoints = 20;//TODO: change this to constant density of supPoints per length
        std::vector<double> fSegmentsX;
        double segmentsXArr[nNumberOfSupPoints];
        std::vector<double> fSegmentsY;
        double segmentsYArr[nNumberOfSupPoints];
        std::vector<double> fSegmentsTime;
        double segmentsTimeArr[nNumberOfSupPoints];

        double fSegmentX = 0;
        double fSegmentY = 0;
        double angleIncrement = alpha / nNumberOfSupPoints;
        MathEngine::CartesianVector globalSegmentVector;
        for (int i = 1; i <= nNumberOfSupPoints; i++)
        {
            double turnAngle;
            //if left --> apply positive angle
            if (onCircleLeft)
            {
                turnAngle = angleIncrement * i;
            }
            else
            {
                turnAngle = angleIncrement * -i;
            }
            //changed from neutral to  current
            fSegmentX = currentVectorGlobal.x * cos(turnAngle) - currentVectorGlobal.y * sin(turnAngle);//global, error, legConst_neutralPoint is NOT global
            fSegmentY = currentVectorGlobal.x * sin(turnAngle) + currentVectorGlobal.y * cos(turnAngle);//global

            globalSegmentVector.x = fSegmentX;
            globalSegmentVector.y = fSegmentY;

            auto localSegmentVector = ConvertFromGlobalToLocalVector(*m_pLegPtrs[idxLeg], globalSegmentVector);

            fSegmentsX.push_back(localSegmentVector.x);
            fSegmentsY.push_back(localSegmentVector.y);

            fSegmentsTime.push_back((m_fCurrentTimeForTraversalms / nNumberOfSupPoints) * i);

            segmentsXArr[i-1] = localSegmentVector.x;
            segmentsYArr[i-1] = localSegmentVector.y;
            segmentsTimeArr[i-1] = fSegmentsTime[i-1];
            if (nNumberOfSupPoints == i)
            {
                localSegmentVector.z = legConst_neutralPointZ; //temporary
                //m_currentTraversalTargetVector[idxLeg] = localSegmentVector;
            }
        }
        tk::spline xSpline;
        tk::spline ySpline;
        xSpline.set_points(fSegmentsTime, fSegmentsX);
        ySpline.set_points(fSegmentsTime, fSegmentsY);
        if (nullptr != pSplineX)
        {
            *pSplineX = xSpline;
        }
        if (nullptr != pSplineY)
        {
            *pSplineY = ySpline;
        }
    }
    return 0;
}



int MotionEngine::StopTraversal()
{
    /*std::vector<double> x = { 0 };
    std::vector<double> y = { 0 };*/
    for (int idxLeg = 0; idxLeg < 6; idxLeg++)
    {
        m_afDeltaX[idxLeg] = 0;
        m_afDeltaY[idxLeg] = 0;
        m_afDeltaZ[idxLeg] = 0;
        //TODO: test
        /*m_currentTraversalTargetVector[idxLeg].x = 0;
        m_currentTraversalTargetVector[idxLeg].y = 0;
        m_currentTraversalTargetVector[idxLeg].z = 0;*/
        /*m_splinesTurnX[idxLeg].set_points(x, y);
        m_splinesTurnY[idxLeg].set_points(x, y);*/
        
    }
    
    m_bIsFinishedTraversing = true;
    m_homingState = MotionEngine::HomingState::isNotAtHome;
    //m_fCurrentSpeed = 0;
    return 0;
}

int MotionEngine::TestTurningOfVectors()
{
    tk::spline xSpline;
    tk::spline YSpline;
    MathEngine::CartesianVector turnLeft;
    CalculateLocalTurnVectors(5, &turnLeft);
    CalculateXYSplinesForTraversal(0, turnLeft, 0.2, true, &xSpline, &YSpline);
    return 0;
}
