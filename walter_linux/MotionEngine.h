#pragma once
//#include <Windows.h>
#include <vector>
#include "Servo.h"
#include "Leg.h"
#include "MathEngine.h"
#include "spline.h"
#include <chrono>

class MotionEngine
{
public:
    int CalculateXYSplinesForTraversal
    (
        int idxLeg,
        MathEngine::CartesianVector targetVectorLocal,
        double fSpeedInMPerS,
        bool onCircleLeft,
        tk::spline* pSplineX = nullptr,
        tk::spline* pSplineY = nullptr
    );

    int CalculateLocalTurnVectors
    (
        double turnAngleDegree,
        MathEngine::CartesianVector* pTurnLeft = nullptr,
        MathEngine::CartesianVector* pTurnRight = nullptr
    );

private:
    enum RobotState
    {
        init,
        idle,
        walking,
        standingUp,
        safetyState,
        layingDown
    };

    enum HomingState
    {
        stopTraversal,
        moveFirstGroupToHome,
        moveSecondGroupToHome,
        isAtHome,
        isNotAtHome
    };

    //internal state:
    Leg* m_pLegPtrs[6]; //TODO: make sure, that the MotionEngine is destroyed before the legPtrs
    bool m_bShiftGroupIsOdds;
    bool m_bShiftGroupsSwitched;
    HomingState m_homingState;
    HomingState m_lastHomingState;
    bool m_bMove;
    RobotState m_currentState;
    
    //basically these are the tarversal-vectors for all legs
    double m_afDeltaX[6];
    double m_afDeltaY[6];
    double m_afDeltaZ[6];

    //these are used for the linear function, which calculate x & y traversal
    double m_afStartDeltaX[6];
    double m_afStartDeltaY[6];
    double m_afStartDeltaZ[6];
    double m_afMiddleZ[6];

    tk::spline m_splinesForTravel[6];
    tk::spline m_splinesTurnX[6];
    tk::spline m_splinesTurnY[6];

    MathEngine::CartesianVector m_targetVectorTravelLocal;
    MathEngine::CartesianVector m_targetVectorShiftLocal;
    MathEngine::CartesianVector m_targetVectors[6];

    //last direction vector
    int m_nLastDirectionX;
    int m_nLastDirectionY;

    bool m_bIsWalking;
    bool m_bIsFinishedWalking; 

    //for single leg traversal
    MathEngine::CartesianVector m_currentTraversalTargetVector[6];

    double m_fCurrentTimeForTraversalms;
    std::chrono::high_resolution_clock::time_point m_traversalStartPoint;
    bool m_bIsFinishedTraversing;
    //double m_fCurrentDirectionAngle;
    //bool m_bDirectionBlocked;
    //double m_fBlockedDirectionAngle;
    //const double m_blockingAngleRange = M_PI_2;
    //double m_fCurrentSpeed = 0;

    std::vector<int>* m_buttons;
    std::vector<std::vector<int>>* m_legAnglesRawAnalogue;

    std::vector<std::string>* m_notifys;
    std::vector<std::string>* m_errors;

    //bool AngleIsInBlockingRange(double fAngleToCheck, double fAngleWithRange);
public:
    MathEngine::CartesianVector m_localTurnLeft;
    MathEngine::CartesianVector m_localTurnRight;

    struct LegAngles
    {
        double hipJoint;
        double upperLegJoint;
        double lowerLegJoint;
        LegAngles(double hip, double upperLeg, double lowerLeg)
        {
            hipJoint = hip;
            upperLegJoint = upperLeg;
            lowerLegJoint = lowerLeg;
        }
        LegAngles()
        {
            hipJoint = 0;
            upperLegJoint = 0;
            lowerLegJoint = 0;
        }
    };

    MotionEngine
    (
        std::vector<Leg*> legPtrs, 
        std::vector<int>* buttons, 
        std::vector<std::vector<int>>* legAnglesRaw,
        std::vector<std::string>* notifys,
        std::vector<std::string>* error
    );

    ~MotionEngine();

	static bool MoveServoToPulseWidth
	(
		int hComm, 
		Servo* pServo, 
		int nPulse, 
		unsigned int nTime = 0,
		bool bOutputMessageToConsole = true,
		bool bOverrideBoundaries = false
	);
	static void MoveGroup
	(
		int hComm,
		std::vector<Servo*> servos,
		std::vector<unsigned int> positions,
		unsigned int nTime,
        bool bPrintToConsole = true
	);

    static void MoveGroupWithAngles
    (
        int hComm,
        std::vector<Servo*> servos,
        std::vector<double> positionsAsDegree,
        unsigned int nTime,
        bool bPrintToConsole = true,
        std::string* pCmd = nullptr
    );

    static MathEngine::CartesianVector ConvertFromGlobalToLocalVector(Leg leg, MathEngine::CartesianVector globalVector);

    static MathEngine::CartesianVector ConvertFromLocalToGlobalVector(Leg leg, MathEngine::CartesianVector localVector);

    static MathEngine::CartesianVector GetCurrentCartesianVectorOfFootGlobal //forward kinematics
    (
        Leg leg, 
        MathEngine::CartesianVector* hipJointRelativeVector = nullptr,
        MathEngine::CartesianVector* localFootVector = nullptr
    );
    
    static MathEngine::CartesianVector GetCurrentCartesianVectorOfFootLocal //forward kinematics
    (
        Leg leg,
        double* lengthOfFootVectorxy = nullptr
    );

    static LegAngles CalculateLegAnglesWithLocalVector //inverse kinematics
    (
        Leg leg, 
        MathEngine::CartesianVector targetVectorLocal,
        int* errorRet = nullptr
    );

    static LegAngles CalculateLegAnglesWithGlobalVector //inverse kinematics
    (
        Leg leg,
        MathEngine::CartesianVector targetVectorGlobal,
        MathEngine::CartesianVector* targetVectorLocal = nullptr
    );

    static bool MoveLegWithGlobalVector
    (
        int hComm,
        Leg* legPtr,
        MathEngine::CartesianVector targetVectorGlobal,
        unsigned int nTime = 0
        );

    static int MoveLegsWithLocalVector
    (
        int hComm, 
        std::vector<Leg*> legPtrs, 
        MathEngine::CartesianVector targetVectorLocal, 
        unsigned int nTime
    );

    static int MoveLegWithLocalVector
    (
        int hComm,
        Leg* legPtr,
        MathEngine::CartesianVector targetVectorLocal,
        unsigned int nTime = 0
    );
    int DoWalk(int nDirectionX, int nDirectionY);
    int TestWalk(int hComm, int nDirectionX, int nDirectiony);


    int Walk(int hComm, int nDirectionX, int nDirectiony);

    //TODO: move StandUp and LayDown from main to here
    
    //int CalculateLocalTurnVectors
    //(
    //    double turnAngleDegree, 
    //    MathEngine::CartesianVector* pTurnLeft = nullptr,
    //    MathEngine::CartesianVector* pTurnRight = nullptr
    //);

    //nDirection > 0 => turn left, nDirection < 0 => turn right
    int Turn(int hComm, bool turnLeft);
    
    //for testing
    int TestTraversalOfOneLeg(int hComm, int idxLeg, double fSpeed);
    
    //returns 0, if finished
    int MoveHome
    (
        int hComm,
        double fSpeed
    );
    
    int TraverseOnCircle
    (
        int hComm,
        int idxLeg,
        double angleInDegree,
        bool moveUp,
        double fSpeed
    );

    int TraverseTo
    (
        int hComm,
        int nLegIndex, 
        MathEngine::CartesianVector targetVectorLocal, 
        bool moveUp, 
        double fSpeed,
        bool onCircle = false,
        bool turnLeft = false
    );

    int StandUp
    (
        int hComm
    );

    int LayDown
    (
        int hComm
    );

    int KeepLegsAlive(int hComm, bool addToPulse);
    //int CalculateXYSplinesForTraversal
    //(
    //    int idxLeg,
    //    MathEngine::CartesianVector targetVectorLocal,
    //    double fSpeedInMPerS,
    //    bool onCircleLeft,
    //    tk::spline* pSplineX,
    //    tk::spline* pSplineY
    //);

    int StopTraversal();

    int TestTurningOfVectors();
};