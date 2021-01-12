// walter_console.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#pragma once
#include <iostream>
#include <iomanip>
#include <sstream>
#include "defines.h"
#include <vector>
#include "SerialConnectionHandler.h"
#include "Configuration.h"
#include "ConfigurationHandler.h"
#include "MathEngine.h"
#include "MotionEngine.h"
#include "PerformanceTimer.h"
#include "Timer.h"
#include <math.h>
#include <map>
#include <chrono>
#include "LegConstants.h"
#include <unistd.h>
#include <curses.h>
#include "spline.h"
#include <future>
#include <map>

#include <cstdio>
#include <iostream>
#include <string>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>

typedef void (*MenuProcessingFunctionPointer)(void);

struct MenuOption
{
	char choice;
	char const* pSelectionText;
	MenuProcessingFunctionPointer pProcesingFunction;
};

enum RobotState
{
    eInitPort = 0,
    eMoveToStartPos = 1,
    eStandUp = 2,
    eNormalOp = 3,
    eIdle = 4,
    eLayDown = 5,
    eResting = 6,
    eHandMode = 7,
    eStopped = 8
};

enum IdleState
{
    eMoveUpAndDown = 0,
    eTravelWithRandomLeg = 1,//back and forth
    eShiftPlatform = 2
};

void ProcessConnectionRequest();
//void ProcessServoParameterizing();
void ProcessConfigurationSaving();
void ProcessConfigurationTest();
void ProcessConfigurationMenu();
void ProcessTestTraversal();
void ProcessConnectionTerminationRequest();
void ProcessMoveToHomePosition();
void ProcessMoveToNeutralPosition();
void ProcessMoveToNeutralPoint();
//void ProcessTestInverseKinematics();
void ProcessStraightWalk();
void ProcessVariableWalk();
void ProcessTurn();
void ProcessPowerTest();
void ProcessRapidLegMovement();
void ProcessTestVertTraversal();
void TestTurningOfVectors();
void MoveAllLegsToStartPos();
void StandUp();
void LayDown();
void Cycle();

static const MenuOption mainMenu[] =
{
  {'0', "normal operation", Cycle},
  {'1', "Establish connection to Walter", ProcessConnectionRequest},
  //{'2', "Move to homeposition", ProcessMoveToHomePosition},
  {'2', "Move to neutral position", ProcessMoveToNeutralPosition},
  {'3', "Move to neutral POINT", ProcessMoveToNeutralPoint},
  //{'4', "Configuration Menu", ProcessConfigurationMenu}, //TODO: Add back in 
  //{'6', "Test traversal", ProcessTestTraversal},
  {'5', "MOVE TO STARTPOS", MoveAllLegsToStartPos},
  {'6', "STAND UP", StandUp},
  {'7', "LAY DOWN", LayDown},
  //{'8', "Test straight walk", ProcessStraightWalk},
  {'8', "Test variable walk", ProcessVariableWalk},//TODO: add back in
  {'9', "Turn", ProcessTurn}
  //{'9', "Terminate connection to Walter", ProcessConnectionTerminationRequest}
  //{'8', "RAPID LEG MOVEMENT!!!!", ProcessRapidLegMovement},
};

//static const MenuOption configurationMenu[] =
//{
//  {'1', "Enter servo-parameterizing", ProcessServoParameterizing},
//  {'2', "Save configuration to file", ProcessConfigurationSaving},
//  {'3', "Test inverse kinematics", ProcessTestInverseKinematics},
//};

static const unsigned int mainMenuEntries =
sizeof(mainMenu) / sizeof(mainMenu[0]);

//static const unsigned int configurationMenuEntries =
//sizeof(configurationMenu) / sizeof(configurationMenu[0]);

#pragma region servos
std::vector<Servo> g_servos =
{
	Servo(0, Servo::ServoKind::hipJoint, 1500),
	Servo(1, Servo::ServoKind::upperlegJoint, 1500),
	Servo(2, Servo::ServoKind::lowerlegJoint, 1500),
	Servo(4, Servo::ServoKind::hipJoint, 1500),
	Servo(5, Servo::ServoKind::upperlegJoint, 1500),
	Servo(6, Servo::ServoKind::lowerlegJoint, 1500),
	Servo(8, Servo::ServoKind::hipJoint, 1500),
	Servo(9, Servo::ServoKind::upperlegJoint, 1500),
	Servo(10, Servo::ServoKind::lowerlegJoint, 1500),
	Servo(16, Servo::ServoKind::hipJoint, 1500),
	Servo(17, Servo::ServoKind::upperlegJoint, 1500),
	Servo(18, Servo::ServoKind::lowerlegJoint, 1500),
	Servo(20, Servo::ServoKind::hipJoint, 1500),
	Servo(21, Servo::ServoKind::upperlegJoint, 1500),
	Servo(22, Servo::ServoKind::lowerlegJoint, 1500),
	Servo(24, Servo::ServoKind::hipJoint, 1500),
	Servo(25, Servo::ServoKind::upperlegJoint, 1500),
	Servo(26, Servo::ServoKind::lowerlegJoint, 1500)
};

std::vector<Leg> g_legs
{
    Leg(&g_servos[0], &g_servos[1], &g_servos[2], 0),
    Leg(&g_servos[3], &g_servos[4], &g_servos[5], 1),
    Leg(&g_servos[6], &g_servos[7], &g_servos[8], 2),
    Leg(&g_servos[9], &g_servos[10], &g_servos[11], 3),
    Leg(&g_servos[12], &g_servos[13], &g_servos[14], 4),
    Leg(&g_servos[15], &g_servos[16], &g_servos[17], 5),
};

std::vector<Servo*> g_servoPtrs;
std::map<int, Servo*> g_servoPtrMap;
std::vector<Leg*> g_legPtrs;

Configuration g_currentConfiguration;
std::string g_configFilePath = "/home/pi/walter.config";

bool g_unlocked = false;
#pragma endregion

//MQTT-sample
// async_subscribe.cpp
//
// This is a Paho MQTT C++ client, sample application.
//
// This application is an MQTT subscriber using the C++ asynchronous client
// interface, employing callbacks to receive messages and status updates.
//
// The sample demonstrates:
//  - Connecting to an MQTT server/broker.
//  - Subscribing to a topic
//  - Receiving messages through the callback API
//  - Receiving network disconnect updates and attempting manual reconnects.
//  - Using a "clean session" and manually re-subscribing to topics on
//    reconnect.
//

/*******************************************************************************
 * Copyright (c) 2013-2017 Frank Pagliughi <fpagliughi@mindspring.com>
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Frank Pagliughi - initial implementation and documentation
 *******************************************************************************/

#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include "mqtt/async_client.h"

const std::string SERVER_ADDRESS("tcp://localhost:1883");
const std::string CLIENT_ID("async_subcribe_cpp");
const std::string TOPIC("hello");

const int	QOS = 0;
const int	N_RETRY_ATTEMPTS = 5;

/////////////////////////////////////////////////////////////////////////////

// Callbacks for the success or failures of requested actions.
// This could be used to initiate further action, but here we just log the
// results to the console.

class action_listener : public virtual mqtt::iaction_listener
{
    std::string name_;

    void on_failure(const mqtt::token& tok) override 
    {
        std::cout << name_ << " failure";
        if (tok.get_message_id() != 0)
            std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
        std::cout << std::endl;
    }

    void on_success(const mqtt::token& tok) override 
    {
        std::cout << name_ << " success";
        if (tok.get_message_id() != 0)
            std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
        auto top = tok.get_topics();
        if (top && !top->empty())
            std::cout << "\ttoken topic: '" << (*top)[0] << "', ..." << std::endl;
        std::cout << std::endl;
    }

public:
    action_listener(const std::string& name) : name_(name) {}
};

/////////////////////////////////////////////////////////////////////////////

/**
 * Local callback & listener class for use with the client connection.
 * This is primarily intended to receive messages, but it will also monitor
 * the connection to the broker. If the connection is lost, it will attempt
 * to restore the connection and re-subscribe to the topic.
 */
class callback : public virtual mqtt::callback,
    public virtual mqtt::iaction_listener

{
    // Counter for the number of connection retries
    int nretry_;
    // The MQTT client
    mqtt::async_client& cli_;
    // Options to use if we need to reconnect
    mqtt::connect_options& connOpts_;
    // An action listener to display the result of actions.
    action_listener subListener_;

    std::map<std::string, int*> mapOfTopics_;

    // This deomonstrates manually reconnecting to the broker by calling
    // connect() again. This is a possibility for an application that keeps
    // a copy of it's original connect_options, or if the app wants to
    // reconnect with different options.
    // Another way this can be done manually, if using the same options, is
    // to just call the async_client::reconnect() method.
    void reconnect() 
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        try 
        {
            cli_.connect(connOpts_, nullptr, *this);
        }
        catch (const mqtt::exception & exc) {
            std::cerr << "Error: " << exc.what() << std::endl;
            exit(1);
        }
    }

    // Re-connection failure
    void on_failure(const mqtt::token& tok) override 
    {
        std::cout << "Connection attempt failed" << std::endl;
        if (++nretry_ > N_RETRY_ATTEMPTS)
            exit(1);
        reconnect();
    }

    // (Re)connection success
    // Either this or connected() can be used for callbacks.
    void on_success(const mqtt::token& tok) override {}

    // (Re)connection success
    void connected(const std::string& cause) override 
    {
        std::cout << "\nConnection success" << std::endl;
        std::cout << "\nSubscribing to topic '" << TOPIC << "'\n"
            //<< "\tfor client " << CLIENT_ID
            << " using QoS" << QOS << "\n"
            << "\nPress Q<Enter> to quit\n" << std::endl;

        //TODO: add logic for subscribing to multiple topics --> test this
        for (auto iter : mapOfTopics_)
        {
            std::string key = iter.first;
            cli_.subscribe(key, QOS, nullptr, subListener_);
        }

        cli_.subscribe("WalterNotify", 0, nullptr, subListener_);
        //cli_.subscribe(TOPIC, QOS, nullptr, subListener_);
    }

    // Callback for when the connection is lost.
    // This will initiate the attempt to manually reconnect.
    void connection_lost(const std::string& cause) override 
    {
        std::cout << "\nConnection lost" << std::endl;
        if (!cause.empty())
            std::cout << "\tcause: " << cause << std::endl;

        std::cout << "Reconnecting..." << std::endl;
        nretry_ = 0;
        reconnect();
    }

    // Callback for when a message arrives.
    void message_arrived(mqtt::const_message_ptr msg) override 
    {
        std::string topic = msg->get_topic();
        std::string payLoad = msg->to_string();
        /*std::cout << "Message arrived" << std::endl;
        std::cout << "\ttopic: '" << topic << "'" << std::endl;
        std::cout << "\tpayload: '" << payLoad << "'\n" << std::endl;*/

        if (topic.substr(0, 10) == "WalterCtrl")
        {
            std::cout << "Message arrived" << std::endl;
            std::cout << "\ttopic: '" << topic << "'" << std::endl;
            std::cout << "\tpayload: '" << payLoad << "'\n" << std::endl;
        }
        if (topic == "WalterNotify" && payLoad == "Web-Controller verbunden.")
        {
            std::cout << "WalterNotify: web controller verbunden" << std::endl;
            g_unlocked = true;
        }
        if (topic == "WalterStop")
        {
            std::cout << "WalterStop: " << payLoad << std::endl;
        }
        //TODO: test this
        try
        {
            int* buffer = mapOfTopics_.at(topic);
            if (nullptr != buffer)
            {
                *buffer = std::stoi(payLoad);
            }
        }
        catch (std::out_of_range ex)
        {

        }
    }

    void delivery_complete(mqtt::delivery_token_ptr token) override {}

public:
    callback(mqtt::async_client& cli, mqtt::connect_options& connOpts, std::map<std::string, int*> mapOfTopics)
        : nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription") 
    {
        mapOfTopics_ = mapOfTopics;
    }
};

/////////////////////////////////////////////////////////////////////////////

//int main(int argc, char* argv[])
//{
//    mqtt::connect_options connOpts;
//    connOpts.set_keep_alive_interval(20);
//    connOpts.set_clean_session(true);
//
//    mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
//
//    callback cb(client, connOpts);
//    client.set_callback(cb);
//
//    // Start the connection.
//    // When completed, the callback will subscribe to topic.
//
//    try {
//        std::cout << "Connecting to the MQTT server..." << std::flush;
//        client.connect(connOpts, nullptr, cb);
//    }
//    catch (const mqtt::exception&) {
//        std::cerr << "\nERROR: Unable to connect to MQTT server: '"
//            << SERVER_ADDRESS << "'" << std::endl;
//        return 1;
//    }
//
//    // Just block till user tells us to quit.
//
//    while (std::tolower(std::cin.get()) != 'q')
//        ;
//
//    // Disconnect
//
//    try {
//        std::cout << "\nDisconnecting from the MQTT server..." << std::flush;
//        client.disconnect()->wait();
//        std::cout << "OK" << std::endl;
//    }
//    catch (const mqtt::exception & exc) {
//        std::cerr << exc.what() << std::endl;
//        return 1;
//    }
//
//    return 0;
//}

//TODO: make this work under Linux
//void ManualServoParamterization
//(
//	int hComm,
//	Servo* servoPtr
//)
//{
//	//TODO: add option to save current position as min, max, neutral, startpos
//	if (hComm == -1)
//	{
//		std::cout << "hComm was invalid, just printing to console..." << std::endl;
//	}
//	
//	std::cout << "F1: save as new min-position" << std::endl;
//	std::cout << "F2: save as new neutral-position" << std::endl;
//	std::cout << "F3: save as new max-position" << std::endl;
//	std::cout << "F4: save as new start-position" << std::endl;
//
//	unsigned long nScale = 800;
//    unsigned long nSlowModulo = 40;
//	unsigned long nPulse = servoPtr->GetCurrentPulse();;
//    unsigned long nPreviousPulse = nPulse;
//    unsigned long nCountFast = nPulse * nScale;
//    unsigned long nCountSlow = nPulse * nScale * nSlowModulo;
//    unsigned long nMaxPulse = servoPtr->GetMaxPulse();
//    unsigned long nMinPulse = servoPtr->GetMinPulse();
//
//	bool bSavingKeyWasPressed = false;
//	while (1)
//	{
//		if (nPulse > 10 && nPulse < 3000)
//		{
//			if (GetAsyncKeyState(KEY_UP) < 0 /*&& nPulse < nMaxPulse*/)
//			{
//				nCountFast++;
//				nCountSlow = nCountFast * nSlowModulo;
//				nPulse = nCountFast / nScale;
//			}
//			else if (GetAsyncKeyState(VK_DOWN) < 0 /*&& nPulse > nMinPulse*/)
//			{
//				nCountFast--;
//				nCountSlow = nCountFast * nSlowModulo;
//				nPulse = nCountFast / nScale;
//			}
//			else if (GetAsyncKeyState(VK_RIGHT) < 0 /*&& nPulse < nMaxPulse*/)
//			{
//				nCountSlow++;
//				if (!(nCountSlow % nSlowModulo))
//				{
//					nCountFast++;
//				}
//				nPulse = nCountFast / nScale;
//			}
//			else if (GetAsyncKeyState(VK_LEFT) < 0 /*&& nPulse > nMinPulse*/)
//			{
//				nCountSlow--;
//				if (!(nCountSlow % nSlowModulo))
//				{
//					nCountFast--;
//				}
//				nPulse = nCountFast / nScale;
//			}
//			else if (GetAsyncKeyState(VK_F1) < 0) // new min pos
//			{
//				if (!bSavingKeyWasPressed)
//				{
//					ConfigurationHandler::SetMinPosition(servoPtr, nPulse);
//					std::cout << "Set min pulse of servo " << servoPtr->GetChannel();
//					std::cout << " to " << nPulse << std::endl;
//					bSavingKeyWasPressed = true;
//				}
//			}
//			else if (GetAsyncKeyState(VK_F2) < 0) // new neutral pos
//			{
//				if (!bSavingKeyWasPressed)
//				{
//					ConfigurationHandler::SetNeutralPosition(servoPtr, nPulse);
//					std::cout << "Set neutral pulse of servo " << servoPtr->GetChannel();
//					std::cout << " to " << nPulse << std::endl;
//					bSavingKeyWasPressed = true;
//				}
//			}
//			else if (GetAsyncKeyState(VK_F3) < 0) // new max pos
//			{
//				if (!bSavingKeyWasPressed)
//				{
//					ConfigurationHandler::SetMaxPosition(servoPtr, nPulse);
//					std::cout << "Set max pulse of servo " << servoPtr->GetChannel();
//					std::cout << " to " << nPulse << std::endl;
//					bSavingKeyWasPressed = true;
//				}
//			}
//			else if (GetAsyncKeyState(VK_F4) < 0) // new start pos
//			{
//				if (!bSavingKeyWasPressed)
//				{
//					ConfigurationHandler::SetStartPosition(servoPtr, nPulse);
//					std::cout << "Set start pulse of servo " << servoPtr->GetChannel();
//					std::cout << " to " << nPulse << std::endl;
//					bSavingKeyWasPressed = true;
//				}
//			}
//			else if (GetAsyncKeyState(VK_ESCAPE) < 0)
//			{
//				break;
//			}
//			if (nPulse != nPreviousPulse)
//			{
//				MotionEngine::MoveServoToPulseWidth
//				(
//					hComm, 
//					servoPtr, 
//					nPulse, 
//					0, 
//					false, 
//					true
//				);
//                std::cout << "Pulse = " << nPulse;
//                std::cout << " Angle = " << servoPtr->GetCurrentAngle() << "\r";
//			}
//			nPreviousPulse = nPulse;
//			if (GetAsyncKeyState(VK_F1) >= 0 &&
//				GetAsyncKeyState(VK_F2) >= 0 &&
//				GetAsyncKeyState(VK_F3) >= 0 &&
//				GetAsyncKeyState(VK_F4) >= 0)
//			{
//				bSavingKeyWasPressed = false;
//			}
//		}
//	}
//	currentConfiguration = ConfigurationHandler::GetConfigurationFromServoVector(servos);
//}

//TODO: Make this work under linux --> replace GetAsyncKeyState
//void ManualServoParamterizationLeg
//(
//    int hComm,
//    Leg* legPtr,
//    Servo::ServoKind servoKind
//)
//{
//    if (hComm == -1)
//    {
//        std::cout << "hComm was invalid, just printing to console..." << std::endl;
//    }
//
//    std::cout << "F1: save as new min-position (for Pulse, NOT Angle!)" << std::endl;
//    std::cout << "F2: save as new neutral-position" << std::endl;
//    std::cout << "F3: save as new max-position (for Pulse, NOT Angle!" << std::endl;
//    std::cout << "F4: save as new start-position" << std::endl;
//
//    Servo* servoPtr;
//    if (Servo::ServoKind::hipJoint == servoKind)
//    {
//        servoPtr = legPtr->GetHipJointServo();
//    }
//    else if (Servo::ServoKind::upperlegJoint == servoKind)
//    {
//        servoPtr = legPtr->GetUpperLegJointServo();
//    }
//    else if (Servo::ServoKind::lowerlegJoint == servoKind)
//    {
//        servoPtr = legPtr->GetLowerLegJointServo();
//    }
//    else
//    {
//        std::cout << "ServoKind was uninitialized, breaking out of method..." << std::endl;
//        return;
//    }
//
//    unsigned long nScale = 800;
//    unsigned long nSlowModulo = 40;
//    unsigned long nPulse = servoPtr->GetCurrentPulse();;
//    unsigned long nPreviousPulse = nPulse;
//    unsigned long nCountFast = nPulse * nScale;
//    unsigned long nCountSlow = nPulse * nScale * nSlowModulo;
//    unsigned long nMaxPulse = servoPtr->GetMaxPulse();
//    unsigned long nMinPulse = servoPtr->GetMinPulse();
//
//    bool bSavingKeyWasPressed = false;
//    while (1)
//    {
//        if (nPulse > 10 && nPulse < 3000)
//        {
//            if (GetAsyncKeyState(VK_UP) < 0 /*&& nPulse < nMaxPulse*/)
//            {
//                nCountFast++;
//                nCountSlow = nCountFast * nSlowModulo;
//                nPulse = nCountFast / nScale;
//            }
//            else if (GetAsyncKeyState(VK_DOWN) < 0 /*&& nPulse > nMinPulse*/)
//            {
//                nCountFast--;
//                nCountSlow = nCountFast * nSlowModulo;
//                nPulse = nCountFast / nScale;
//            }
//            else if (GetAsyncKeyState(VK_RIGHT) < 0 /*&& nPulse < nMaxPulse*/)
//            {
//                nCountSlow++;
//                if (!(nCountSlow % nSlowModulo))
//                {
//                    nCountFast++;
//                }
//                nPulse = nCountFast / nScale;
//            }
//            else if (GetAsyncKeyState(VK_LEFT) < 0 /*&& nPulse > nMinPulse*/)
//            {
//                nCountSlow--;
//                if (!(nCountSlow % nSlowModulo))
//                {
//                    nCountFast--;
//                }
//                nPulse = nCountFast / nScale;
//            }
//            else if (GetAsyncKeyState(VK_F1) < 0) // new min pos
//            {
//                if (!bSavingKeyWasPressed)
//                {
//                    ConfigurationHandler::SetMinPosition(servoPtr, nPulse);
//                    std::cout << "Set min pulse of servo " << servoPtr->GetChannel();
//                    std::cout << " to " << nPulse << std::endl;
//                    bSavingKeyWasPressed = true;
//                }
//            }
//            else if (GetAsyncKeyState(VK_F2) < 0) // new neutral pos
//            {
//                if (!bSavingKeyWasPressed)
//                {
//                    ConfigurationHandler::SetNeutralPosition(servoPtr, nPulse);
//                    std::cout << "Set neutral pulse of servo " << servoPtr->GetChannel();
//                    std::cout << " to " << nPulse << std::endl;
//                    bSavingKeyWasPressed = true;
//                }
//            }
//            else if (GetAsyncKeyState(VK_F3) < 0) // new max pos
//            {
//                if (!bSavingKeyWasPressed)
//                {
//                    ConfigurationHandler::SetMaxPosition(servoPtr, nPulse);
//                    std::cout << "Set max pulse of servo " << servoPtr->GetChannel();
//                    std::cout << " to " << nPulse << std::endl;
//                    bSavingKeyWasPressed = true;
//                }
//            }
//            else if (GetAsyncKeyState(VK_F4) < 0) // new start pos
//            {
//                if (!bSavingKeyWasPressed)
//                {
//                    ConfigurationHandler::SetStartPosition(servoPtr, nPulse);
//                    std::cout << "Set start pulse of servo " << servoPtr->GetChannel();
//                    std::cout << " to " << nPulse << std::endl;
//                    bSavingKeyWasPressed = true;
//                }
//            }
//            else if (GetAsyncKeyState(VK_ESCAPE) < 0)
//            {
//                break;
//            }
//            if (nPulse != nPreviousPulse)
//            {
//                MotionEngine::MoveServoToPulseWidth
//                (
//                    hComm,
//                    servoPtr,
//                    nPulse,
//                    0,
//                    false,
//                    true
//                );
//                MathEngine::CartesianVector footVector = MotionEngine::GetCurrentCartesianVectorOfFootGlobal(*legPtr, nullptr);
//                std::cout << "Pulse = " << nPulse;
//                std::cout << " Angle = " << servoPtr->GetCurrentAngle();
//                std::cout << " FootCoord: x=" << footVector.x;
//                std::cout << " y=" << footVector.y;
//                std::cout << " z=" << footVector.z << "\r";
//            }
//            nPreviousPulse = nPulse;
//            if (GetAsyncKeyState(VK_F1) >= 0 &&
//                GetAsyncKeyState(VK_F2) >= 0 &&
//                GetAsyncKeyState(VK_F3) >= 0 &&
//                GetAsyncKeyState(VK_F4) >= 0)
//            {
//                bSavingKeyWasPressed = false;
//            }
//        }
//    }
//    currentConfiguration = ConfigurationHandler::GetConfigurationFromServoVector(servos);
//}

//void ManualLegMoving
//(
//    int hComm,
//    Leg* legPtr
//)
//{
//    if (hComm == -1)
//    {
//        std::cout << "hComm was invalid, just printing to console..." << std::endl;
//    }
//
//    std::cout << "F1: increase x" << std::endl;
//    std::cout << "F2: decrease x" << std::endl;
//    std::cout << "F3: increase y" << std::endl;
//    std::cout << "F4: decrease y" << std::endl;
//    std::cout << "F6: increase z" << std::endl;
//    std::cout << "F7: decrease z" << std::endl;
//    std::cout << "F8: increase override" << std::endl;
//    std::cout << "F9: decrease override" << std::endl;
//
//    MathEngine::CartesianVector currentFootVector = MotionEngine::GetCurrentCartesianVectorOfFootGlobal(*legPtr);
//    MathEngine::CartesianVector targetFootVector = currentFootVector;
//    double override = 50;
//    double overrideIncrement = 0.5;
//    double fCycleIncrementBase = 0.5 / (double)override;
//    double fCycleIncrement = fCycleIncrementBase * override;
//    bool bSavingKeyWasPressed = false;
//    bool bTickOfClock = false;
//
//    //setting up chrono for the cycle
//    std::chrono::milliseconds cycleTime = std::chrono::milliseconds(10);
//    std::chrono::time_point<std::chrono::system_clock> cycleStart, currentTimePoint;
//    std::chrono::milliseconds ellapsedMilliseconds;
//    cycleStart = std::chrono::system_clock::now();
//    while (1)
//    {
//        if (bTickOfClock)
//        {
//            bTickOfClock = false;
//        }
//        currentTimePoint = std::chrono::system_clock::now();
//        ellapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>
//            (
//                currentTimePoint - cycleStart
//                );
//        if (ellapsedMilliseconds >= cycleTime)
//        {
//            cycleStart = std::chrono::system_clock::now();
//            targetFootVector = currentFootVector;
//            bTickOfClock = true;
//            fCycleIncrement = fCycleIncrementBase * override;
//        }
//
//        if (GetAsyncKeyState(VK_ESCAPE) < 0)
//        {
//            break;
//        }
//
//        //gets called once per cycle...
//        if (bTickOfClock)
//        {
//            if (GetAsyncKeyState(VK_F1) < 0) // increase x
//            {
//                targetFootVector.x = targetFootVector.x + fCycleIncrement;
//            }
//            if (GetAsyncKeyState(VK_F2) < 0) // decrease x
//            {
//                targetFootVector.x = targetFootVector.x - fCycleIncrement;
//            }
//            if (GetAsyncKeyState(VK_F3) < 0) // increase y
//            {
//                targetFootVector.y = targetFootVector.y + fCycleIncrement;
//            }
//            if (GetAsyncKeyState(VK_F4) < 0) // decrease y
//            {
//                targetFootVector.y = targetFootVector.y - fCycleIncrement;
//            }
//            if (GetAsyncKeyState(VK_F6) < 0) // increase z
//            {
//                targetFootVector.z = targetFootVector.z + fCycleIncrement;
//            }
//            if (GetAsyncKeyState(VK_F7) < 0) // decrease z
//            {
//                targetFootVector.z = targetFootVector.z - fCycleIncrement;
//            }
//            if (GetAsyncKeyState(VK_F8) < 0) // increase override
//            {
//                if (override <=  100)
//                {
//                    override += overrideIncrement;
//                    std::cout << std::fixed;
//                    std::cout << std::setprecision(2);
//                    std::cout << "override set to" << override << std::endl;
//                }
//            }
//            if (GetAsyncKeyState(VK_F9) < 0) // decrease override
//            {
//                if (override > 0)
//                {
//                    override -= overrideIncrement;
//                    std::cout << std::fixed;
//                    std::cout << std::setprecision(2);
//                    std::cout << "override set to" << override << std::endl;
//                }
//            }
//
//            if (currentFootVector != targetFootVector)
//            {
//                //TODO: add boundaries...
//                //Maybe problem with negative theta1 values...
//                bool bMovedSuccessFull 
//                    = MotionEngine::MoveLegWithGlobalVector
//                    (
//                        hComm, 
//                        legPtr, 
//                        targetFootVector, 
//                        0
//                    );
//                if (!bMovedSuccessFull)
//                {
//                    std::cout << "leg did not move, min / max reached" << std::endl;
//                }
//                else
//                {
//                    //TODO: add output of current targetvector
//                    currentFootVector = MotionEngine::GetCurrentCartesianVectorOfFootGlobal(*legPtr, nullptr);
//                    std::cout << std::fixed;
//                    std::cout << std::setprecision(2);
//                    std::cout << " FootCoord: x=" << currentFootVector.x;
//                    std::cout << " y=" << currentFootVector.y;
//                    std::cout << " z=" << currentFootVector.z;
//                    std::cout << "hip=" << legPtr->GetHipJointServo()->GetCurrentAngle();
//                    std::cout << "upper=" << legPtr->GetUpperLegJointServo()->GetCurrentAngle();
//                    std::cout << "lower=" << legPtr->GetLowerLegJointServo()->GetCurrentAngle() << "\r";
//
//                }    
//            }
//        }
//    }
//    currentConfiguration = ConfigurationHandler::GetConfigurationFromServoVector(servos);
//}

int g_hComm = -1;
int g_walterCtrlX = 0;
int g_walterCtrlY = 0;
int g_walterRotateR = 0;
int g_walterRotateL = 0;
int g_walterStahp = 0;
bool g_calledFromService = false;


//int walterBtn0 = 0;
//int walterBtn1 = 0;
//int walterBtn2 = 0;
//int walterBtn3 = 0;
//int walterBtn4 = 0;
//int walterBtn5 = 0;
std::vector<int> g_walterButtons =
{
    0,0,0,0,0,0
};

//int walterAngleLeg0Hip = 0;
//int walterAngleLeg0Thigh= 0;
//int walterAngleLeg0Foot = 0;
//int walterAngleLeg1Hip = 0;
//int walterAngleLeg1Thigh= 0;
//int walterAngleLeg1Foot = 0;
//int walterAngleLeg2Hip = 0;
//int walterAngleLeg2Thigh= 0;
//int walterAngleLeg2Foot = 0;
//int walterAngleLeg3Hip = 0;
//int walterAngleLeg3Thigh= 0;
//int walterAngleLeg3Foot = 0;
//int walterAngleLeg4Hip = 0;
//int walterAngleLeg4Thigh= 0;
//int walterAngleLeg4Foot = 0;
//int walterAngleLeg5Hip = 0;
//int walterAngleLeg5Thigh= 0;
//int walterAngleLeg5Foot = 0;

//first: foot, second: thigh, third: hip
//index of vector of vectors is leg-index
std::vector<std::vector<int>> g_walterAngles =
{
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
};

mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);

std::vector<std::string> g_notifys;
std::vector<std::string> g_errors;

int main(int argc, char* argv[])
{
    //test this
    
    std::map<std::string, int*> mapOfTopics;
    mapOfTopics.emplace(std::string("WalterCtrlX"), &g_walterCtrlX);
    mapOfTopics.emplace(std::string("WalterCtrlY"), &g_walterCtrlY);
    mapOfTopics.emplace(std::string("WalterRotateR"), &g_walterRotateR);
    mapOfTopics.emplace(std::string("WalterRotateL"), &g_walterRotateL);
    mapOfTopics.emplace(std::string("WalterStop"), &g_walterStahp);
    
    mapOfTopics.emplace(std::string("BUTTONS-GROUP1-LEG1"), &g_walterButtons[0]);
    mapOfTopics.emplace(std::string("BUTTONS-GROUP1-LEG2"), &g_walterButtons[2]);
    mapOfTopics.emplace(std::string("BUTTONS-GROUP1-LEG3"), &g_walterButtons[4]);
    mapOfTopics.emplace(std::string("BUTTONS-GROUP2-LEG1"), &g_walterButtons[1]);
    mapOfTopics.emplace(std::string("BUTTONS-GROUP2-LEG2"), &g_walterButtons[3]);
    mapOfTopics.emplace(std::string("BUTTONS-GROUP2-LEG3"), &g_walterButtons[5]);
    //leg0
    mapOfTopics.emplace(std::string("ADC-GROUP1-LEG1-FOOT"), &g_walterAngles[0][0]);
    mapOfTopics.emplace(std::string("ADC-GROUP1-LEG1-THIGH"), &g_walterAngles[0][1]);
    mapOfTopics.emplace(std::string("ADC-GROUP1-LEG1-HIP"), &g_walterAngles[0][2]);
    //leg2
    mapOfTopics.emplace(std::string("ADC-GROUP1-LEG2-FOOT"), &g_walterAngles[2][0]);
    mapOfTopics.emplace(std::string("ADC-GROUP1-LEG2-THIGH"), &g_walterAngles[2][1]);
    mapOfTopics.emplace(std::string("ADC-GROUP1-LEG2-HIP"), &g_walterAngles[2][2]);
    //leg4
    mapOfTopics.emplace(std::string("ADC-GROUP1-LEG3-FOOT"), &g_walterAngles[4][0]);
    mapOfTopics.emplace(std::string("ADC-GROUP1-LEG3-THIGH"), &g_walterAngles[4][1]);
    mapOfTopics.emplace(std::string("ADC-GROUP1-LEG3-HIP"), &g_walterAngles[4][2]);
    //leg1
    mapOfTopics.emplace(std::string("ADC-GROUP2-LEG1-FOOT"), &g_walterAngles[1][0]);
    mapOfTopics.emplace(std::string("ADC-GROUP2-LEG1-THIGH"), &g_walterAngles[1][1]);
    mapOfTopics.emplace(std::string("ADC-GROUP2-LEG1-HIP"), &g_walterAngles[1][2]);
    //leg3
    mapOfTopics.emplace(std::string("ADC-GROUP2-LEG2-FOOT"), &g_walterAngles[3][0]);
    mapOfTopics.emplace(std::string("ADC-GROUP2-LEG2-THIGH"), &g_walterAngles[3][1]);
    mapOfTopics.emplace(std::string("ADC-GROUP2-LEG2-HIP"), &g_walterAngles[3][2]);
    //leg5
    mapOfTopics.emplace(std::string("ADC-GROUP2-LEG3-FOOT"), &g_walterAngles[5][0]);
    mapOfTopics.emplace(std::string("ADC-GROUP2-LEG3-THIGH"), &g_walterAngles[5][1]);
    mapOfTopics.emplace(std::string("ADC-GROUP2-LEG3-HIP"), &g_walterAngles[5][2]);


    // TODO: encapsulate in method
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(5);
    connOpts.set_clean_session(true);

    callback cb(client, connOpts, mapOfTopics);
    client.set_callback(cb);

    // Start the connection.
    // When completed, the callback will subscribe to topic.

    try 
    {
        std::cout << "Connecting to the MQTT server..." << std::flush;
        client.connect(connOpts, nullptr, cb);
        int nTries = 10;
        int nCurrentTries = 0;

        while (!client.is_connected() || nCurrentTries >= nTries)
        {
            std::cout << "waiting for client to connect" << std::endl;
            sleep(1);
            nCurrentTries++;
        }
    }
    catch (const mqtt::exception&) 
    {
        std::cerr << "\nERROR: Unable to connect to MQTT server: '"
            << SERVER_ADDRESS << "'" << std::endl;
        return 1;
    }

    //// Just block till user tells us to quit.

    //while (std::tolower(std::cin.get()) != 'q')
    //    ;

	//initialize servoPtrs
	for (int i = 0; i < g_servos.size(); i++)
	{
		g_servoPtrs.push_back(&g_servos[i]);
	}

    //initialize legs and legPtrs
    for (int nLeg = 0; nLeg < 6; nLeg++)
    {
        Leg* ptr = &g_legs[nLeg];
        g_legPtrs.push_back(ptr);
    }

	//initialize servoPtrMap
	for (Servo* servoPtr : g_servoPtrs)
	{
		bool noServoWithChannelNumberInMap
			= g_servoPtrMap.find(servoPtr->GetChannel()) == g_servoPtrMap.end();
		if (noServoWithChannelNumberInMap)
		{
			g_servoPtrMap.emplace
			(
				std::pair<int, Servo*>
				(
					servoPtr->GetChannel(),
					servoPtr
					)
			);
		}
	}

	//TODO: add check, if successfull
	g_currentConfiguration = ConfigurationHandler::LoadConfiguration(g_configFilePath);
	ConfigurationHandler::ApplyConfigurationToServoVector(g_servoPtrMap, g_currentConfiguration);

	bool bExit = false;

    if (DEBUG_MODE)
    {
        std::cout << "DEBUG_MODE activated!" << std::endl;
        //temporary place for testing things out
        TestTurningOfVectors();
    }
    else
    {
        //menu-loop
        while (!bExit)
        {
            std::string arg = "";
            system("clear");
            if (argc >= 2)
            {
                arg = argv[1];
            }
            if ("-n" == arg || "-a" == arg)
            {
                if ("-a" == arg)
                {
                    g_calledFromService = true;
                }
                Cycle();
                if (g_hComm != -1)
                {
                    serialClose(g_hComm);
                }
                bExit = true;
            }
            else
            {
                //Menu handling
                static const char menu_title[] =
                    "\n"
                    "------------------------------\n"
                    "       Walter Main Menu\n"
                    "------------------------------\n"
                    ;

                std::cout.write(menu_title, sizeof(menu_title) - 1);
                for (unsigned int i = 0; i < mainMenuEntries; ++i)
                {
                    std::cout << mainMenu[i].choice << ": "
                        << mainMenu[i].pSelectionText << std::endl;
                }
                std::cout << "Enter selection, x to quit: ";
                char choice;
                std::cin >> choice;

                if ('x' == choice)		//exit
                {
                    if (g_hComm != -1)
                    {
                        serialClose(g_hComm);
                    }
                    bExit = true;
                }
                else
                {
                    for (unsigned int i = 0; i < mainMenuEntries; ++i)
                    {
                        if (choice == mainMenu[i].choice)
                        {
                            MenuProcessingFunctionPointer pFunction
                                = mainMenu[i].pProcesingFunction;
                            (pFunction)();
                            break;
                        }
                    }
                }
            }
        }
    }
    

    // TODO: encapsulate in method
    // Disconnect
    try 
    {
        std::cout << "\nDisconnecting from the MQTT server..." << std::flush;
        client.disconnect()->wait();
        std::cout << "OK" << std::endl;
    }
    catch (const mqtt::exception & exc) 
    {
        std::cerr << exc.what() << std::endl;
        return 1;
    }
	return EXIT_SUCCESS;
}

void ProcessConnectionRequest()
{
	//TODO: add check, if port was already opened
	std::cout << "Processing connection Request to Walter" << std::endl;
	//int nPort = 11;
	bool bResult = SerialConnectionHandler::CreateConnectionLx(/*nPort,*/ &g_hComm);
	if (!bResult)
	{
		std::cout << "Failed to open connection at port" /*<< nPort */<< std::endl;
	}
	/*else
	{
		bResult = SerialConnectionHandler::ParameterizeConnection(hComm);
		if (!bResult)
		{
			std::cout << "Failed to parameterize serial ";
			std::cout << "connection, closing port " << nPort << std::endl;
			SerialConnectionHandler::TerminateConnection(&hComm);
		}
		else
		{
			if (hComm == -1)
			{
				std::cout << "Handle for port " << nPort << "is invalid" << std::endl;
			}
			else
			{
				std::cout << "Successfully opened and parameterized port " << nPort << std::endl;
			}
		}
	}*/
    else
    {
        std::cout << "Successfully opened and parameterized port " /*<< nPort*/ << std::endl;
    }
    system("PAUSE");
	return;
}

int GetLegNumberDirty(int ServoNumber, Servo::ServoKind* kind)
{
    int retVal = -1;
    if (0 == ServoNumber ||
        1 == ServoNumber ||
        2 == ServoNumber)
    {
        retVal = 0;
    }
    else if (
        4 == ServoNumber ||
        5 == ServoNumber ||
        6 == ServoNumber)
    {
        retVal = 1;
    }
    else if (
        8 == ServoNumber ||
        9 == ServoNumber ||
        10 == ServoNumber)
    {
        retVal = 2;
    }
    else if (
        16 == ServoNumber ||
        17 == ServoNumber ||
        18 == ServoNumber)
    {
        retVal = 3;
    }
    else if (
        20 == ServoNumber ||
        21 == ServoNumber ||
        22 == ServoNumber)
    {
        retVal = 4;
    }
    else if (
        24 == ServoNumber ||
        25 == ServoNumber ||
        26 == ServoNumber)
    {
        retVal = 5;
    }
    else
    {
        retVal = -1;
    }
    if (kind != nullptr)
    {
        switch (ServoNumber)
        {
        case 0:
        case 4:
        case 8:
        case 16:
        case 20:
        case 24:
            *kind = Servo::ServoKind::hipJoint;
            break;
        case 1:
        case 5:
        case 9:
        case 17:
        case 21:
        case 25:
            *kind = Servo::ServoKind::upperlegJoint;
            break;
        case 2:
        case 6:
        case 10:
        case 18:
        case 22:
        case 26:
            *kind = Servo::ServoKind::lowerlegJoint;
            break;
        default:
            *kind = Servo::ServoKind::unitialized;
        }
    }
    return retVal;
}

//TODO: make this work under linux
//void ProcessServoParameterizing()
//{
//	bool bExit = false;
//	int choice = 0;
//	std::cout << "BOUNDARIES OF THE SERVOS WILL BE OVERRIDDEN IN THIS MODE!" << std::endl << std::endl;
//	std::cout << "press arrow keys to modify angle of servo" << std::endl;
//	//std::cout << "Up/Down: SLOW, Left/Right: FAST, ESC for exit..." << std::endl;
//
//	while (!bExit)
//	{
//		std::cout << "Enter channelnumber between 0 and 31" << std::endl;
//		std::cout << "Enter 42 to exit" << std::endl;
//		std::cin >> choice;
//
//		if (choice <= 31 && 
//			choice >= 0 &&
//			servoPtrMap.find(choice) != servoPtrMap.end())
//		{
//			auto servoIterator = servoPtrMap.find(choice);
//            
//            //TODO: construct temporary leg to pass to ManualServoParam...
//            // to visualize the relative foot-coordinates..
//            Servo::ServoKind kind;
//            int legNumber = GetLegNumberDirty(servoIterator->first, &kind);
//
//            //construct Leg
//            Leg tempLeg = Leg
//            (
//                servoPtrs[0 + 3 * legNumber],
//                servoPtrs[1 + 3 * legNumber],
//                servoPtrs[2 + 3 * legNumber],
//                legNumber
//            );
//            ManualServoParamterizationLeg(hComm, &tempLeg, kind);
//			//ManualServoParamterization(hComm, servoIterator->second);
//		}
//        else if (GetAsyncKeyState(VK_ESCAPE) < 0)
//        {
//            bExit = true;
//        }
//		else if (42 == choice)
//		{
//			bExit = true;
//		}
//	}
//}

void ProcessConfigurationSaving()
{
	//TODO: generate config-entries for every servo and save it in file
	bool bRet = ConfigurationHandler::SaveConfiguration(g_configFilePath, g_currentConfiguration);
	if (bRet)
	{
		std::cout << "saved!" << std::endl;
	}
	else
	{
		std::cout << "error while saving!" << std::endl;
	}
}

//void ProcessConfigurationTest()
//{
//	//TODO: test loading and saving of configuration
//
//	std::vector<ConfigurationEntry> configEntries;
//	configEntries.emplace_back(ConfigurationEntry(0, 200, 300, 400, 300));
//	configEntries.emplace_back(ConfigurationEntry(1, 201, 301, 401, 301));
//	configEntries.emplace_back(ConfigurationEntry(2, 202, 302, 402, 302));
//	configEntries.emplace_back(ConfigurationEntry(3, 203, 303, 403, 303));
//
//	Configuration config = Configuration(configEntries);
//
//	ConfigurationHandler::SaveConfiguration(configFilePath, config);
//
//	Configuration configFromFile = ConfigurationHandler::LoadConfiguration(configFilePath);
//	bool channelIsSame = true;
//	bool minPosIsSame = true;
//	bool neutralPosIsSame = true;
//	bool maxPosIsSame = true;
//	bool allIsSame = true;
//	for (int i = 0; i < (std::min(configFromFile.GetEntries().size(), config.GetEntries().size())); i++)
//	{
//        
//		channelIsSame = channelIsSame && config.GetEntries()[i].m_nChannelNumber == configFromFile.GetEntries()[i].m_nChannelNumber;
//		minPosIsSame = minPosIsSame && config.GetEntries()[i].m_nMinPulse == configFromFile.GetEntries()[i].m_nMinPulse;
//		neutralPosIsSame = neutralPosIsSame && config.GetEntries()[i].m_nNeutralPulse == configFromFile.GetEntries()[i].m_nNeutralPulse;
//		maxPosIsSame = maxPosIsSame && config.GetEntries()[i].m_nMaxPulse == configFromFile.GetEntries()[i].m_nMaxPulse;
//		allIsSame = allIsSame && channelIsSame && minPosIsSame && neutralPosIsSame && maxPosIsSame;
//	}
//	if (!allIsSame)
//	{
//		std::cout << "Test failed" << std::endl;
//	}
//	else
//	{
//		std::cout << "Test successfull" << std::endl;
//	}
//}

//TODO: add back in 
//void ProcessConfigurationMenu()
//{
//	bool bExit = false;
//	while (!bExit)
//	{
//		static const char menu_title[] =
//			"\n"
//			"------------------------------\n"
//			" Walter Configuration Utility \n"
//			"------------------------------\n"
//			;
//		std::cout.write(menu_title, sizeof(menu_title) - 1);
//		for (unsigned int i = 0; i < configurationMenuEntries; ++i)
//		{
//			std::cout << configurationMenu[i].choice << ": "
//				<< configurationMenu[i].pSelectionText << "\n";
//		}
//		std::cout << "Enter selection, 0 to quit: ";
//		char choice;
//		std::cin >> choice;
//
//		if ('0' == choice)
//		{
//			bExit = true;
//		}
//		else
//		{
//			for (unsigned int i = 0; i < configurationMenuEntries; ++i)
//			{
//				if (choice == configurationMenu[i].choice)
//				{
//					MenuProcessingFunctionPointer pFunction
//						= configurationMenu[i].pProcesingFunction;
//					(pFunction)();
//					break;
//				}
//			}
//		}
//	}
//}

void ProcessTestTraversal()
{
    auto localEngine = MotionEngine(g_legPtrs, &g_walterButtons, &g_walterAngles, &g_notifys, &g_errors);
    std::cout << "Enter selection (0 to 5), anything else to quit: ";
    int choice;
    std::cin >> choice;
    if (choice >= 0 && choice <= 5)
    {
        localEngine.TestTraversalOfOneLeg(g_hComm, choice, 0.1);
    }
}

void ProcessConnectionTerminationRequest()
{
	if (g_hComm == -1)
	{
		std::cout << "No connection to terminate" << std::endl;
	}
	else
	{
		std::cout << "Terminating connection" << std::endl;
		/*SerialConnectionHandler::TerminateConnection(&hComm);*/
        serialClose(g_hComm);
	}
}

void ProcessMoveToHomePosition()
{
	for (int i = 0; i < 6; i++)
	{
		std::vector<Servo*> leg;
		leg.push_back(g_servoPtrs[0 + 3 * i]);
		leg.push_back(g_servoPtrs[1 + 3 * i]);
		leg.push_back(g_servoPtrs[2 + 3 * i]);

		std::vector<unsigned int> positions;
		positions.push_back(g_servoPtrs[0 + 3 * i]->GetStartPulse());
		positions.push_back(g_servoPtrs[1 + 3 * i]->GetStartPulse());
		positions.push_back(g_servoPtrs[2 + 3 * i]->GetStartPulse());

		MotionEngine::MoveGroup(g_hComm, leg, positions, 400);
		/*Sleep(100);*/
	}
}

void ProcessMoveToNeutralPosition()
{
	for (int i = 0; i < 6; i++)
	{
		std::vector<Servo*> leg;
		leg.push_back(g_servoPtrs[0 + 3 * i]);
		leg.push_back(g_servoPtrs[1 + 3 * i]);
		leg.push_back(g_servoPtrs[2 + 3 * i]);

		std::vector<unsigned int> positions;
		positions.push_back(g_servoPtrs[0 + 3 * i]->GetNeutralPulse());
		positions.push_back(g_servoPtrs[1 + 3 * i]->GetNeutralPulse());
		positions.push_back(g_servoPtrs[2 + 3 * i]->GetNeutralPulse());

		MotionEngine::MoveGroup(g_hComm, leg, positions, 300);
        /*if (i == 2)
        {
            Sleep(50);
        }*/
	}
	//system("PAUSE");
}

void ProcessMoveToNeutralPoint()
{
    /*for (int i = 0; i < 6; i++)
    {*/
        MathEngine::CartesianVector home = 
            MathEngine::CartesianVector
            (
                legConst_neutralPointX, 
                legConst_neutralPointY, 
                legConst_neutralPointZ
            );
        //Sleep(20);
    //}
    MotionEngine::MoveLegsWithLocalVector(g_hComm, g_legPtrs, home, 300);
}

//void ProcessTestInverseKinematics()
//{
//    bool bExit = false;
//    int legNumber = 0;
//    //TODO: add keys to move coordinates...maybe F-Keys?
//
//    while (!bExit)
//    {
//        std::cout << "Enter legnumber between 0 and 5" << std::endl;
//        std::cout << "Enter 42 to exit" << std::endl;
//        std::cin >> legNumber;
//
//        if (legNumber <= 5 &&
//            legNumber >= 0)
//        {
//            ManualLegMoving(hComm, legPtrs[legNumber]);
//        }
//        else if (GetAsyncKeyState(VK_ESCAPE) < 0)//TODO: find alternative for GetAsyncKeyState
//        {
//            bExit = true;
//        }
//        else if (42 == legNumber)
//        {
//            bExit = true;
//        }
//    }
//}

//void ProcessStraightWalk()
//{
//    auto localEngine = MotionEngine(legPtrs);
//    bool bBreak = false;
//    while (!bBreak)
//    {
//        localEngine.Walk(hComm, 10, 10);
//        if (GetAsyncKeyState(VK_ESCAPE) < 0)
//        {
//            bBreak = true;
//        }
//    }
//    ProcessMoveToNeutralPoint();
//}


//TODO: make this work under linux
void ProcessVariableWalk()
{
    if (UNLOCK_MOVEMENT)
    {
        std::cout << "MOVEMENT IS UNLOCKED!!" << std::endl << std::endl;
    }
    else
    {
        std::cout << "MOVEMENT IS LLLOCKED!!" << std::endl << std::endl;
    }

    auto localEngine = MotionEngine(g_legPtrs, &g_walterButtons, &g_walterAngles, &g_notifys, &g_errors);
    bool bBreak = false;
    int nX = 0;
    int nY = 0;
    int ch = -1;
    int lastCh = ch;
    nodelay(stdscr, TRUE);
    while (!bBreak)
    {
        ch = getch();
        if (ch != lastCh)
        {
            if (ch == KEY_UP)
            {
                nY = 100;
                nX = 0;
            }
            else if (ch == KEY_DOWN)
            {
                nY = -100;
                nX = 0;
            }
            else if (ch == KEY_RIGHT)
            {
                nX = 100;
                nY = 0;
            }
            else if (ch == KEY_LEFT)
            {
                nX = -100;
                nY = 0;
            }
            else if (ch == 27)//ESC
            {
                nX = 0;
                nY = 0;
            }
        }

        localEngine.Walk(g_hComm, nX, nY );
        if (ch == KEY_BACKSPACE)
        {
            bBreak = true;
        }
        lastCh = ch;
    }
    //ProcessMoveToNeutralPoint();
    nodelay(stdscr, FALSE);
}

void ProcessTurn()
{
    auto localEngine = MotionEngine(g_legPtrs, &g_walterButtons, &g_walterAngles, &g_notifys, &g_errors);
    bool bBreak = false;
    bool bTurnLeft = false;
    bool bTurn = false;
    int ch = -1;
    int lastCh = ch;
    nodelay(stdscr, TRUE);
    while (!bBreak)
    {
        ch = getch();
        if (ch != lastCh)
        {
            if (ch == KEY_RIGHT)
            {
                bTurn = true;
                bTurnLeft = false;
            }
            else if (ch == KEY_LEFT)
            {
                bTurn = true;
                bTurnLeft = true;
            }
            else if (ch == 27)//ESC
            {
                bTurn = false;
            }
        }
        if (bTurn)
        {
            localEngine.Turn(g_hComm, bTurnLeft);

        }
        if (ch == KEY_BACKSPACE)
        {
            bBreak = true;
        }
        lastCh = ch;
    }
    //ProcessMoveToNeutralPoint();
    nodelay(stdscr, FALSE);
}

void ProcessPowerTest()
{
    ProcessMoveToNeutralPosition();
    
    sleep(0.2);
    ProcessMoveToNeutralPoint();
    std::cout << "enter anything if ready, BACKSPACE for exit" << std::endl;
    int get = getch();
    nodelay(stdscr, TRUE);
    int time = 400;

    MathEngine::CartesianVector upVector = MathEngine::CartesianVector(legConst_neutralPointX, legConst_neutralPointY, legConst_neutralPointZ - 10);
    MathEngine::CartesianVector downVector = MathEngine::CartesianVector(legConst_neutralPointX, legConst_neutralPointY, legConst_neutralPointZ + 50);
    bool bBreak = false;
    bool bUp = true;
    auto startTimePoint = std::chrono::high_resolution_clock::now();
    auto currentTimePoint = startTimePoint;
    while (!bBreak)
    {
        int ch = getch();
        currentTimePoint = std::chrono::high_resolution_clock::now();
        auto difference = std::chrono::duration_cast<std::chrono::milliseconds>(currentTimePoint - startTimePoint);
        auto differenceMs = difference.count();
        if (differenceMs > time +100)
        {
            if (bUp)
            {
                MotionEngine::MoveLegsWithLocalVector(g_hComm, g_legPtrs, upVector, time);
            }
            else
            {
                MotionEngine::MoveLegsWithLocalVector(g_hComm, g_legPtrs, downVector, time);
            }
            bUp = !bUp;
            startTimePoint = std::chrono::high_resolution_clock::now();
        }

        
        if (ch == KEY_UP)
        {
            bBreak = true;
        }
    }
    nodelay(stdscr, FALSE);
}

//void ProcessRapidLegMovement()
//{
//    int leg = 0;
//
//    bool bBreak = false;
//    int toMoveTo = 1000;
//    bool direction = false;
//    while (!bBreak)
//    {
//        if (GetAsyncKeyState(VK_ESCAPE) < 0)
//        {
//            bBreak = true;
//        }
//
//        MotionEngine::MoveServoToPulseWidth(hComm, servoPtrs[leg], toMoveTo);
//        sleep(800);
//        direction = !direction;
//        if (direction)
//        {
//            toMoveTo = 1100;
//        }
//        else
//        {
//            toMoveTo = 1900;
//        }
//    }
//}

//void ProcessTestVertTraversal()
//{
//    //TODO: complete this
//    auto localEngine = MotionEngine(legPtrs);
//    std::cout << "Enter selection (0 to 5), anything else to quit: ";
//    int choice;
//    std::cin >> choice;
//    if (choice >= 0 && choice <= 5)
//    {
//        MathEngine::CartesianVector targetVectorLocal =
//            MathEngine::CartesianVector(legConst_neutralPointX, legConst_neutralPointY, legConst_neutralPointZ);
//        bool bBreak = false;
//        bool bUp = false;
//        int nResult = 0;
//        while (!bBreak)
//        {
//            if (GetAsyncKeyState(VK_ESCAPE) < 0)
//            {
//                bBreak = true;
//            }
//
//            if (!bUp)
//            {
//
//                targetVectorLocal.x = legConst_neutralPointX;
//                targetVectorLocal.y = legConst_neutralPointY;
//                targetVectorLocal.z = legConst_neutralPointZ;
//            }
//            else
//            {
//
//                targetVectorLocal.x = legConst_neutralPointX + 60;
//                targetVectorLocal.y = legConst_neutralPointY;
//                targetVectorLocal.z = legConst_neutralPointZ - 115;
//            }
//            int nResult = localEngine.TraverseTo
//            (
//                hComm,
//                choice,
//                targetVectorLocal,
//                false,
//                0.08
//            );
//            if (nResult == 0)
//            {
//                bUp = !bUp;
//            }
//        }
//    }
//}

void MoveAllLegsToStartPos()
{
    MathEngine::CartesianVector home =
        MathEngine::CartesianVector
        (
            legConst_neutralPointX + 60,
            legConst_neutralPointY,
            legConst_neutralPointZ - 110
        );
    MotionEngine::MoveLegsWithLocalVector(g_hComm, g_legPtrs, home, 1000);
}

void StandUp()
{
    auto localEngine = MotionEngine(g_legPtrs, &g_walterButtons, &g_walterAngles, &g_notifys, &g_errors);
    MathEngine::CartesianVector neutralPointLocal =
        MathEngine::CartesianVector
        (
            legConst_neutralPointX, 
            legConst_neutralPointY, 
            legConst_neutralPointZ
        );
    int nResult = 0;
    do 
    {
        for (int idxLeg = 0; idxLeg < 6; idxLeg++)
        {
            nResult = localEngine.TraverseTo
            (
                g_hComm,
                idxLeg,
                neutralPointLocal,
                false,
                0.1
            );
        }
    } while (nResult != 0);
    
}

void LayDown()
{
    auto localEngine = MotionEngine(g_legPtrs, &g_walterButtons, &g_walterAngles, &g_notifys, &g_errors);
    MathEngine::CartesianVector startPosLocal =
        MathEngine::CartesianVector
        (
            legConst_neutralPointX + 60,
            legConst_neutralPointY,
            legConst_neutralPointZ - 110
        );
    int nResult = 0;
    do
    {
        for (int idxLeg = 0; idxLeg < 6; idxLeg++)
        {
            nResult = localEngine.TraverseTo
            (
                g_hComm,
                idxLeg,
                startPosLocal,
                false,
                0.1
            );
        }
    } while (nResult != 0);
}

void TestTurningOfVectors()
{
    auto localEngine = MotionEngine(g_legPtrs, &g_walterButtons, &g_walterAngles, &g_notifys, &g_errors);

    localEngine.TestTurningOfVectors();
}

static std::mutex mqttMutex;

void GetOutFromCmd(std::string cmd, std::string* target) 
{

    std::string data;
    FILE* stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1");
    
    std::lock_guard<std::mutex> lock(mqttMutex);
    stream = popen(cmd.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
        pclose(stream);
    }
    if (nullptr != target)
    {
        *target = data;
    }
    //return data;
}

void HandleNotifys(std::vector<std::string>* notifys)
{
    if (nullptr != notifys && notifys->size() > 0)
    {
        for (std::string notify : *notifys)
        {
            client.publish("WalterNotify", notify, 1, true);
        }
        notifys->clear();
    }
}

void HandleErrors(std::vector<std::string>* errors)
{
    if (nullptr != errors && errors->size() > 0)
    {
        for (std::string notify : *errors)
        {
            client.publish("WalterNotify", notify,1, true);
        }
        errors->clear();
    }
}

//main function
void Cycle()
{
    //ncurses for manual operating
    if (!g_calledFromService)
    {
        initscr();
        keypad(stdscr, TRUE);
    }


    int nDirectionX = 0;
    int nLastDirectionX = 0;
    int nDirectionY = 0;
    int nLastDirectionY = 0;
    int nDirectionManX = 0;
    int nDirectionManY = 0;
    bool bTurn = false;
    bool bTurnMan = false;
    bool bTurnLastCycle = false;
    bool bTurnLastCycleMan = false;
    bool bTurnLeft = false;
    bool bTurnLeftLastCycle = false;
    bool bTurnLeftMan = false;
    bool bInitialized = false;

    std::string errorMsg;
    MotionEngine engine = MotionEngine
    (
        g_legPtrs, 
        &g_walterButtons, 
        &g_walterAngles, 
        &g_notifys, 
        &g_errors
    );
    Timer timer = Timer();

    Timer energySaverTimer = Timer();
    energySaverTimer.SetTime(10000);
    bool keepAliveIter = false;
    MathEngine::CartesianVector upVector =
        MathEngine::CartesianVector
        (
            legConst_neutralPointX,
            legConst_neutralPointY,
            legConst_neutralPointZ - 20
        );
    MathEngine::CartesianVector neutralVector =
        MathEngine::CartesianVector
        (
            legConst_neutralPointX,
            legConst_neutralPointY,
            legConst_neutralPointZ);
    
    RobotState state = RobotState::eInitPort;
    bool bBreak = false;
    std::cout << "--CYCLE: enter BACKSPACE to ABORT--" << std::endl << std::endl;
    
    if (!g_calledFromService)
    {
        nodelay(stdscr, TRUE);
    }

    int lastCh = 0;
    int ch = 0;
    while (!bBreak)
    {
        //emergency-stop
        if (RobotState::eStopped == state && g_walterStahp == 0)
        {
            if (!bInitialized)
            {
                state = RobotState::eInitPort;
            }
            else
            {
                state = RobotState::eStandUp;
            }
        }
        else if (g_walterStahp != 0 && RobotState::eStopped != state)
        {
            /*std::string msg = "Walter Stopped";
            client.publish("WalterNotify", msg.c_str(), 1, true);*/
            std::cout << "STOPPED!!" << std::endl;
            state = RobotState::eStopped;
        }
        
        //evaluate MQTT-variables
        nDirectionX = g_walterCtrlX;
        nDirectionY = g_walterCtrlY;
        if (g_walterRotateL || g_walterRotateR)
        {
            bTurn = true;
        }
        else
        {
            bTurn = false;
        }
        if (g_walterRotateL != 0)
        {
            bTurnLeft = true;
        }
        else
        {
            bTurnLeft = false;
        }

        if (!g_calledFromService)
        {
            ch = getch();
        }
        
        
        try
        {
            //add statemachine
            switch (state)
            {
            case RobotState::eStopped:
            {
                engine.StopTraversal();
                break;
            }
            case RobotState::eInitPort:
            {
                std::string msg = "/startup";
                client.publish("WalterDisplay", msg.c_str(), 1, true);
                //init serial Port
                SerialConnectionHandler::CreateConnectionLx(&g_hComm);
                if (-1 == g_hComm)
                {
                    errorMsg = "Error while opening port";
                    bBreak = true;
                }
                else
                {
                    state = RobotState::eMoveToStartPos;
                    std::cout << "Opened port " << g_hComm << " successfully" << std::endl << "entering MoveToStartPos" << std::endl;
                    std::string msg = "/wakeup";
                    client.publish("WalterDisplay", msg.c_str(), 1, true);
                }
                break;
            }
            case RobotState::eMoveToStartPos:
            {
                if (g_unlocked)
                {
                    //move servos to start position
                    MoveAllLegsToStartPos();
                    timer.SetTime(1000);
                    timer.Start();
                    if (timer.IsFinished())
                    {
                        state = RobotState::eStandUp;
                        std::cout << "Moved legs to startPos, entering StandUp" << std::endl;
                        bInitialized = true;
                    }
                }
                break;
            }
            case RobotState::eStandUp:
            {
                //stand up
                int result = engine.StandUp(g_hComm);
                if (0 == result)
                {
                    std::cout << "Stood up, entering normalOp" << std::endl;
                    state = RobotState::eNormalOp;
                    engine.StopTraversal();
                }
                break;
            }
            case RobotState::eNormalOp:
            {
                /*try
                {*/
                int nRet = 0;
                    if (bTurn)
                    {
                        if (bTurnLeftLastCycle != bTurnLeft)
                        {
                            engine.StopTraversal();
                        }
                        nRet = engine.Turn(g_hComm, bTurnLeft);
                    }
                    else
                    {
                        if (bTurnLastCycle)
                        {
                            //stop turn traversal, if turning was recently aborted
                            engine.StopTraversal();
                        }
                        nRet = engine.Walk(g_hComm, nDirectionX, nDirectionY);//TODO: add pointer to string for error-string
                    }
                //}
                /*catch*/ if (-1 == nRet)
                {
                    engine.StopTraversal();
                    std::stringstream msg;
                    msg << "Error in Motion-Engine! Resetting to Idle-State.." << std::endl;
                    client.publish("WalterError", msg.str());
                    client.publish("WalterNotify", msg.str());
                }

                if (nDirectionX == 0 && nDirectionY == 0 && !bTurn)
                {
                    if (nLastDirectionX != 0 || nLastDirectionY != 0)
                    {
                        timer.Stop();
                        timer.SetTime(2000);
                    }
                    timer.Start();
                    if (timer.IsFinished())
                    {
                        std::cout << "entering idle" << std::endl;
                        state = RobotState::eIdle;
                        engine.StopTraversal();
                    }
                }

                if (KEY_DOWN == ch)
                {
                    state = RobotState::eLayDown;
                    std::cout << "entering LayDown" << std::endl;
                    engine.StopTraversal();
                }
                break;
            }
            case RobotState::eIdle:
            {
                //do idle things
                //move up and down for now...
                int result = 0;
                energySaverTimer.Start();
                for (int idxLeg = 0; idxLeg < 6; idxLeg++)
                {
                    if (keepAliveIter)
                    {
                        result += engine.TraverseTo(g_hComm, idxLeg, upVector, false, 0.05);
                    }
                    else
                    {
                        result += engine.TraverseTo(g_hComm, idxLeg, neutralVector, false, 0.025);
                    }
                    if (0 == result)
                    {
                        keepAliveIter = !keepAliveIter;
                    }
                }

                if (nDirectionX != 0 || nDirectionY != 0 || bTurn != false)
                {
                    state = RobotState::eNormalOp;
                    std::cout << "entering normalOp" << std::endl;
                    engine.StopTraversal();
                    energySaverTimer.Stop();
                }

                if (KEY_DOWN == ch || energySaverTimer.IsFinished())
                {
                    state = RobotState::eLayDown;
                    std::cout << "entering LayDown" << std::endl;
                    engine.StopTraversal();
                    energySaverTimer.Stop();
                }
                if (KEY_RIGHT == ch)
                {
                    state = RobotState::eNormalOp;
                    bTurn = true;
                    bTurnLeft = false;
                }
                if (KEY_LEFT == ch)
                {
                    state = RobotState::eNormalOp;
                    bTurn = true;
                    bTurnLeft = true;
                }
                break;
            }
            case RobotState::eLayDown:
            {
                int result = engine.LayDown(g_hComm);
                if (0 == result)
                {
                    state = RobotState::eResting;
                    std::cout << "layed down, entering resting" << std::endl;
                    std::string msg = "/sleep";
                    client.publish("WalterDisplay", msg.c_str(), 1, true);
                    engine.StopTraversal();
                }
                break;
            }
            case RobotState::eResting:
            {
                if (KEY_UP == ch || nDirectionX != 0 || nDirectionY != 0 || bTurn)
                {
                    state = RobotState::eStandUp;
                    std::cout << "entering standUp" << std::endl;
                    std::string msg = "/wakeup";
                    client.publish("WalterDisplay", msg.c_str(), 1, true);
                    engine.StopTraversal();
                }
                break;
            }
            case RobotState::eHandMode:
            {
                int nRet = 0;
                if (KEY_UP == ch)
                {
                    nDirectionManY = 100;
                    nDirectionManX = 0;
                }
                if (KEY_DOWN == ch)
                {
                    nDirectionManY = -100;
                    nDirectionManX = 0;
                }
                if (KEY_RIGHT == ch)
                {
                    nDirectionManX = 100;
                    nDirectionManY = 0;
                }
                if (KEY_LEFT == ch)
                {
                    nDirectionManX = -100;
                    nDirectionManY = 0;
                }
                if ('s' == ch)
                {
                    nDirectionManX = 0;
                    nDirectionManY = 0;
                    bTurnMan = false;
                    bTurnLeftMan = false;
                }
                if ('r' == ch)
                {
                    if ('r' != lastCh)
                    {
                        if (bTurnMan && !bTurnLeftMan)
                        {
                            bTurnMan = false;
                        }
                        else
                        {
                            bTurnMan = true;
                        }
                    }
                    bTurnLeftMan = false;
                }
                if ('l' == ch)
                {
                    if ('l' != lastCh)
                    {
                        if (bTurnMan && bTurnLeftMan)
                        {
                            bTurnMan = false;
                        }
                        else
                        {
                            bTurnMan = true;
                        }
                    }
                    bTurnLeftMan = true;
                }

                /*try
                {*/
                    if (bTurnMan)
                    {
                        nRet = engine.Turn(g_hComm, bTurnLeftMan);
                    }
                    else
                    {
                        if (bTurnLastCycleMan)
                        {
                            //stop turn traversal, if turning was recently aborted
                            engine.StopTraversal();
                        }
                        nRet = engine.Walk(g_hComm, nDirectionManX, nDirectionManY);
                    }
                //}
                /*catch*/ if (-1 == nRet)
                {
                    std::cout << "Error in motion-engine, resetting to idle" << std::endl;
                    bTurnLastCycleMan = false;
                    bTurnMan = false;
                    nDirectionManX = 0;
                    nDirectionManY = 0;
                    engine.StopTraversal();
                    state = RobotState::eIdle;
                }

                if ('x' == ch)
                {
                    engine.StopTraversal();
                    state = RobotState::eNormalOp;
                    std::cout << "Entering normalOp" << std::endl;
                    bTurnMan = false;
                    bTurnLeftMan = false;
                    nDirectionManX = 0;
                    nDirectionManY = 0;
                }
            }
            default:
                break;
            }
        }
        catch (const std::exception&)
        {
            engine.StopTraversal();
            std::stringstream msg;
            msg << "Error in Motion-Engine! Resetting to Idle-State.." << std::endl;
            client.publish("WalterError", msg.str());
            client.publish("WalterNotify", msg.str());
            state = RobotState::eIdle;
            std::cout << "caught exception" << std::endl;
        }
        /*if (RobotState::eNormalOp == state)
        {
            keepAliveTimer.Start();
            if (keepAliveTimer.IsFinished())
            {
                keepAliveTimer.Restart();
                engine.KeepLegsAlive(hComm, keepAliveIter);
                std::cout << "kept legs alive" << std::endl;
                keepAliveIter = !keepAliveIter;
            }
        }*/
        if (ch == KEY_BACKSPACE)
        {
            bBreak = true;
        }
        if (ch == 'm')
        {
            state = eHandMode;
            engine.StopTraversal();
            std::cout << "entering Handmode" << std::endl;
            std::cout << "Abort Motion: s; Exit Handmode: x" << std::endl;
        }
        nLastDirectionX = nDirectionX;
        nLastDirectionY = nDirectionY;
        bTurnLastCycle = bTurn;
        bTurnLeftLastCycle = bTurnLeft;
        bTurnLastCycleMan = bTurnMan;
        
        lastCh = ch;
        /*HandleErrors(&g_errors);
        HandleNotifys(&g_notifys);*/
        g_notifys.clear();
        g_errors.clear();
    }
    if (!g_calledFromService)
    {
        nodelay(stdscr, FALSE);
        endwin();
    }
}