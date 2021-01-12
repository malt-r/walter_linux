#include "SerialConnectionHandler.h"

#include <wiringPi.h>
#include <wiringSerial.h>

//bool SerialConnectionHandler::CreateConnection(int nPort, HANDLE* phComm)
//{
//	bool bRet = true;
//		if (nPort < 0 || nPort > 256)
//		{
//			std::cout << "nPort was invalid" << std::endl;
//			bRet = false;
//		}
//		else if (phComm == NULL)
//		{
//			std::cout << "hComm was null" << std::endl;
//			bRet = false;
//		}
//		else
//		{
//			std::string sPortString("\\\\.\\COM");
//			sPortString.append(std::to_string(nPort));
//
//			*phComm = CreateFile(sPortString.c_str(),                //port name
//				GENERIC_READ | GENERIC_WRITE, //Read/Write
//				0,                            // No Sharing
//				NULL,                         // No Security
//				OPEN_EXISTING,// Open existing port only
//				0,            // Non Overlapped I/O
//				NULL);        // Null for Comm Devices
//
//			if (*phComm == INVALID_HANDLE_VALUE)
//			{
//                std::cout << "Error in opening serial port" << std::endl;
//                std::cout << "GetLastError: " << GetLastError() << std::endl;
//				bRet = false;
//			}
//			else
//			{
//				std::cout << "opening serial port successful" << std::endl;
//			}
//		}
//		return bRet;
//}
//
//bool SerialConnectionHandler::ParameterizeConnection(HANDLE hComm)
//{
//	bool bRet = true;
//		if (hComm == INVALID_HANDLE_VALUE)
//		{
//			std::cout << "hComm was invalid!" << std::endl;
//			bRet = false;
//		}
//		else
//		{
//			DCB dcb;
//			dcb.DCBlength = sizeof(DCB);
//			bool bSuccess = GetCommState(hComm, &dcb);
//			if (!bSuccess)
//			{
//				std::cout << "Error while loading DCB-structure from Port! Errorcode: "
//					<< GetLastError() << std::endl;
//				bRet = false;
//			}
//			else
//			{
//				dcb.BaudRate = CBR_115200;     //  baud rate
//				dcb.ByteSize = 8;             //  data size, xmit and rcv
//				dcb.Parity = NOPARITY;      //  parity bit
//				dcb.StopBits = ONESTOPBIT;    //  stop bit
//				bSuccess = SetCommState(hComm, &dcb);
//				if (!bSuccess)
//				{
//					std::cout << "Error while loading DCB-structure to Port! Errorcode: "
//						<< GetLastError() << std::endl;
//					bRet = false;
//				}
//			}
//		}
//		return bRet;
//}
//
//bool SerialConnectionHandler::TerminateConnection(HANDLE* hComm)
//{
//	if (nullptr == hComm)
//	{
//		std::cout << "hCommm was nullptr" << std::endl;
//        return false;
//	}
//	else
//	{
//		if (*hComm == INVALID_HANDLE_VALUE)
//		{
//			std::cout << "Handle is invalid!" << std::endl;
//			return false;
//		}
//		else
//		{
//			//Closing the Serial Port
//			CloseHandle(*hComm);
//			//*hComm = INVALID_HANDLE_VALUE;
//			return true;
//		}
//	}
//}

bool SerialConnectionHandler::CreateConnectionLx(int* serialNumber)
{
	if (serialNumber == nullptr)
	{
		return false;
	}
	else
	{
		int ret = wiringPiSetup();
		*serialNumber = serialOpen("/dev/lynxmotion", 115200);
		if (*serialNumber != -1)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}
