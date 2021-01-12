#include "PerformanceTimer.h"
#include <iostream>
#include <string>
#include <sstream>
//#include <Windows.h>

PerformanceTimer::PerformanceTimer()
{
    m_startPoint = std::chrono::high_resolution_clock::now();
}

PerformanceTimer::PerformanceTimer(std::string output)
{
    m_output = output;
    m_startPoint = std::chrono::high_resolution_clock::now();
}

PerformanceTimer::~PerformanceTimer()
{
    m_endPoint = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_startPoint).time_since_epoch().count();
    auto end = std::chrono::time_point_cast<std::chrono::microseconds>(m_endPoint).time_since_epoch().count();
    auto ellapsedMicroseconds = end - start;
    double ms = ellapsedMicroseconds * 0.001;
    std::stringstream output;
    output << m_output << " timed ";
    output << ellapsedMicroseconds << "us (" << ms << "ms)\n";
    std::cout << output.str().c_str();
}


