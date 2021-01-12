#include "Timer.h"

Timer::Timer(/*int64_t durationInMs*/)
{
    /*m_durationInMs = durationInMs;*/
    m_durationInMs = 0;
}

//void Timer::Start()
//{
//    m_startPoint = std::chrono::system_clock::now();
//}

void Timer::Start()
{
    if (!m_bIsRunning)
    {
        m_startPoint = std::chrono::system_clock::now();
        m_bIsRunning = true;
    }
}

void Timer::Stop()
{
    m_bIsRunning = false;
}

void Timer::Restart()
{
    m_startPoint = std::chrono::system_clock::now();
    m_bIsRunning = true;
}

void Timer::SetTime(int64_t durationInMs)
{
    m_durationInMs = durationInMs;
}

bool Timer::IsFinished()
{
    bool finished = false;
    if (m_bIsRunning)
    {
        auto now = std::chrono::system_clock::now();
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_startPoint);
        auto ms = milliseconds.count();
        finished = ms >= m_durationInMs;
        if (finished)
        {
            m_bIsRunning = false;
        }
    }
    return finished;
}

Timer::~Timer()
{
}
