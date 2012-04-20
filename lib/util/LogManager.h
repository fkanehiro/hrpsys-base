#ifndef __LOG_MANAGER_H__
#define __LOG_MANAGER_H__

#include <iostream>
#include <sys/time.h>
#include <deque>
#include "LogManagerBase.h"

template<class T>
class LogManager : public LogManagerBase
{
public:
    LogManager() : m_index(-1), m_isNewStateAdded(false), m_atLast(true),
        m_maxLogLength(0){
    }
    void add(const T& state){
        m_log.push_back(state);
        if (m_maxLogLength > 0 && m_log.size() > m_maxLogLength) m_log.pop_front();
        m_isNewStateAdded = true;
    }
    void clear(){
        m_log.clear();
        m_index = -1;
    }
    void prev(int delta=1){ setIndex(m_index - delta); }
    void next(int delta=1){ setIndex(m_index + delta); }
    void head(){ setIndex(0); }
    void tail(){ if (!m_log.empty()) setIndex(m_log.size()-1); } 
    void move(double ratio){
        if (m_log.size()) setIndex(ratio*(m_log.size()-1));
    }
    bool isNewStateAdded() { return m_isNewStateAdded; }
    double currentTime() { 
        if (!m_log.empty() && m_index>=0){
            return m_log[m_index].time - m_log[0].time;
        }else{
            return -1;
        }
    }
    int index() { return m_index; }
    double time(int i) { return m_log[i].time; }
    void faster(){
        m_playRatio *= 2;
        if (m_isPlaying){
            m_initT = m_log[m_index].time;
            gettimeofday(&m_startT, NULL);
        }
    }
    void slower(){
        m_playRatio /= 2;
        if (m_isPlaying){
            m_initT = m_log[m_index].time;
            gettimeofday(&m_startT, NULL);
        }
    }
    bool record(){
        if (m_log.empty()) return false;

        setIndex(0);
        m_initT = m_log[0].time;
        m_isRecording = true;
        return true;
    }
    void play(){
        if (m_log.empty()) return;

        if (!m_isPlaying){
            m_isPlaying = true;
            if (m_atLast) setIndex(0);
            m_initT = m_log[m_index].time;
            gettimeofday(&m_startT, NULL);
        }else{
            m_isPlaying = false;
        }
    }
    int updateIndex(){
        if (m_isPlaying){
            // compute time to draw
            struct timeval tv;
            gettimeofday(&tv, NULL);
            double drawT = m_initT + ((tv.tv_sec - m_startT.tv_sec) + (tv.tv_usec - m_startT.tv_usec)*1e-6)*m_playRatio;
            //
            while(drawT > m_log[m_index].time){
                setIndex(m_index+1);
                if (m_atLast) {
                    m_isPlaying = false;
                    break;
                }
            }
        } else if (m_isNewStateAdded && m_atLast){
            // draw newest state
            setIndex(m_log.size() - 1);
            m_isNewStateAdded = false;
        }
        if(m_isRecording){
            while(m_initT > m_log[m_index].time){
                setIndex(m_index+1);
                if (m_atLast) {
                    m_isRecording = false;
                    break;
                }
            } 
            m_initT += 1.0/DEFAULT_FPS*m_playRatio;
        }
        return m_index;
    }
    T& state() { return m_log[m_index]; }
    void enableRingBuffer(int len) { m_maxLogLength = len; }
    unsigned int length() { return m_log.size(); }
protected:
    void setIndex(int i){
        if (m_log.empty()) return;
        
        m_index = i;
        if (m_index < 0) m_index = 0;
        if (m_index >= m_log.size()) m_index = m_log.size()-1;
        m_atLast = m_index == m_log.size()-1; 
    }

    std::deque<T> m_log;
    int m_index;
    bool m_isNewStateAdded, m_atLast;
    double m_initT;
    struct timeval m_startT;
    int m_maxLogLength;
};
#endif
