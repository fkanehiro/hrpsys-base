#ifndef __LOG_MANAGER_H__
#define __LOG_MANAGER_H__

#include <iostream>
#include <sys/time.h>
#include <deque>
#include <boost/thread/thread.hpp>
#include "LogManagerBase.h"

template<class T>
class LogManager : public LogManagerBase
{
public:
    LogManager() : m_index(-1), m_isNewStateAdded(false), m_atLast(true),
        m_maxLogLength(0){
    }
    void add(const T& state){
        boost::mutex::scoped_lock lock(m_mutex);
        m_log.push_back(state);
        if (m_log.size() == 1) m_offsetT = state.time;
        if (m_maxLogLength > 0 && m_log.size() > m_maxLogLength) {
            m_log.pop_front();
            if (m_index >= 1) m_index--;
        }
        m_isNewStateAdded = true;
    }
    void clear(){
        boost::mutex::scoped_lock lock(m_mutex);
        m_log.clear();
        m_index = -1;
        m_atLast = true;
    }
    void prev(int delta=1){ 
        boost::mutex::scoped_lock lock(m_mutex);
        setIndex(m_index - delta); 
    }
    void next(int delta=1){ 
        boost::mutex::scoped_lock lock(m_mutex);
        setIndex(m_index + delta); 
    }
    void head(){ 
        boost::mutex::scoped_lock lock(m_mutex);
        setIndex(0); 
    }
    void tail(){ 
        boost::mutex::scoped_lock lock(m_mutex);
        if (!m_log.empty()) setIndex(m_log.size()-1); 
    } 
    void move(double ratio){
        boost::mutex::scoped_lock lock(m_mutex);
        if (m_log.size()) setIndex(ratio*(m_log.size()-1));
    }
    bool isNewStateAdded() { return m_isNewStateAdded; }
    double currentTime() { 
        boost::mutex::scoped_lock lock(m_mutex);
        if (!m_log.empty() && m_index>=0){
            return m_log[m_index].time - m_offsetT;
        }else{
            return -1;
        }
    }
    int index() { return m_index; }
    double time(int i) { 
        boost::mutex::scoped_lock lock(m_mutex);
        return m_log[i].time; 
    }
    void faster(){
        boost::mutex::scoped_lock lock(m_mutex);
        m_playRatio *= 2;
        if (m_isPlaying){
            m_initT = m_log[m_index].time;
            gettimeofday(&m_startT, NULL);
        }
    }
    void slower(){
        boost::mutex::scoped_lock lock(m_mutex);
        m_playRatio /= 2;
        if (m_isPlaying){
            m_initT = m_log[m_index].time;
            gettimeofday(&m_startT, NULL);
        }
    }
    bool record(){
        boost::mutex::scoped_lock lock(m_mutex);
        if (m_log.empty()) return false;

        setIndex(0);
        m_initT = m_log[0].time;
        m_isRecording = true;
        return true;
    }
    void play(){
        boost::mutex::scoped_lock lock(m_mutex);
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
        boost::mutex::scoped_lock lock(m_mutex);
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
    T& state() { 
        boost::mutex::scoped_lock lock(m_mutex);
        return m_log[m_index]; 
    }
    void enableRingBuffer(int len) { m_maxLogLength = len; }
    unsigned int length() { 
        boost::mutex::scoped_lock lock(m_mutex);
        return m_log.size(); 
    }
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
    double m_offsetT;
    boost::mutex m_mutex; 
};
#endif
