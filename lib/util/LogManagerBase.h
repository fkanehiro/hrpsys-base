#ifndef __LOG_MANAGER_BASE_H__
#define __LOG_MANAGER_BASE_H__

#define DEFAULT_FPS 10

class LogManagerBase
{
public:
    LogManagerBase() : 
        m_isPlaying(false), m_isRecording(false), m_playRatio(1.0){}
    virtual ~LogManagerBase(){}
    virtual void play() = 0;
    virtual bool record() = 0;
    virtual void faster() = 0;
    virtual void slower() = 0;
    virtual void head() = 0; 
    virtual void tail() = 0; 
    virtual void prev(int delta) = 0;
    virtual void next(int delta) = 0;
    virtual void move(double ratio) = 0;
    virtual unsigned int length() = 0;
    virtual double currentTime() = 0;
    virtual int updateIndex() = 0;
    virtual int index() = 0;
    virtual void clear() = 0;
    bool isPlaying(){ return m_isPlaying; }
    bool isRecording(){ return m_isRecording; }
    double playRatio() { return m_playRatio; }
protected:
    bool m_isPlaying, m_isRecording;
    double m_playRatio;
};
#endif
