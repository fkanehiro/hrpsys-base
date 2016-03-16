#ifndef BEEP_H
#define BEEP_H

void init_beep();
void start_beep(int freq, int length=50);
void stop_beep();
void quit_beep();

typedef enum {BEEP_INFO_START, BEEP_INFO_FREQ, BEEP_INFO_LENGTH, NUM_BEEP_INFO} beep_info;

// Beep client to send command to BeeperRTC
#include <rtm/idl/BasicDataTypeSkel.h>
class BeepClient
{
 private:
  bool is_start_beep, prev_is_start_beep;
  int freq, length;
 public:
  BeepClient () : is_start_beep(false), prev_is_start_beep(false) {};
  ~BeepClient () {};
  int get_num_beep_info () const { return NUM_BEEP_INFO; };
  void startBeep (const int _freq, const int _length = 50)
  {
    prev_is_start_beep = is_start_beep;
    is_start_beep = true;
    freq = _freq;
    length = _length;
  };
  void stopBeep ()
  {
    prev_is_start_beep = is_start_beep;
    is_start_beep = false;
    freq = 1; // dummy
    length = 0; // dummy
  };
  bool isWritable () const
  {
    // Write data port to overwrite and pass through between client RTCs.
    // If currently "start" or changed to "stop", write data port.
    // If keep "stop", do not write data port.
    return (is_start_beep || prev_is_start_beep);
  };
  void setDataPort (RTC::TimedLongSeq& out_data)
  {
    out_data.data[BEEP_INFO_START] = (is_start_beep?1:0);
    out_data.data[BEEP_INFO_FREQ] = freq;
    out_data.data[BEEP_INFO_LENGTH] = length;
  };
};
#endif // BEEP_H
