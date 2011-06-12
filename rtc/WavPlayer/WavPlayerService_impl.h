// -*-C++-*-
#ifndef WAVPLAYSERVICESVC_IMPL_H
#define WAVPLAYSERVICESVC_IMPL_H

#include "WavPlayerService.hh"

using namespace OpenHRP;

class WavPlayerService_impl 
  : public virtual POA_OpenHRP::WavPlayerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  WavPlayerService_impl();
  virtual ~WavPlayerService_impl();
  //
  void playWav(const char *filename);
  void playWavNoWait(const char *filename);
private:
};				 

#endif
