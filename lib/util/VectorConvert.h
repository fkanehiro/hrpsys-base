// -*- C++ -*-

#include <istream>
#include <ostream>
#include <vector>
#include <string>
#include <coil/stringutil.h>
#include <hrpUtil/EigenTypes.h>

template<typename T>
std::istream& operator>>(std::istream& is, std::vector<T>& v)
{
  std::string s;
  std::vector<std::string> sv;
  getline(is,s);
  sv = coil::split(s ,",");
  v.resize(sv.size());
  for (int i(0), len(sv.size()); i < len; ++i)
    {
      T tv;
      if (coil::stringTo(tv, sv[i].c_str()))
	{
	  v[i] = tv;
	}
    }
  return is;
}

std::istream& operator>>(std::istream& is, hrp::dvector& v)
{
  std::string s;
  std::vector<std::string> sv;
  getline(is,s);
  sv = coil::split(s ,",");
  v.resize(sv.size());
  for (int i(0), len(sv.size()); i < len; ++i)
    {
      double tv;
      if (coil::stringTo(tv, sv[i].c_str()))
	{
	  v[i] = tv;
	}
    }
  return is;
}

std::istream& operator>>(std::istream& is, hrp::Vector3& v)
{
  std::string s;
  std::vector<std::string> sv;
  getline(is,s);
  sv = coil::split(s ,",");
  for (int i(0); i < 3; ++i)
    {
      double tv;
      if (coil::stringTo(tv, sv[i].c_str()))
	{
	  v[i] = tv;
	}
    }
  return is;
}
