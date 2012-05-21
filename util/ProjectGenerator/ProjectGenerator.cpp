// -*- C++ -*-
/*!
 * @file ProjectGeneratorComp.cpp
 * @brief Standalone component
 * @date $Date$
 *
 * $Id$
 */

#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>
#include <string>
#include <sstream>
#include <fstream>

void xmlTextWriterWriteProperty(const xmlTextWriterPtr writer, const std::string name, const std::string value) {
  xmlTextWriterStartElement(writer, BAD_CAST "property");
  xmlTextWriterWriteAttribute(writer, BAD_CAST "name", BAD_CAST name.c_str());
  xmlTextWriterWriteAttribute(writer, BAD_CAST "value", BAD_CAST value.c_str());
  xmlTextWriterEndElement(writer);
}

std::string basename(const std::string name){
  std::string ret = name.substr(name.find_last_of('/')+1);
  return ret.substr(0,ret.find_last_of('.'));
}

int main (int argc, char** argv)
{
  std::string output;
  std::vector<std::string> inputs;

  for (int i = 1; i < argc; ++ i) {
    std::string arg(argv[i]);
    coil::normalize(arg);
    if ( arg == "--output" ) {
      if (++i < argc) output = argv[i];
    } else if ( arg == "-o" ) {
      ++i;
    } else if ( arg[0] == '-' ||  arg[0] == '_'  ) {
      std::cerr << argv[0] << " : Unknwon arguments " << arg << std::endl;
    } else {
      inputs.push_back(argv[i]);
    }
  }

  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);

  std::string nameServer = manager->getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
    comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(manager->getORB(), nameServer.c_str());

  xmlTextWriterPtr writer;
  writer = xmlNewTextWriterFilename(output.c_str(), 0);
  xmlTextWriterSetIndent(writer, 4);
  xmlTextWriterStartElement(writer, BAD_CAST "grxui");
  {
    xmlTextWriterStartElement(writer, BAD_CAST "mode");
    xmlTextWriterWriteAttribute(writer, BAD_CAST "name", BAD_CAST "Simulation");
    {
      xmlTextWriterStartElement(writer, BAD_CAST "item");
      xmlTextWriterWriteAttribute(writer, BAD_CAST "class", BAD_CAST "com.generalrobotix.ui.item.GrxSimulationItem");
      xmlTextWriterWriteAttribute(writer, BAD_CAST "name", BAD_CAST "simulationItem");
      {
	xmlTextWriterWriteProperty(writer, "integrate", "true");
	xmlTextWriterWriteProperty(writer, "timeStep", "0.0050");
	xmlTextWriterWriteProperty(writer, "method", "EULER");
      }
      xmlTextWriterEndElement(writer); // item
      for (std::vector<std::string>::iterator it = inputs.begin();
	   it != inputs.end();
	   it++) {
	std::string filename = *it;
	std::string name = filename.substr(filename.find_last_of('/')+1);
	name = name.substr(0,name.size()-4);

	xmlTextWriterStartElement(writer, BAD_CAST "item");
	xmlTextWriterWriteAttribute(writer, BAD_CAST "class", BAD_CAST "com.generalrobotix.ui.item.GrxModelItem");
	xmlTextWriterWriteAttribute(writer, BAD_CAST "name", BAD_CAST name.c_str());
	xmlTextWriterWriteAttribute(writer, BAD_CAST "url", BAD_CAST filename.c_str());

	hrp::BodyPtr body = new hrp::Body();
	if (!loadBodyFromModelLoader(body, filename.c_str(),
				     CosNaming::NamingContext::_duplicate(naming.getRootContext()),
				     true)){
	  std::cerr << "failed to load model[n" << filename << "]" << std::endl;
	  return 1;
	}
	name = body->rootLink()->name;
	xmlTextWriterWriteProperty(writer, name+".NumOfAABB", "1");
	std::ostringstream os;
	os << body->rootLink()->p[0] << "  "
	   << body->rootLink()->p[1] << "  "
	   << body->rootLink()->p[2] + 0.1; // 10cm margin
	xmlTextWriterWriteProperty(writer, name+".translation", os.str());

	xmlTextWriterWriteProperty(writer, name+".rotation", "0.0 0.0 1.0 0.0");
	if ( ! body->isStaticModel() ) {
	  xmlTextWriterWriteProperty(writer, name+".mode", "Torque");
	  xmlTextWriterWriteProperty(writer, "controller", basename(output));
	}
	for(int i = 0; i < body->numJoints(); i++){
	  if ( body->joint(i)->index > 0 ) {
	    std::cerr << body->joint(i)->jointId << std::endl;
	    name = body->joint(i)->name;
	    xmlTextWriterWriteProperty(writer, name+".angle", "0.0");
	    xmlTextWriterWriteProperty(writer, name+".mode", "HighGain");
	    xmlTextWriterWriteProperty(writer, name+".NumOfAABB", "1");
	  }
	}
	xmlTextWriterEndElement(writer); // item
      }

      {
	for (std::vector<std::string>::iterator it1 = inputs.begin(); it1 != inputs.end(); it1++) {
	  std::string name1 = basename(*it1);
	  for (std::vector<std::string>::iterator it2 = it1+1; it2 != inputs.end(); it2++) {
	    std::string name2 = basename(*it2);

	    xmlTextWriterStartElement(writer, BAD_CAST "item");
	    xmlTextWriterWriteAttribute(writer, BAD_CAST "class", BAD_CAST "com.generalrobotix.ui.item.GrxCollisionPairItem");
	    xmlTextWriterWriteAttribute(writer, BAD_CAST "name", BAD_CAST std::string("CP#"+name2+"_#"+name1+"_").c_str());
	    {
	      xmlTextWriterWriteProperty(writer, "springConstant", "0 0 0 0 0 0");
	      xmlTextWriterWriteProperty(writer, "slidingFriction", "0.5");
	      xmlTextWriterWriteProperty(writer, "jointName2", "");
	      xmlTextWriterWriteProperty(writer, "jointName1", "");
	      xmlTextWriterWriteProperty(writer, "damperConstant", "0 0 0 0 0 0");
	      xmlTextWriterWriteProperty(writer, "objectName2", name2);
	      xmlTextWriterWriteProperty(writer, "objectName1", name1);
	      xmlTextWriterWriteProperty(writer, "springDamperModel", "false");
	      xmlTextWriterWriteProperty(writer, "staticFriction", "0.5");
	    }
	    xmlTextWriterEndElement(writer); // item
	  }
	}
      }
    }
    xmlTextWriterEndElement(writer); // mode
  }
  xmlTextWriterEndElement(writer); // grxui

  xmlFreeTextWriter(writer);

  std::string conf_file = output.substr(0,output.find_last_of('.'))+".conf";
  std::fstream s(conf_file.c_str(), std::ios::out);
  
  s << "model: file://" << inputs[0] << std::endl;
  s << "dt: 0.005" << std::endl;

  return 0;
}
