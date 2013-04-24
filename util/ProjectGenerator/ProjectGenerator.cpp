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
#include <hrpModel/Sensor.h>
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
  std::string conf_file_option;

  for (int i = 1; i < argc; ++ i) {
    std::string arg(argv[i]);
    coil::normalize(arg);
    if ( arg == "--output" ) {
      if (++i < argc) output = argv[i];
    } else if ( arg == "-o" ) {
      ++i;
    } else if ( arg == "--conf-file-option" ) {
      if (++i < argc) conf_file_option += std::string("\n") + argv[i];
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
        xmlTextWriterWriteProperty(writer, "totalTime", "2000000.0");
	xmlTextWriterWriteProperty(writer, "method", "EULER");
      }
      xmlTextWriterEndElement(writer); // item
      for (std::vector<std::string>::iterator it = inputs.begin();
	   it != inputs.end();
	   it++) {
	std::string filename = *it;
	hrp::BodyPtr body(new hrp::Body());
	if (!loadBodyFromModelLoader(body, filename.c_str(),
				     CosNaming::NamingContext::_duplicate(naming.getRootContext()),
				     true)){
	  std::cerr << "failed to load model[" << filename << "]" << std::endl;
	  return 1;
	}
	std::string name = body->name();

	xmlTextWriterStartElement(writer, BAD_CAST "item");
	xmlTextWriterWriteAttribute(writer, BAD_CAST "class", BAD_CAST "com.generalrobotix.ui.item.GrxRTSItem");
	xmlTextWriterWriteAttribute(writer, BAD_CAST "name", BAD_CAST name.c_str());
	xmlTextWriterWriteAttribute(writer, BAD_CAST "select", BAD_CAST "true");

	xmlTextWriterWriteProperty(writer, name+"(Robot)0.period", "0.005");
	xmlTextWriterWriteProperty(writer, "HGcontroller0.period", "0.005");
	xmlTextWriterWriteProperty(writer, "HGcontroller0.factory", "HGcontroller");
	xmlTextWriterWriteProperty(writer, "connection", "HGcontroller0.qOut:"+name+"(Robot)0.qRef");
	xmlTextWriterWriteProperty(writer, "connection", "HGcontroller0.dqOut:"+name+"(Robot)0.dqRef");
	xmlTextWriterWriteProperty(writer, "connection", "HGcontroller0.ddqOut:"+name+"(Robot)0.ddqRef");
	xmlTextWriterEndElement(writer); // item


	xmlTextWriterStartElement(writer, BAD_CAST "item");
	xmlTextWriterWriteAttribute(writer, BAD_CAST "class", BAD_CAST "com.generalrobotix.ui.item.GrxModelItem");
        xmlTextWriterWriteAttribute(writer, BAD_CAST "name", BAD_CAST (basename(*it).c_str()));
	xmlTextWriterWriteAttribute(writer, BAD_CAST "url", BAD_CAST filename.c_str());

	xmlTextWriterWriteProperty(writer, "rtcName", name + "(Robot)0");
	xmlTextWriterWriteProperty(writer, "inport", "qRef:JOINT_VALUE");
	xmlTextWriterWriteProperty(writer, "inport", "dqRef:JOINT_VELOCITY");
	xmlTextWriterWriteProperty(writer, "inport", "ddqRef:JOINT_ACCELERATION");
	xmlTextWriterWriteProperty(writer, "outport", "q:JOINT_VALUE");
    xmlTextWriterWriteProperty(writer, "outport", "tau:JOINT_TORQUE");

    // set outport for sensros
    int nforce = body->numSensors(hrp::Sensor::FORCE);
    for (unsigned int i=0; i<nforce; i++){
        hrp::Sensor *s = body->sensor(hrp::Sensor::FORCE, i);
        // port name and sensor name is same in case of ForceSensor
        xmlTextWriterWriteProperty(writer, "outport", s->name + ":" + s->name + ":FORCE_SENSOR");
        std::cerr << s->name << std::endl;
    }
    int ngyro = body->numSensors(hrp::Sensor::RATE_GYRO);
    if(ngyro == 1){
      // port is named with no number when there is only one gyro
      hrp::Sensor *s = body->sensor(hrp::Sensor::RATE_GYRO, 0);
      xmlTextWriterWriteProperty(writer, "outport", "rate:" + s->name + ":RATE_GYRO_SENSOR");
      std::cerr << s->name << std::endl;
    }else{
      for (unsigned int i=0; i<ngyro; i++){
        hrp::Sensor *s = body->sensor(hrp::Sensor::RATE_GYRO, i);
        std::stringstream str_strm;
        str_strm << "rate" << i << ":" + s->name << ":RATE_GYRO_SENSOR";
        xmlTextWriterWriteProperty(writer, "outport", str_strm.str());
        std::cerr << s->name << std::endl;
      }
    }
    int nacc = body->numSensors(hrp::Sensor::ACCELERATION);
    if(nacc == 1){
      // port is named with no number when there is only one acc
      hrp::Sensor *s = body->sensor(hrp::Sensor::ACCELERATION, 0);      
      xmlTextWriterWriteProperty(writer, "outport", "acc:" + s->name + ":ACCELERATION_SENSOR");
      std::cerr << s->name << std::endl;
    }else{
      for (unsigned int i=0; i<nacc; i++){
        hrp::Sensor *s = body->sensor(hrp::Sensor::ACCELERATION, i);
        std::stringstream str_strm;
        str_strm << "acc" << i << ":" << s->name << ":ACCELERATION_SENSOR";
        xmlTextWriterWriteProperty(writer, "outport", str_strm.str());
        std::cerr << s->name << std::endl;
      }
    }
    
	//
	std::string root_name = body->rootLink()->name;
	xmlTextWriterWriteProperty(writer, root_name+".NumOfAABB", "1");
	std::ostringstream os;
	os << body->rootLink()->p[0] << "  "
	   << body->rootLink()->p[1] << "  "
	   << body->rootLink()->p[2] + 0.1; // 10cm margin
	xmlTextWriterWriteProperty(writer, root_name+".translation", os.str());

	xmlTextWriterWriteProperty(writer, root_name+".rotation", "0.0 0.0 1.0 0.0");
	if ( ! body->isStaticModel() ) {
	  xmlTextWriterWriteProperty(writer, root_name+".mode", "Torque");
	  xmlTextWriterWriteProperty(writer, "controller", basename(output));
	}
	for(int i = 0; i < body->numJoints(); i++){
	  if ( body->joint(i)->index > 0 ) {
	    std::cerr << body->joint(i)->jointId << std::endl;
	    std::string joint_name = body->joint(i)->name;
	    xmlTextWriterWriteProperty(writer, joint_name+".angle", "0.0");
	    xmlTextWriterWriteProperty(writer, joint_name+".mode", "HighGain");
	    xmlTextWriterWriteProperty(writer, joint_name+".NumOfAABB", "1");
	  }
	}
	xmlTextWriterEndElement(writer); // item

	//
	xmlTextWriterStartElement(writer, BAD_CAST "item");
	xmlTextWriterWriteAttribute(writer, BAD_CAST "class", BAD_CAST "com.generalrobotix.ui.item.GrxCollisionPairItem");
	xmlTextWriterWriteAttribute(writer, BAD_CAST "name", BAD_CAST std::string("CP#"+name).c_str());
	xmlTextWriterWriteAttribute(writer, BAD_CAST "select", BAD_CAST "true");
	{
	  xmlTextWriterWriteProperty(writer, "springConstant", "0 0 0 0 0 0");
	  xmlTextWriterWriteProperty(writer, "slidingFriction", "0.5");
	  xmlTextWriterWriteProperty(writer, "jointName2", "");
	  xmlTextWriterWriteProperty(writer, "jointName1", "");
	  xmlTextWriterWriteProperty(writer, "damperConstant", "0 0 0 0 0 0");
	  xmlTextWriterWriteProperty(writer, "objectName2", name);
	  xmlTextWriterWriteProperty(writer, "objectName1", name);
	  xmlTextWriterWriteProperty(writer, "springDamperModel", "false");
	  xmlTextWriterWriteProperty(writer, "staticFriction", "0.5");
	}
	xmlTextWriterEndElement(writer); // item

	xmlTextWriterStartElement(writer, BAD_CAST "view");
	xmlTextWriterWriteAttribute(writer, BAD_CAST "class", BAD_CAST "com.generalrobotix.ui.view.GrxRobotHardwareClientView");
	xmlTextWriterWriteAttribute(writer, BAD_CAST "name", BAD_CAST "RobotHardware RTC Client");
	xmlTextWriterWriteProperty(writer, "robotHost", "localhost");
	xmlTextWriterWriteProperty(writer, "StateHolderRTC", "StateHolder0");
	xmlTextWriterWriteProperty(writer, "interval", "100");
	xmlTextWriterWriteProperty(writer, "RobotHardwareServiceRTC", "RobotHardware0");
	xmlTextWriterWriteProperty(writer, "robotPort", "2809");
	xmlTextWriterWriteProperty(writer, "ROBOT", name.c_str());
	xmlTextWriterEndElement(writer); // item

	xmlTextWriterStartElement(writer, BAD_CAST "view");
	xmlTextWriterWriteAttribute(writer, BAD_CAST "class", BAD_CAST "com.generalrobotix.ui.view.Grx3DView");
	xmlTextWriterWriteAttribute(writer, BAD_CAST "name", BAD_CAST "3DView");
	xmlTextWriterWriteProperty(writer, "view.mode", "Room");
	xmlTextWriterWriteProperty(writer, "showCoM", "false");
	xmlTextWriterWriteProperty(writer, "showCoMonFloor", "false");
	xmlTextWriterWriteProperty(writer, "showDistance", "false");
	xmlTextWriterWriteProperty(writer, "showIntersection", "false");
	xmlTextWriterWriteProperty(writer, "eyeHomePosition", "-0.70711 -0 0.70711 2 0.70711 -0 0.70711 2 0 1 0 0.8 0 0 0 1 ");
	xmlTextWriterWriteProperty(writer, "showCollision", "true");
	xmlTextWriterWriteProperty(writer, "showActualState", "true");
	xmlTextWriterWriteProperty(writer, "showScale", "true");
	xmlTextWriterEndElement(writer); // view
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

  {
      std::string conf_file = output.substr(0,output.find_last_of('.'))+".conf";
      std::fstream s(conf_file.c_str(), std::ios::out);
  
      s << "model: file://" << inputs[0] << std::endl;
      s << "dt: 0.005" << std::endl;
      s << conf_file_option << std::endl;
  }

  {
      std::string conf_file = output.substr(0,output.find_last_of('.'))+".RobotHardware.conf";
      std::fstream s(conf_file.c_str(), std::ios::out);
  
      s << "model: file://" << inputs[0] << std::endl;
      s << "exec_cxt.periodic.type: hrpExecutionContext" << std::endl;
      s << "sexec_cxt.periodic.rate: 200" << std::endl;
  }

  return 0;
}
