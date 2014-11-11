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
  std::vector<std::string> inputs, filenames; // filenames is for conf file
  std::string conf_file_option, robothardware_conf_file_option, integrate("true"), dt("0.005"), timeStep(dt), joint_properties;
  bool use_highgain_mode(true);

  int rtmargc=0;
  std::vector<char *> rtmargv;
  rtmargv.push_back(argv[0]);
  rtmargc++;
  for (int i = 1; i < argc; ++ i) {
    std::string arg(argv[i]);
    coil::normalize(arg);
    if ( arg == "--output" ) {
      if (++i < argc) output = argv[i];
    } else if ( arg == "--integrate" ) {
      if (++i < argc) integrate = argv[i];
    } else if ( arg == "--dt" ) {
      if (++i < argc) dt = argv[i];
    } else if ( arg == "--timestep" ) {
      if (++i < argc) timeStep = argv[i];
    } else if ( arg == "--conf-file-option" ) {
      if (++i < argc) conf_file_option += std::string("\n") + argv[i];
    } else if ( arg == "--robothardware-conf-file-option" ) {
      if (++i < argc) robothardware_conf_file_option += std::string("\n") + argv[i];
    } else if ( arg == "--joint-properties" ) {
      if (++i < argc) joint_properties = argv[i];
    } else if ( arg == "--use-highgain-mode" ) {
      if (++i < argc) use_highgain_mode = (std::string(argv[i])==std::string("true")?true:false);
    } else if ( arg.find("--gtest_output") == 0  ||arg.find("--text") == 0 || arg.find("__log") == 0 || arg.find("__name") == 0 ) { // skip
    } else if ( arg[0] == '-'  ) {
      rtmargv.push_back(argv[i]);
      rtmargv.push_back(argv[i+1]);
      rtmargc+=2;
      ++i;
    } else {
      inputs.push_back(argv[i]);
    }
  }

  RTC::Manager* manager;
  manager = RTC::Manager::init(rtmargc, rtmargv.data());

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
	xmlTextWriterWriteProperty(writer, "integrate", integrate);
	xmlTextWriterWriteProperty(writer, "timeStep", timeStep);
        xmlTextWriterWriteProperty(writer, "totalTime", "2000000.0");
	xmlTextWriterWriteProperty(writer, "method", "EULER");
      }
      xmlTextWriterEndElement(writer); // item
      // default WAIST offset
      hrp::Vector3 WAIST_offset_pos = hrp::Vector3::Zero();
      Eigen::AngleAxis<double> WAIST_offset_rot = Eigen::AngleAxis<double>(0, hrp::Vector3(0,0,1)); // rotation in VRML is represented by axis + angle
      for (std::vector<std::string>::iterator it = inputs.begin();
	   it != inputs.end();
	   it++) {
        // argument for VRML supports following two mode:
        //  1. xxx.wrl
        //    To specify VRML file. WAIST offsets is 0 transformation.
        //  2. xxx.wrl,0,0,0,0,0,1,0
        //    To specify both VRML and WAIST offsets.
        //    WAIST offset: for example, "0,0,0,0,0,1,0" -> position offset (3dof) + axis for rotation offset (3dof) + angle for rotation offset (1dof)
        coil::vstring filename_arg_str = coil::split(*it, ",");
	std::string filename = filename_arg_str[0];
        filenames.push_back(filename);
        if ( filename_arg_str.size () > 1 ){ // if WAIST offset is specified 
          for (size_t i = 0; i < 3; i++) {
            coil::stringTo(WAIST_offset_pos(i), filename_arg_str[i+1].c_str());
            coil::stringTo(WAIST_offset_rot.axis()(i), filename_arg_str[i+1+3].c_str());
          }
          coil::stringTo(WAIST_offset_rot.angle(), filename_arg_str[1+3+3].c_str());
        }
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

	xmlTextWriterWriteProperty(writer, name+"(Robot)0.period", dt);
        if (use_highgain_mode) {
          xmlTextWriterWriteProperty(writer, "HGcontroller0.period", dt);
          xmlTextWriterWriteProperty(writer, "HGcontroller0.factory", "HGcontroller");
          if (it==inputs.begin()) {
            xmlTextWriterWriteProperty(writer, "connection", "HGcontroller0.qOut:"+name+"(Robot)0.qRef");
            xmlTextWriterWriteProperty(writer, "connection", "HGcontroller0.dqOut:"+name+"(Robot)0.dqRef");
            xmlTextWriterWriteProperty(writer, "connection", "HGcontroller0.ddqOut:"+name+"(Robot)0.ddqRef");
          }
        } else {
          xmlTextWriterWriteProperty(writer, "PDcontroller0.period", dt);
          xmlTextWriterWriteProperty(writer, "PDcontroller0.factory", "PDcontroller");
          if (it==inputs.begin()) {
            xmlTextWriterWriteProperty(writer, "connection", "PDcontroller0.torque:"+name+"(Robot)0.tauRef");
            xmlTextWriterWriteProperty(writer, "connection", ""+name+"(Robot)0.q:PDcontroller0.angle");
          }
        }
	xmlTextWriterEndElement(writer); // item


	xmlTextWriterStartElement(writer, BAD_CAST "item");
	xmlTextWriterWriteAttribute(writer, BAD_CAST "class", BAD_CAST "com.generalrobotix.ui.item.GrxModelItem");
        xmlTextWriterWriteAttribute(writer, BAD_CAST "name", BAD_CAST (basename(*it).c_str()));
	xmlTextWriterWriteAttribute(writer, BAD_CAST "url", BAD_CAST filename.c_str());

	xmlTextWriterWriteProperty(writer, "rtcName", name + "(Robot)0");
        if (use_highgain_mode) {
          xmlTextWriterWriteProperty(writer, "inport", "qRef:JOINT_VALUE");
          xmlTextWriterWriteProperty(writer, "inport", "dqRef:JOINT_VELOCITY");
          xmlTextWriterWriteProperty(writer, "inport", "ddqRef:JOINT_ACCELERATION");
        } else {
          xmlTextWriterWriteProperty(writer, "inport", "tauRef:JOINT_TORQUE");
        }
	xmlTextWriterWriteProperty(writer, "outport", "q:JOINT_VALUE");
    xmlTextWriterWriteProperty(writer, "outport", "tau:JOINT_TORQUE");

    // set outport for sensros
    int nforce = body->numSensors(hrp::Sensor::FORCE);
    if ( nforce > 0 ) std::cerr << "hrp::Sensor::FORCE";
    for (unsigned int i=0; i<nforce; i++){
        hrp::Sensor *s = body->sensor(hrp::Sensor::FORCE, i);
        // port name and sensor name is same in case of ForceSensor
        xmlTextWriterWriteProperty(writer, "outport", s->name + ":" + s->name + ":FORCE_SENSOR");
        std::cerr << " " << s->name;
    }
    if ( nforce > 0 ) std::cerr << std::endl;
    int ngyro = body->numSensors(hrp::Sensor::RATE_GYRO);
    if ( ngyro > 0 ) std::cerr << "hrp::Sensor::GYRO";
    if(ngyro == 1){
      // port is named with no number when there is only one gyro
      hrp::Sensor *s = body->sensor(hrp::Sensor::RATE_GYRO, 0);
      xmlTextWriterWriteProperty(writer, "outport", s->name + ":" + s->name + ":RATE_GYRO_SENSOR");
      std::cerr << " " << s->name;
    }else{
      for (unsigned int i=0; i<ngyro; i++){
        hrp::Sensor *s = body->sensor(hrp::Sensor::RATE_GYRO, i);
        std::stringstream str_strm;
        str_strm << s->name << i << ":" + s->name << ":RATE_GYRO_SENSOR";
        xmlTextWriterWriteProperty(writer, "outport", str_strm.str());
        std::cerr << " " << s->name;
      }
    }
    if ( ngyro > 0 ) std::cerr << std::endl;
    int nacc = body->numSensors(hrp::Sensor::ACCELERATION);
    if ( nacc > 0 ) std::cerr << "hrp::Sensor::ACCELERATION";
    if(nacc == 1){
      // port is named with no number when there is only one acc
      hrp::Sensor *s = body->sensor(hrp::Sensor::ACCELERATION, 0);      
      xmlTextWriterWriteProperty(writer, "outport", s->name + ":" + s->name + ":ACCELERATION_SENSOR");
      std::cerr << " " << s->name;
    }else{
      for (unsigned int i=0; i<nacc; i++){
        hrp::Sensor *s = body->sensor(hrp::Sensor::ACCELERATION, i);
        std::stringstream str_strm;
        str_strm << s->name << i << ":" << s->name << ":ACCELERATION_SENSOR";
        xmlTextWriterWriteProperty(writer, "outport", str_strm.str());
        std::cerr << " " << s->name;
      }
    }
    if ( nacc > 0 ) std::cerr << std::endl;
    
	//
	std::string root_name = body->rootLink()->name;
	xmlTextWriterWriteProperty(writer, root_name+".NumOfAABB", "1");

        // write waist pos and rot by considering both VRML original WAIST and WAIST_offset_pos and WAIST_offset_rot from arguments
	std::ostringstream os;
	os << body->rootLink()->p[0] + WAIST_offset_pos(0) << "  "
	   << body->rootLink()->p[1] + WAIST_offset_pos(1) << "  "
	   << body->rootLink()->p[2] + WAIST_offset_pos(2); // 10cm margin
	xmlTextWriterWriteProperty(writer, root_name+".translation", os.str());
        os.str(""); // reset ostringstream
        Eigen::AngleAxis<double> tmpAA = Eigen::AngleAxis<double>(hrp::Matrix33(body->rootLink()->R * WAIST_offset_rot.toRotationMatrix()));
        os << tmpAA.axis()(0) << " "
           << tmpAA.axis()(1) << " "
           << tmpAA.axis()(2) << " "
           << tmpAA.angle();
        xmlTextWriterWriteProperty(writer, root_name+".rotation", os.str());

	if ( ! body->isStaticModel() ) {
	  xmlTextWriterWriteProperty(writer, root_name+".mode", "Torque");
	  xmlTextWriterWriteProperty(writer, "controller", basename(output));
	}

        // store joint properties
        //   [property 1],[value 1],....
        //   ex. --joint-properties RARM_JOINT0.angle,0.0,RARM_JOINT2.mode,HighGain,...
        coil::vstring joint_properties_arg_str = coil::split(joint_properties, ",");
        std::map <std::string, std::string> joint_properties_map;
        for (size_t i = 0; i < joint_properties_arg_str.size()/2; i++) {
          joint_properties_map.insert(std::pair<std::string, std::string>(joint_properties_arg_str[i*2], joint_properties_arg_str[i*2+1]));
        }
        if ( body->numJoints() > 0 ) std::cerr << "hrp::Joint";
	for(int i = 0; i < body->numJoints(); i++){
	  if ( body->joint(i)->index > 0 ) {
	    std::cerr << " " << body->joint(i)->name << "(" << body->joint(i)->jointId << ")";
	    std::string joint_name = body->joint(i)->name;
            std::string j_property = joint_name+".angle";
	    xmlTextWriterWriteProperty(writer, j_property,
                                       (joint_properties_map.find(j_property) != joint_properties_map.end()) ? joint_properties_map[j_property] : "0.0");
            j_property = joint_name+".mode";
	    xmlTextWriterWriteProperty(writer, j_property,
                                       (joint_properties_map.find(j_property) != joint_properties_map.end()) ? joint_properties_map[j_property] : (use_highgain_mode?"HighGain":"Torque"));
            j_property = joint_name+".NumOfAABB";
	    xmlTextWriterWriteProperty(writer, j_property,
                                       (joint_properties_map.find(j_property) != joint_properties_map.end()) ? joint_properties_map[j_property] : "1");
	  }
	}
	xmlTextWriterEndElement(writer); // item
        if ( body->numJoints() > 0 ) std::cerr << std::endl;

	//
        // comment out self collision settings according to issues at https://github.com/fkanehiro/hrpsys-base/issues/122
	// xmlTextWriterStartElement(writer, BAD_CAST "item");
	// xmlTextWriterWriteAttribute(writer, BAD_CAST "class", BAD_CAST "com.generalrobotix.ui.item.GrxCollisionPairItem");
	// xmlTextWriterWriteAttribute(writer, BAD_CAST "name", BAD_CAST std::string("CP#"+name).c_str());
	// xmlTextWriterWriteAttribute(writer, BAD_CAST "select", BAD_CAST "true");
	// {
	//   xmlTextWriterWriteProperty(writer, "springConstant", "0 0 0 0 0 0");
	//   xmlTextWriterWriteProperty(writer, "slidingFriction", "0.5");
	//   xmlTextWriterWriteProperty(writer, "jointName2", "");
	//   xmlTextWriterWriteProperty(writer, "jointName1", "");
	//   xmlTextWriterWriteProperty(writer, "damperConstant", "0 0 0 0 0 0");
	//   xmlTextWriterWriteProperty(writer, "objectName2", name);
	//   xmlTextWriterWriteProperty(writer, "objectName1", name);
	//   xmlTextWriterWriteProperty(writer, "springDamperModel", "false");
	//   xmlTextWriterWriteProperty(writer, "staticFriction", "0.5");
	// }
	// xmlTextWriterEndElement(writer); // item

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
  
      s << "model: file://" << filenames[0] << std::endl;
      s << "dt: " << dt << std::endl;
      s << conf_file_option << std::endl;
  }

  {
      std::string conf_file = output.substr(0,output.find_last_of('.'))+".RobotHardware.conf";
      std::fstream s(conf_file.c_str(), std::ios::out);
  
      s << "model: file://" << filenames[0] << std::endl;
      s << "exec_cxt.periodic.type: hrpExecutionContext" << std::endl;
      s << "exec_cxt.periodic.rate: " << static_cast<size_t>(1/atof(dt.c_str())+0.5) << std::endl; // rounding to specify integer rate value
      s << robothardware_conf_file_option << std::endl;
  }

  return 0;
}
