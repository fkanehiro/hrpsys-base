#include <iostream.h>
#include <fstream.h>
#include <string.h>

#include "vclip.h"

////////////////////////////////////////////////////////////////////////////////
// data structures
////////////////////////////////////////////////////////////////////////////////

struct Body {
  PolyTree *geometry;  // object geometry
  VclipPose pose;      // xformation from PolyTree to global frame
};

////////////////////////////////////////////////////////////////////////////////
// main
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  int i, j;
  PolyTreeLibrary polyTreeLibrary;
  list<Body> bodies;
  list<Body>::const_iterator body1i, body2i;
  Body body;

  Real dist;        // distance between bodies
  Vect3 cp1, cp2;   // closest points between bodies, in body (local) frames
  Vect3 cp1g, cp2g; // closest points between bodies, in global frame
  VclipPose X12;         // xform from body1 frame to body2 frame

  char ptreeFname[200]; // filename of a PolyTree file
  char geomName[200];   // name of a PolyTree for a body's geometru
  ClosestFeaturesHT closestFeaturesHT(3000);


  if (argc != 2) {
    cout << "usage:  allpairs <body-spec-file>" << endl;
    return 1;
  }

  // read in required PolyTree files
  ifstream ifs(argv[1]);
  while (1) {
    ifs >> ptreeFname;
    if (!strcmp(ptreeFname, "*")) break;
    cout << "loading " << ptreeFname << endl;
    loadPolyTreeFile(ptreeFname, polyTreeLibrary);
  }


  // read in bodies
  cout << "bodies:" << endl;
  for (i = 0; ; i++) {
    ifs >> geomName;
    if (!strcmp(geomName, "*")) break;
    if (!(body.geometry = polyTreeLibrary.create(geomName))) {
      cout << "error:  PolyTree " << geomName << " not defined" << endl;
      return 1;
    }
    ifs >> body.pose;
    bodies.push_back(body);
    cout << i << "  " << body.geometry->name << "\n" << body.pose << endl;
  }

  
  // run vclip between all pairs of bodies
  for (body1i = bodies.begin(), i = 0; body1i != bodies.end(); ++body1i, i++)
    for ((body2i = body1i)++, j = i+1; body2i != bodies.end(); ++body2i, j++) {
      
      X12.invert(body2i->pose);
      X12.postmult(body1i->pose);  // X12 is xform from body1 to body2 frame

      dist = PolyTree::vclip(body1i->geometry, body2i->geometry, X12, 
			     closestFeaturesHT, cp1, cp2);
      body1i->pose.xformPoint(cp1, cp1g);
      body2i->pose.xformPoint(cp2, cp2g);

      cout 
	<< "body pair (" << i << "," << j << ")" << endl
	<< "  distance = " << dist
	<< ((dist <= 0.0) ? " (penetration)" : " (disjoint)") << endl
	<< "  closest point, body " << i << ": " << cp1 << " / " << cp1g << endl
	<< "  closest point, body " << j << ": " << cp2 << " / " << cp2g << endl
	<< endl;

    }

}

