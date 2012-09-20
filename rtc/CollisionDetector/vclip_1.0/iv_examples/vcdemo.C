#include <stdlib.h>
#include <math.h>
#include <stdlib.h>
#include <iostream.h>
#include <fstream.h>

#include <Inventor/engines/SoElapsedTime.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/sensors/SoNodeSensor.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/manips/SoTransformBoxManip.h>

#include "vclip.h"

// maximum number of PolyTrees in demo
#define MAX_PTREES 100

////////////////////////////////////////////////////////////////////////////////
// global variables
////////////////////////////////////////////////////////////////////////////////


SoSelection *root;                 // top of viewer scene graph
PolyTreeLibrary polyTreeLib;       // PolyTree library
ClosestFeaturesHT cfHash(3000);    // closest features Hash Table for V-Clip

SoNodeSensor *manipSensor;       // sensor to respond to user manipulation
SoTimerSensor *autoMotionSensor; // sensor to cause automatic motion
float time_;                     // parameterizes PolyTree motion
int speed = 50;                  // controls PolyTree movement speed
int selectedPtreeIdx;            // index of currently selected PolyTree, or -1

// Material choices for rendering PolyTrees;  See materials.iv for details.
SoSeparator  *matChoices;
int numMatChoices;


int numPtrees;                       // number of PolyTrees in demo
PolyTree     *ptree   [MAX_PTREES];  // PolyTree i
SoSeparator  *graph   [MAX_PTREES];  // scene graph for PolyTree i
Se3          pose     [MAX_PTREES];  // current pose of PolyTree i


// Each PolyTree follows a trajectory defined by these parameters.
// The three position coordinates each follow an independent sinusoid;
// these are described by amp, freq, and phase.  The rotation is
// constant speed about a fixed axis, and is described by axis, omega,
// and spinPhase.
Vect3        amp     [MAX_PTREES];
Vect3        freq    [MAX_PTREES];
Vect3        phase   [MAX_PTREES];
Vect3        axis    [MAX_PTREES];
Real         omega   [MAX_PTREES];
Real         spinPhase [MAX_PTREES];

// These control the closest point segements between PolyTrees.
// cpLoc[i][j] and cpLoc[j][i] point to SoTranslations used in drawing
// the spheres at the closest points between PolyTrees i and j.
// cpSegCoords do the same for the endpoints of the segment.
// cpSegSwitch turns the segment on and off, based on whether
// PolyTrees i and j are disjoint or penetrating.
SoSwitch      *cpSegSwitch [MAX_PTREES][MAX_PTREES];
SoTranslation *cpLoc       [MAX_PTREES][MAX_PTREES];
SoCoordinate3 *cpSegCoords [MAX_PTREES][MAX_PTREES];

// pens indicates which pairs of PolyTrees are currently penetrating;
// numPens indicates how many other PolyTrees currently penetrate a
// given one.
int pens [MAX_PTREES][MAX_PTREES];
int numPens [MAX_PTREES];



////////////////////////////////////////////////////////////////////////////////
// misc. support stuff
////////////////////////////////////////////////////////////////////////////////


#define XFORM(i)       ((SoTransform *)         graph[i]->getChild(0))
#define XFORM_MANIP(i) ((SoTransformBoxManip *) graph[i]->getChild(0))

// return random integer from 0 to n-1
int random(int n)
{
  return (int) floor(((double) n) * rand() / (RAND_MAX + 1));
}


// return random real in interval [0,1)
double randomReal()
{
  return ((double) rand()) / (RAND_MAX + 1);
}



////////////////////////////////////////////////////////////////////////////////
// updating scene graph as PolyTrees move
////////////////////////////////////////////////////////////////////////////////


// Update closest point segments.  If only >= 0, only segs involving
// that specific PolyTree are changed.  Useful since user can only
// move one at a time.

void updateCpSegs(int only = -1)
{
  int i, j;
  Real d;
  Vect3 cpi, cpj;
  Se3 Tij;
  MatX Xij;
  SbVec3f sbvi, sbvj;

  for (i = 0; i < numPtrees; i++)
    for (j = i + 1; j < numPtrees; j++) {

      if (only >= 0 && i != only && j != only) continue;

      Tij.invert(pose[j]);
      Tij.postmult(pose[i]);
      Xij.set(Tij);

      d = PolyTree::vclip(ptree[i], ptree[j], Xij, cfHash, cpi, cpj);

      if (d > 0) {
	if (pens[i][j]) {
	  pens[i][j] = 0;
	  numPens[i]--;
	  numPens[j]--;
	}
	cpSegSwitch[i][j]->whichChild = 0;  // enable cp segment
	pose[i].xformPoint(cpi);
	pose[j].xformPoint(cpj);
	cpi.toSbVec3f(sbvi);
	cpj.toSbVec3f(sbvj);
	cpLoc[i][j]->translation.setValue(sbvi);
	cpLoc[j][i]->translation.setValue(sbvj);
	cpSegCoords[i][j]->point.set1Value(0, sbvi);
	cpSegCoords[i][j]->point.set1Value(1, sbvj);
      }
      else {
	if (!pens[i][j]) {
	  pens[i][j] = 1;
	  numPens[i]++;
	  numPens[j]++;
	}
	cpSegSwitch[i][j]->whichChild = SO_SWITCH_NONE;  // disable cp segment
      }
    }

  // update materials of PolyTrees based on numPens
  for (i = 0; i < numPtrees; i++) graph[i]->replaceChild(1, 
  matChoices->getChild(numPens[i] < numMatChoices ? 
		       numPens[i] : numMatChoices - 1));
}


// Called at regular, short intervals to automatically move the
// PolyTrees along their orbits.

void updatePolys(void *, SoSensor *)
{
  int i;
  Vect3 d;
  static SbVec3f zero(0, 0, 0);
  Quat q;

  time_ += (speed / 500.0);

  for (i = 0; i < numPtrees; i++) {
    if (i == selectedPtreeIdx) continue; // PolyTree being manipulated by user
    d.x = amp[i].x * cos(freq[i].x * time_ + phase[i].x);
    d.y = amp[i].y * cos(freq[i].y * time_ + phase[i].y);
    d.z = amp[i].z * cos(freq[i].z * time_ + phase[i].z);
    q.set(time_ * omega[i] + spinPhase[i], axis[i], FALSE);
    pose[i].set(q, d);
    pose[i].toSoTransform(XFORM(i));
  }

  updateCpSegs();
}


////////////////////////////////////////////////////////////////////////////////
// interaction stuff
////////////////////////////////////////////////////////////////////////////////


// Convert selection path to a PolyTree index.  Return -1 if we can't
// find the corresponding PolyTree.

int pathToPtreeIndex(SoPath *selection)
{
  int i;
  SoTransform *selectionXform;
  SoSearchAction sa;

  if (!selection->getTail()->isOfType(SoShapeKit::getClassTypeId())) 
    return -1;

  sa.setType(SoTransform::getClassTypeId(), TRUE);// catch TransformManips also
  sa.setInterest(SoSearchAction::FIRST);
  sa.apply(selection);
  if (!sa.getPath()) return -1;
  
  selectionXform = (SoTransform *) sa.getPath()->getTail();

  for (i = 0; i < numPtrees; i++) if (XFORM(i) == selectionXform) break;
  return (i < numPtrees) ? i : -1;
}


// Callback when a manipulator is dragged.

void CB_manip(void *data, SoSensor *)
{
  int i;

  i = (int) data;
  pose[i].set(XFORM_MANIP(i));
  updateCpSegs(i);
}


// Called when a PolyTree is selected.

void attachManip(void *, SoPath *selectionPath) 
{
  int i;
  SoTransformBoxManip *manip;
  SoPath *xformPath;
  SoSearchAction sa;

  selectedPtreeIdx = pathToPtreeIndex(selectionPath);
  cout << "grabbed polytree " << selectedPtreeIdx << endl;
  if (selectedPtreeIdx < 0) return;
  
  sa.setNode(XFORM(selectedPtreeIdx));
  sa.apply(selectionPath);
  xformPath = sa.getPath();
  manip = new SoTransformBoxManip;
  manip->replaceNode(xformPath);

  manipSensor = new SoNodeSensor(CB_manip, (void *) selectedPtreeIdx);
  manipSensor->setPriority(0);
  manipSensor->attach(manip);
}


// Called when a PolyTree is deselected.

void detachManip(void *, SoPath *deselectionPath)
{
  int i;
  SoTransformBoxManip *manip;
  SoPath *xformPath;
  SoSearchAction sa;
  float angle;
  SbVec3f ax, trans, scale;
  SbRotation rot, scaleOrientation;
  SbMatrix R, Rinv;

  i = pathToPtreeIndex(deselectionPath);
  //cout << "released polytree " << i << endl;
  if (selectedPtreeIdx < 0) return;
  if (selectedPtreeIdx == i) selectedPtreeIdx = -1;

  sa.setNode(XFORM(i));
  sa.apply(deselectionPath);
  xformPath = sa.getPath();
  manip = (SoTransformBoxManip *) xformPath->getTail();

  // choose new trajectory parameters that are consistent with new position
  pose[i].set(XFORM_MANIP(i));
  amp[i].x = (randomReal() + 1.0) * fabs(pose[i].trans().x);
  amp[i].y = (randomReal() + 1.0) * fabs(pose[i].trans().y);
  amp[i].z = (randomReal() + 1.0) * fabs(pose[i].trans().z);
  phase[i].x = fmod(acos(pose[i].trans().x/amp[i].x) - freq[i].x*time_, 2*M_PI);
  phase[i].y = fmod(acos(pose[i].trans().y/amp[i].y) - freq[i].y*time_, 2*M_PI);
  phase[i].z = fmod(acos(pose[i].trans().z/amp[i].z) - freq[i].z*time_, 2*M_PI);
  XFORM_MANIP(i)->getRotationSpaceMatrix(R, Rinv);
  R.getTransform(trans, rot, scale, scaleOrientation);
  rot.getValue(ax, angle);
  axis[i].set(ax);
  spinPhase[i] = fmod(angle - time_ * omega[i], 2 * M_PI);

  manip->replaceManip(xformPath, new SoTransform);
  delete manipSensor;
}


////////////////////////////////////////////////////////////////////////////////
// initialization
////////////////////////////////////////////////////////////////////////////////


// One-time intialization, called from main()
void setup()
{
  numPtrees = 0;

  static SoElapsedTime *timer = new SoElapsedTime;
  timer->speed = 2.0;
  SoSFTime *theTime = (SoSFTime *)
    SoDB::createGlobalField("theTime", SoType::fromName("SoSFTime"));
  theTime->connectFrom(&timer->timeOut);
  autoMotionSensor = new SoTimerSensor(updatePolys, theTime);
  autoMotionSensor->setInterval(1/2000.0);

  root->addSelectionCallback(attachManip);
  root->addDeselectionCallback(detachManip);

  // initialize material choices
  matChoices = new SoSeparator;
  matChoices->ref();
  SoInput sceneInput;
  sceneInput.openFile("materials.iv");
  matChoices = SoDB::readAll(&sceneInput);
  if (!matChoices) cerr << "error reading materials.iv file \a" << endl;
  sceneInput.closeFile();
  numMatChoices = matChoices->getNumChildren();

}


// Initialization when opening a new demo file.  Read in PolyTrees,
// compute their scene graphs, and hook them onto the root of the main
// scene graph.  Initialize the graph, amp, freq, phase, and feat
// arrays.

void initPtrees(const char *demoFname)
{
  int i, j;
  SoSeparator *sep, *segGroup, *choices;
  SoNode *gfx;
  Real maxRad, radScaling;
  Vect3 J;
  SbVec3f zero(0, 0, 0);
  char pname[80], polyFname[200];


  time_ = 0;

  // clear out old demo debris
  root->deselectAll();
  root->removeAllChildren();
  polyTreeLib.clear();
  cfHash.clear();
  for (i = 0; i < numPtrees; i++) delete ptree[i];
  selectedPtreeIdx = -1;

  // load required PolyTree files
  cout << "demoFname=" << demoFname << endl;
  ifstream ifs(demoFname);
  while (1) {
    ifs >> polyFname;
    if (!ifs.good()) {
      cerr << "error reading demo file " << endl;
      exit(1);
    }
    if (!strcmp(polyFname, "*")) break;
    cout << "loading " << polyFname << endl;
    loadPolyTreeFile(polyFname, polyTreeLib);
  }

  ifs >> radScaling;  // read radius scale factor

  // instantiate PolyTrees
  maxRad = 0.0;
  for (i = 0; ; i++) {
    ifs >> pname;
    if (!ifs.good()) break;
    ptree[i] = polyTreeLib.create(pname);
    if (!ptree[i]) {
      cerr << "error:  can't find PolyTree " << pname << endl;
      exit(1);
    }
    if (ptree[i]->rad() > maxRad) maxRad = ptree[i]->rad();
    gfx = ptree[i]->buildInvModel();

    // Build scene graph for PolyTree i.  The top node is a separator with
    // three children: xform, material, and gfx model (NodeKit), in
    // that order.
    graph[i] = new SoSeparator;
    graph[i]->addChild(new SoTransform);
    graph[i]->addChild(matChoices->getChild(0));
    graph[i]->addChild(gfx);
    root->addChild(graph[i]);
  }
  numPtrees = i;
  cout << numPtrees << " PolyTrees instantiated" << endl;

  // Choose random initial trajectory parameters for each PolyTree
  maxRad *= radScaling;
  for (i = 0; i < numPtrees; i++) {
    amp[i].x = 5.0 * maxRad * randomReal(); 
    amp[i].y = 5.0 * maxRad * randomReal();
    amp[i].z = 5.0 * maxRad * randomReal();
    freq[i].x = 2.0 * randomReal() - 1.0;
    freq[i].y = 2.0 * randomReal() - 1.0;
    freq[i].z = 2.0 * randomReal() - 1.0;
    phase[i].x = 2 * M_PI * randomReal();
    phase[i].y = 2 * M_PI * randomReal();
    phase[i].z = 2 * M_PI * randomReal();

    axis[i].x = 2 * randomReal() - 1;
    axis[i].y = 2 * randomReal() - 1;
    axis[i].z = 2 * randomReal() - 1;
    axis[i].normalize();
    omega[i] = 1 * randomReal();
    spinPhase[i] = 0.0;
  }

  // initialize penetration state variables 
  for (i = 0; i < numPtrees; i++) {
    numPens[i] = 0;
    for (j = 0; j < numPtrees; j++) pens[i][j] = 0;
  }

  // intialize closest point segment part of scene graph

  SoMaterial *mat = new SoMaterial;
  mat->diffuseColor.setValue(0.0, 1.0, 0.0); // green closest point segments
  root->addChild(mat);

  SoSphere *sphere[MAX_PTREES];
  for (i = 0; i < numPtrees; i++) {
    sphere[i] = new SoSphere;
    sphere[i]->radius.setValue(0.05 * ptree[i]->rad());
  }

  for (i = 0; i < numPtrees; i++)
    for (j = i+1; j < numPtrees; j++) {

      cpSegSwitch[i][j] = new SoSwitch;
      segGroup = new SoSeparator;
      cpSegSwitch[i][j]->addChild(segGroup);

      sep = new SoSeparator;
      cpLoc[i][j] = new SoTranslation;
      cpLoc[i][j]->translation.setValue(zero);
      sep->addChild(cpLoc[i][j]);
      sep->addChild(sphere[i]);
      segGroup->addChild(sep);
  
      sep = new SoSeparator;
      cpLoc[j][i] = new SoTranslation;
      cpLoc[j][i]->translation.setValue(zero);
      sep->addChild(cpLoc[j][i]);
      sep->addChild(sphere[j]);
      segGroup->addChild(sep);

      cpSegCoords[i][j] = new SoCoordinate3;
      cpSegCoords[i][j]->point.setNum(2);
      cpSegCoords[i][j]->point.set1Value(0, zero);
      cpSegCoords[i][j]->point.set1Value(1, zero);
      segGroup->addChild(cpSegCoords[i][j]);
  
      SoLineSet *line = new SoLineSet;
      line->numVertices.setValue(2);
      segGroup->addChild(line);

      root->addChild(cpSegSwitch[i][j]);
    }

  updatePolys(NULL, NULL);
}


