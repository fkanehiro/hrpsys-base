#include <iostream.h>
#include <fstream.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/nodekits/SoNodeKit.h>
#include <Inventor/actions/SoWriteAction.h>

#include "vclip.h"

// Read in the PolyTree file, generate an inventor model of its final
// PolyTree, write the model to standard output

int main(int argc, char **argv)
{
  SoNode *model;
  PolyTreeLibrary polyTreeLibrary;

  if (argc != 2) {
    cout << "usage:  inventorize <PolyTree file>" << endl;
    return 1;
  }

  loadPolyTreeFile(argv[1], polyTreeLibrary);

  SoDB::init();
  SoNodeKit::init();

  // last PolyTree read in is at position 0 in the library
  model = polyTreeLibrary.lookup(0)->buildInvModel();


  SoWriteAction wa;  // default output stream is cout
  wa.getOutput()->setBinary(FALSE);  // write out in ascii format
  wa.getOutput()->setFloatPrecision(9);
  wa.apply(model);
  return 0;
}
