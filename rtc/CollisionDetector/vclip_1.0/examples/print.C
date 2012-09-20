#include <iostream.h>
#include <fstream.h>

#include "vclip.h"

// Read in the PolyTree file specified on the command line;
// print out the generated PolyTree models.

int main(int argc, char **argv)
{
  int i;
  PolyTreeLibrary polyTreeLibrary;

  if (argc != 2) {
    cout << "usage:  print <PolyTree file>" << endl;
    return 1;
  }

  loadPolyTreeFile(argv[1], polyTreeLibrary);

  // print out all PolyTrees in library
  for (i = 0; i < polyTreeLibrary.size(); i++) 
    cout << "\n\n" << polyTreeLibrary.lookup(i) << endl;

  return 0;
}
