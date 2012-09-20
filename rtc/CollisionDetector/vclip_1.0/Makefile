# This Makefile builds all components of the V-Clip distribution.

# To build, follow these two steps, and then make.  To rebuild, make
# clean, and then make.


# STEP 1.  Select targets, based on whether SGI Inventor libraries are
# available.
#
#	If so:
#
all:	lib/libvclipi.a lib/libvclip.a exmpls iv_exmpls
#
#	If not:
#
#all:	lib/libvclip.a exmpls
#
# If the Inventor libraries are not available, the examples in directory
# iv_examples will not be built.


# STEP 2.  If the Qhull library is available, leave these three lines
# uncommented, and point to the Qhull directory on the first line.  If
# the Qhull library is not available, comment out these three lines.
# (The qhull directory should contain libqhull.a and several header
# files such as qhull_a.h.)
#
QHULL_DIR  = qhull
QHULL_CC   = -DQHULL -I $(QHULL_DIR)
QHULL_LD   = -L $(QHULL_DIR) -lqhull
# 
# The V-Clip library and the example programs do not require Qhull,
# but it is needed to define compound PolyTrees and to compute faces of
# atomic PolyTrees automatically.  These operations will generate run
# time errors if Qhull is not available.




OPTIMIZER= -O2

CFLAGS= $(OPTIMIZER) -I include $(QHULL_CC)


# Note: the edg_prelink commands below are for instantiating static
# template variables in the V-Clip library, under SGI's 7.1 C++
# compiler for IRIX 6.2.  The ii_files directory is also related to
# this compiler's template mechanism.

lib/libvclip.a:
	CC $(CFLAGS) -c src/mv.C 
	CC $(CFLAGS) -c src/PolyTree.C -woff 3271,3666
	CC $(CFLAGS) -c src/vclip.C 
	/usr/lib/DCC/edg_prelink PolyTree.o vclip.o
	ld -r vclip.o PolyTree.o mv.o $(QHULL_LD) -o lib/libvclip.a
	rm -r *.o ii_files

lib/libvclipi.a:
	CC $(CFLAGS) -DINVENTOR -c src/mv.C 
	CC $(CFLAGS) -DINVENTOR -c src/PolyTree.C -woff 3271,3666
	CC $(CFLAGS)            -c src/vclip.C 
	/usr/lib/DCC/edg_prelink PolyTree.o vclip.o
	ld -r vclip.o PolyTree.o mv.o $(QHULL_LD)  -o lib/libvclipi.a
	rm -r *.o ii_files

exmpls:
	(cd examples; make)

iv_exmpls:
	(cd iv_examples; make)

clean:
	-rm *.o
	-rm lib/libvclip.a lib/libvclipi.a
	-rm -r ii_files
	(cd examples; make clean)
	(cd iv_examples; make clean)
