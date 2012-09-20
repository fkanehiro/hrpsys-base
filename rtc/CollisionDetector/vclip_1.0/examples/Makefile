# This Makefile builds two example programs that do not use Inventor.

CFLAGS=  -O2 -I ../include
LDFLAGS= -L ../lib -lvclip -lm

all:	print allpairs

print:
	CC $(CFLAGS) print.C $(LDFLAGS) -o print
	-rm -r ii_files

allpairs:
	CC $(CFLAGS) allpairs.C $(LDFLAGS) -o allpairs
	-rm -r ii_files

clean:
	-rm print
	-rm allpairs
	-rm -r ii_files
