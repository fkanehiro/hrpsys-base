# This Makefile builds two example programs that use Inventor.

CFLAGS=  -O2 -I ../include -DINVENTOR -woff 3262,3666
LDFLAGS= -L ../lib -lvclipi -lInventor -lm 

all:	inventorize vcdemo

inventorize: 
	CC $(CFLAGS) inventorize.C $(LDFLAGS) -o inventorize
	rm -r ii_files

vcdemo: 
	CC $(CFLAGS) vcdemo.C main.C Hpaned.C HpanedUI.C VkwindowMainWindow.C  \
	$(LDFLAGS) -lInventorXt -lvkEZ -lvk -lvkhelp -lSgm \
	-lXpm -lXm -lXt -lX11 -lgen -o vcdemo
	rm -r ii_files

clean:
	-rm inventorize
	-rm vcdemo
	-rm -r ii_files
	-rm -r *.o
