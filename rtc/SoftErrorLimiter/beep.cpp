#include <stdio.h>
#include "beep.h"

/* beep function */
static FILE *console_fp;
void init_beep() {
  /* try to snag the console */
  if(! (console_fp = fopen("/dev/console", "w"))) {
    fprintf(stderr, ";;\n;; Could not open /dev/console for writing.\n;;\n");
    perror("open");
  } else {
    fprintf(stderr, ";; Opening /dev/console for writing.\n;;\n");
  }
}

void start_beep(int freq, int length) {
  if ( console_fp && fileno(console_fp) > 0 ) {
    fprintf(console_fp, "\033[10;%d]\033[11;%d]\a\n", freq, length);
  }
}

void stop_beep() {
  if ( console_fp && fileno(console_fp) > 0 ) {
    fprintf(console_fp,"\033[10]\033[11]\n");
  }
}

void quit_beep() {
  if ( console_fp && fileno(console_fp) > 0 ) {
    fclose(console_fp);
  }
}

