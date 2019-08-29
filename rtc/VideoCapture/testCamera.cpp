#include "camera.h"
#include <fstream>
#include <iostream>

int main(int argc, char *argv[])
{
    int w=640, h=480, n=1;
    for (int i=1; i<argc; i++){
        if (strcmp(argv[i], "-w") == 0){
            w = atoi(argv[++i]);
        }else if (strcmp(argv[i], "-h") == 0){
            h = atoi(argv[++i]);
        }else if (strcmp(argv[i], "-n") == 0){
            n = atoi(argv[++i]);
        }
    }

    v4l_capture cam;

    if (cam.init(w, h, n) != 0){
        std::cerr << "failed to initialize device(/dev/video" << n << ")" 
                  << std::endl;
        return 1;
    }

    uchar *img = cam.capture();

    std::ofstream ofs("test.ppm");
    ofs << "P6" << std::endl;
    ofs << w << " " << h << std::endl;
    ofs << "255" << std::endl;
    ofs.write((const char *)img, w*h*3); 

    return 0;
}
