#ifndef __GLtexture_H__
#define __GLtexture_H__

class GLtexture
{
public:
    int numComponents;
    int width, height;
    bool repeatS, repeatT;
    std::vector<unsigned char> image; 
    std::string url;
};

#endif
