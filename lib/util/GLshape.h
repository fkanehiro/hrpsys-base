#ifndef __GLSHAPE_H__
#define __GLSHAPE_H__

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <vector>
#include <boost/intrusive_ptr.hpp>
#include <hrpCorba/ModelLoader.hh>

class GLshape
{
public:
    GLshape();
    ~GLshape();
    void draw();
    void setDrawInfo(OpenHRP::ShapeSetInfo_ptr i_ssinfo, 
                     const OpenHRP::TransformedShapeIndexSequence &i_tsis);
private:
    double m_trans[16];
    int m_list;
    std::vector<GLuint> m_textures;
};

#endif
