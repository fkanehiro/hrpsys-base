#include "GLutil.h"
#include "GLshape.h"

GLshape::GLshape() : m_list(0)
{
    for (int i=0; i<16; i++) m_trans[i] = 0.0;
    m_trans[0] = m_trans[5] = m_trans[10] = m_trans[15] = 1.0;
}

GLshape::~GLshape()
{
    for (unsigned int i=0; i<m_textures.size(); i++){
        glDeleteTextures(1, &m_textures[i]);
    }
    if (m_list) glDeleteLists(m_list, 1);
}

void GLshape::setDrawInfo(OpenHRP::ShapeSetInfo_ptr i_ssinfo, 
                          const OpenHRP::TransformedShapeIndexSequence &i_tsis)
{
    m_list = glGenLists(1);
    glNewList(m_list, GL_COMPILE);
    m_textures = compileShape(i_ssinfo, i_tsis);
    glEndList();
}

void GLshape::draw()
{
    glPushMatrix();
    glMultMatrixd(m_trans);
    glCallList(m_list);
    glPopMatrix();
}

