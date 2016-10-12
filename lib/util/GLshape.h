#ifndef __GLSHAPE_H__
#define __GLSHAPE_H__

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <Eigen/Core>
#include <vector>
#include <boost/intrusive_ptr.hpp>
#include <hrpCorba/ModelLoader.hh>
#include "GLcoordinates.h"

class GLtexture;

class GLshape : public GLcoordinates
{
public:
    GLshape();
    ~GLshape();
    size_t draw(int i_mode);
    void setVertices(unsigned int nvertices, const float *vertices);
    void setTriangles(unsigned int ntriangles, const int *vertexIndices);
    void setNormals(unsigned int nnormal, const float *normals);
    void setNormalIndices(unsigned int len, const int *normalIndices);
    void setDiffuseColor(float r, float g, float b, float a);
    void setSpecularColor(float r, float g, float b);
    void setShininess(float s);
    void setColors(unsigned int ncolors, const float *colors);
    void normalPerVertex(bool flag);
    void solid(bool flag);
    void setTextureCoordinates(unsigned int len, const float *coordinates);
    void setTextureCoordIndices(unsigned int len, const int *coordinates);
    void setTexture(GLtexture *texture);
    void compile();
    void highlight(bool flag);
    void divideLargeTriangles(double maxEdgeLen);
    void computeAABB(const hrp::Vector3& i_p, const hrp::Matrix33& i_R,
                     hrp::Vector3& o_min, hrp::Vector3& o_max);
protected:
    int doCompile(bool isWireFrameMode);

    std::vector<Eigen::Vector3f> m_vertices, m_normals, m_colors;
    std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > m_textureCoordinates;
    std::vector<Eigen::Vector3i> m_triangles;
    std::vector<int> m_normalIndices, m_textureCoordIndices;
    float m_diffuse[4], m_specular[4], m_shininess;
    bool m_normalPerVertex;
    bool m_solid;
    GLtexture *m_texture;
    bool m_requestCompile;
    int m_shadingList, m_wireFrameList;
    GLuint m_textureId;
    bool m_highlight;
};

#endif
