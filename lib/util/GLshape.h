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
    void draw();
    void setVertices(int nvertices, const float *vertices);
    void setTriangles(int ntriangles, const long int *vertexIndices);
    void setNormals(int nnormal, const float *normals);
    void setNormalIndices(int len, const long int *normalIndices);
    void setDiffuseColor(float r, float g, float b, float a);
    void setSpecularColor(float r, float g, float b);
    void setShininess(float s);
    void setNormalPerVertex(bool flag);
    void setTextureCoordinates(int len, const float *coordinates);
    void setTextureCoordIndices(int len, const long int *coordinates);
    void setTexture(GLtexture *texture);
    void compile();
private:
    void doCompile();

    std::vector<Eigen::Vector3f> m_vertices, m_normals;
    std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > m_textureCoordinates;
    std::vector<Eigen::Vector3i> m_triangles;
    Eigen::VectorXi m_normalIndices, m_textureCoordIndices;
    float m_diffuse[4], m_specular[4], m_shininess;
    bool m_normalPerVertex;
    GLtexture *m_texture;
    bool m_requestCompile;
    int m_list;
    GLuint m_textureId;
};

#endif
