#include <iostream>
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif
#include "GLutil.h"
#include "GLlink.h"
#include "GLshape.h"
#include "GLtexture.h"

GLshape::GLshape() : m_texture(NULL), m_requestCompile(false), m_shininess(0.2), m_shadingList(0), m_wireFrameList(0), m_highlight(false)
{
    for (int i=0; i<16; i++) m_trans[i] = 0.0;
    m_trans[0] = m_trans[5] = m_trans[10] = m_trans[15] = 1.0;
    for (int i=0; i<4; i++) m_specular[i] = 0;
}

GLshape::~GLshape()
{
    if (m_texture){
        if (m_texture->image.size()) glDeleteTextures(1, &m_textureId);
        delete m_texture;
    }
    if (m_shadingList) glDeleteLists(m_shadingList, 1);
    if (m_wireFrameList) glDeleteLists(m_wireFrameList, 1);
}

void GLshape::draw(int i_mode)
{
    glPushMatrix();
    glMultMatrixd(m_trans);
    if (m_requestCompile){
        m_shadingList = doCompile(false);
        m_wireFrameList = doCompile(true);
        m_requestCompile = false;
    } 
    glCallList(i_mode == GLlink::DM_SOLID ? m_shadingList : m_wireFrameList);
    glPopMatrix();
}

void GLshape::setVertices(int nvertices, const float *vertices)
{
    m_vertices.resize(nvertices);
    for (size_t i=0; i<nvertices; i++){
        m_vertices[i] = Eigen::Vector3f(vertices[i*3  ], 
					vertices[i*3+1], 
					vertices[i*3+2]);
    }
}

void GLshape::setTriangles(int ntriangles, const int *vertexIndices)
{
    m_triangles.resize(ntriangles);
    for (size_t i=0; i<ntriangles; i++){
        m_triangles[i] = Eigen::Vector3i(vertexIndices[i*3  ],
					 vertexIndices[i*3+1],
					 vertexIndices[i*3+2]);
    }
}

void GLshape::setNormals(int nnormal, const float *normals)
{
    m_normals.resize(nnormal);
    for (size_t i=0; i<nnormal; i++){
        m_normals[i] = Eigen::Vector3f(normals[i*3  ],
				       normals[i*3+1],
				       normals[i*3+2]);
    }
}

void GLshape::setDiffuseColor(float r, float g, float b, float a)
{
    m_diffuse[0] = r; m_diffuse[1] = g; m_diffuse[2] = b; m_diffuse[3] = a; 
}

void GLshape::normalPerVertex(bool flag)
{
    m_normalPerVertex = flag;
}

void GLshape::solid(bool flag)
{
    m_solid = flag;
}

void GLshape::setNormalIndices(int len, const int *normalIndices)
{
    m_normalIndices.resize(len);
    for (size_t i=0; i<len; i++){
        m_normalIndices[i] = normalIndices[i];
    }
}

void GLshape::setTextureCoordinates(int ncoords, const float *coordinates)
{
    m_textureCoordinates.resize(ncoords);
    for (size_t i=0; i<ncoords; i++){
        m_textureCoordinates[i] = Eigen::Vector2f(coordinates[i*2  ],
						  coordinates[i*2+1]);
    }
}

void GLshape::setTextureCoordIndices(int len, const int *coordIndices)
{
    m_textureCoordIndices.resize(len);
    for (size_t i=0; i<len; i++){
        m_textureCoordIndices[i] = coordIndices[i];
    }
}

void GLshape::setTexture(GLtexture *texture)
{
    m_texture = texture;
}

void GLshape::compile()
{
    m_requestCompile = true;
}

int GLshape::doCompile(bool isWireFrameMode)
{
    if (isWireFrameMode){
        if (m_wireFrameList) glDeleteLists(m_wireFrameList, 1);
    }else{
        if (m_shadingList) glDeleteLists(m_shadingList, 1);
    }

    //std::cout << "doCompile" << std::endl;
    int list = glGenLists(1);
    glNewList(list, GL_COMPILE);

    if (m_solid){
        glEnable(GL_CULL_FACE);
    }else{
        glDisable(GL_CULL_FACE);
    }
    double scale[3];
    for (int i=0; i<3; i++){
        scale[i] = sqrt(m_trans[i]*m_trans[i]
                        +m_trans[i+4]*m_trans[i+4]
                        +m_trans[i+8]*m_trans[i+8]);
    }

    bool drawTexture = false;
    if (!isWireFrameMode && m_texture && !m_highlight){
        drawTexture = true;
        glGenTextures(1, &m_textureId);
        glBindTexture(GL_TEXTURE_2D, m_textureId);
        
        if (m_texture->repeatS){
            glTexParameteri(GL_TEXTURE_2D, 
                            GL_TEXTURE_WRAP_S, GL_REPEAT);
        }else{
            glTexParameteri(GL_TEXTURE_2D, 
                            GL_TEXTURE_WRAP_S, GL_CLAMP);
        }
        if (m_texture->repeatT){
            glTexParameteri(GL_TEXTURE_2D,
                            GL_TEXTURE_WRAP_T, GL_REPEAT);
        }else{
            glTexParameteri(GL_TEXTURE_2D,
                            GL_TEXTURE_WRAP_T, GL_CLAMP);
        }
        int format;
        if (m_texture->numComponents == 3){
            format = GL_RGB;
        }else if (m_texture->numComponents == 4){
            format = GL_RGBA;
        }
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        gluBuild2DMipmaps(GL_TEXTURE_2D, 3, 
                          m_texture->width, m_texture->height, 
                          format, GL_UNSIGNED_BYTE, 
                          &m_texture->image[0]);
        
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, 
                        GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, 
                        GL_LINEAR);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        
        glEnable(GL_TEXTURE_2D);
    }
    
    if (!isWireFrameMode) glBegin(GL_TRIANGLES);
    if (m_highlight){
        float red[] = {1,0,0,1};
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, red);
    }else{
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, m_diffuse);
        //glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            m_specular);
        glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS,           m_shininess);
    }
    for(size_t j=0; j < m_triangles.size(); ++j){
        if (isWireFrameMode) glBegin(GL_LINE_LOOP);
        if (!m_normalPerVertex){
            int p;
            if (m_normalIndices.size() == 0){
                p = j;
            }else{
                p = m_normalIndices[j];
            }
            if (p < m_normals.size()){
                const Eigen::Vector3f &n = m_normals[p];
                glNormal3f(scale[0]*n[0], scale[1]*n[1], scale[2]*n[2]);
            }
        }
        for(int k=0; k < 3; ++k){
            long vertexIndex = m_triangles[j][k];
            if (m_normalPerVertex){
                int p;
                if (m_normalIndices.size() == 0){
                    p = vertexIndex;
                }else{
                    p = m_normalIndices[j*3+k];
                }
                const Eigen::Vector3f &n = m_normals[p];
                glNormal3f(scale[0]*n[0], scale[1]*n[1], scale[2]*n[2]);
            }
            if (drawTexture){
                int texCoordIndex = m_textureCoordIndices[j*3+k];
                glTexCoord2d(m_textureCoordinates[texCoordIndex][0],
                             -m_textureCoordinates[texCoordIndex][1]);
            }
            glVertex3fv(m_vertices[vertexIndex].data());
        }
        if (isWireFrameMode) glEnd(); // GL_LINE_LOOP
    }
    if (!isWireFrameMode) glEnd(); // GL_TRIANGLES
    if (drawTexture) glDisable(GL_TEXTURE_2D);
    glEndList();

    return list;
}

void GLshape::setShininess(float s)
{
    m_shininess = s;
}

void GLshape::setSpecularColor(float r, float g, float b)
{
    m_specular[0] = r; m_specular[1] = g; m_specular[2] = b; 
}


void GLshape::highlight(bool flag)
{
    if (m_highlight != flag) compile();
    m_highlight = flag;
}
