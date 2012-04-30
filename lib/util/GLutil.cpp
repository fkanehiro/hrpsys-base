#include <iostream>
#include <math.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include "GLutil.h"

using namespace OpenHRP;

std::vector<GLuint> compileShape(OpenHRP::BodyInfo_var i_binfo,
                              const TransformedShapeIndexSequence& tsis)
{
    std::vector<GLuint> textures;
    ShapeInfoSequence_var sis = i_binfo->shapes();
    AppearanceInfoSequence_var ais = i_binfo->appearances();
    MaterialInfoSequence_var mis = i_binfo->materials();
    for (unsigned int l=0; l<tsis.length(); l++){
        const TransformedShapeIndex &tsi = tsis[l];
        double tform[16], scale[3];
        for (int i=0; i<3; i++){
            for (int j=0; j<4; j++){
                tform[j*4+i] = tsi.transformMatrix[i*4+j];
            }
            scale[i] = sqrt(tform[i]*tform[i]+tform[i+4]*tform[i+4]+tform[i+8]*tform[i+8]);
        }
        tform[3] = tform[7] = tform[11] = 0.0; tform[15] = 1.0;
        
        glPushMatrix();
        glMultMatrixd(tform);
        //printMatrix(tform);
        
        short index = tsi.shapeIndex;
        ShapeInfo& si = sis[index];
        const float *vertices = si.vertices.get_buffer();
        const LongSequence& triangles = si.triangles;
        const AppearanceInfo& ai = ais[si.appearanceIndex];
        const float *texcoord=NULL;
        const long int *texindices;
        if (ai.textureIndex >=0){
            TextureInfo &ti = (*i_binfo->textures())[ai.textureIndex];
            if (ti.image.length()==0){
                std::cerr<< "texture image(" << ti.url << ") is not loaded"
                         << std::endl;
            }else{
                texcoord = ai.textureCoordinate.get_buffer();
                texindices = ai.textureCoordIndices.get_buffer();
                GLuint tex;
                glGenTextures(1, &tex);
                textures.push_back(tex);
                glBindTexture(GL_TEXTURE_2D, tex);
                
                if (ti.repeatS){
                    glTexParameteri(GL_TEXTURE_2D, 
                                    GL_TEXTURE_WRAP_S, GL_REPEAT);
                }else{
                    glTexParameteri(GL_TEXTURE_2D, 
                                    GL_TEXTURE_WRAP_S, GL_CLAMP);
                }
                if (ti.repeatT){
                    glTexParameteri(GL_TEXTURE_2D,
                                    GL_TEXTURE_WRAP_T, GL_REPEAT);
                }else{
                    glTexParameteri(GL_TEXTURE_2D,
                                    GL_TEXTURE_WRAP_T, GL_CLAMP);
                }
                int format;
                if (ti.numComponents == 3){
                    format = GL_RGB;
                }else if (ti.numComponents == 4){
                    format = GL_RGBA;
                }else{
                    std::cerr << "texture image which has "
                              << ti.numComponents << " is not supported"
                              << std::endl;
                }
                glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
                gluBuild2DMipmaps(GL_TEXTURE_2D, 3, ti.width, ti.height, 
                                  format, GL_UNSIGNED_BYTE, ti.image.get_buffer());

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, 
                                GL_LINEAR_MIPMAP_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, 
                                GL_LINEAR);
                glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

                glEnable(GL_TEXTURE_2D);
            }
        }
        glBegin(GL_TRIANGLES);
        const float *normals = ai.normals.get_buffer();
        //std::cout << "length of normals = " << ai.normals.length() << std::endl;
        const LongSequence& normalIndices = ai.normalIndices;
        //std::cout << "length of normalIndices = " << normalIndices.length() << std::endl;
        const int numTriangles = triangles.length() / 3;
        //std::cout << "numTriangles = " << numTriangles << std::endl;
        if (ai.colors.length()){
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, 
                         ai.colors.get_buffer());
        }else if (ai.materialIndex >= 0){ 
            const MaterialInfo& mi = mis[ai.materialIndex];
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, 
                         mi.diffuseColor);
        }else{
            std::cout << "no material" << std::endl;
        }

        for(int j=0; j < numTriangles; ++j){
            if (!ai.normalPerVertex){
                int p;
                if (normalIndices.length() == 0){
                    p = j*3;
                }else{
                    p = normalIndices[j]*3;
                }
                if (p < ai.normals.length()){
                    glNormal3f(normals[p  ]*scale[0],
                               normals[p+1]*scale[1],
                               normals[p+2]*scale[2]);
                }
            }
            for(int k=0; k < 3; ++k){
                int vi = j*3+k;
                if (ai.normalPerVertex){
                    int p;
                    if (normalIndices.length() == 0){
                        p = vi*3;
                    }else{
                        p = normalIndices[vi]*3;
                    }
                    glNormal3f(normals[p  ]*scale[0],
                               normals[p+1]*scale[1],
                               normals[p+2]*scale[2]);
                }
                if (texcoord){
                    glTexCoord2d(texcoord[texindices[vi]*2],
                                 -texcoord[texindices[vi]*2+1]);
#if 0
                    std::cout << "tri:" << j << ", index:" 
                              << texindices[j*3+k] << "," 
                              << texcoord[texindices[vi]*2]  << ","
                              << texcoord[texindices[vi]*2+1] << std::endl;
#endif
                }
                long vertexIndex = si.triangles[vi];
                int p = vertexIndex * 3;
                glVertex3fv(vertices+p);
            }
        }
        glEnd();
        if (texcoord) glDisable(GL_TEXTURE_2D);
        glPopMatrix();
    }
    return textures;
}

void mulTrans(const double i_m1[16], const double i_m2[16], double o_m[16])
{
    for (int i=0; i<4; i++){
        for (int j=0; j<4;j++){
            double v = 0;
            for (int k=0; k<4; k++){
                v += i_m1[i*4+k]*i_m2[j+k*4];
            }
            o_m[i*4+j] = v;
        }
    }
}

