#include <iostream>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include "GLutil.h"

using namespace OpenHRP;

void compileShape(OpenHRP::BodyInfo_var i_binfo,
                  const TransformedShapeIndexSequence& tsis)
{
    ShapeInfoSequence_var sis = i_binfo->shapes();
    AppearanceInfoSequence_var ais = i_binfo->appearances();
    MaterialInfoSequence_var mis = i_binfo->materials();
    for (unsigned int l=0; l<tsis.length(); l++){
        const TransformedShapeIndex &tsi = tsis[l];
        double tform[16];
        for (int i=0; i<3; i++){
            for (int j=0; j<4; j++){
                tform[j*4+i] = tsi.transformMatrix[i*4+j];
            }
        }
        tform[3] = tform[7] = tform[11] = 0.0; tform[15] = 1.0;
        
        glPushMatrix();
        glMultMatrixd(tform);
        //printMatrix(tform);
        
        glBegin(GL_TRIANGLES);
        short index = tsi.shapeIndex;
        ShapeInfo& si = sis[index];
        const float *vertices = si.vertices.get_buffer();
        const LongSequence& triangles = si.triangles;
        const AppearanceInfo& ai = ais[si.appearanceIndex];
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
                if (p < ai.normals.length()) glNormal3fv(normals+p);
            }
            for(int k=0; k < 3; ++k){
                if (ai.normalPerVertex){
                    int p = normalIndices[j*3+k]*3;
                    glNormal3fv(normals+p);
                }
                long orgVertexIndex = si.triangles[j * 3 + k];
                int p = orgVertexIndex * 3;
                glVertex3fv(vertices+p);
            }
        }
        glEnd();
        glPopMatrix();
    }
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

