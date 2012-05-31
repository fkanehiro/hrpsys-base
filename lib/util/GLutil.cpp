#include <iostream>
#include <math.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include <hrpUtil/Eigen3d.h>
#include "GLshape.h"
#include "GLbody.h"
#include "GLlink.h"
#include "GLcamera.h"
#include "GLtexture.h"
#include "GLutil.h"

using namespace OpenHRP;
using namespace hrp;

std::vector<GLuint> compileShape(OpenHRP::ShapeSetInfo_ptr i_binfo,
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
        const CORBA::Long *texindices;
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
#if 1
        std::cout << std::endl;
        std::cout << "normalPerVertex = " << ai.normalPerVertex << std::endl;
        std::cout << "numTriangles = " << numTriangles << std::endl;
        std::cout << "numVertices = " << si.vertices.length()/3 << std::endl;
        std::cout << "length of normalIndices = " << normalIndices.length() << std::endl;
        std::cout << "length of normals = " << ai.normals.length()/3 << std::endl;
#endif
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
                long vertexIndex = si.triangles[vi];
                int p = vertexIndex * 3;
                if (ai.normalPerVertex){
                    int p;
                    if (normalIndices.length() == 0){
                        p = vertexIndex*3;
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

bool loadTextureFromTextureInfo(GLtexture *texture, TextureInfo &ti)
{
    texture->repeatS = ti.repeatS;
    texture->repeatT = ti.repeatT;
    texture->numComponents = ti.numComponents;
    if (texture->numComponents != 3 && texture->numComponents != 4){
        std::cerr << "texture image which has "
                  << texture->numComponents 
                  << " components is not supported"
                  << std::endl;
        std::cerr << "url = " << texture->url << std::endl;
        return false;
    }
    texture->url = ti.url;
    texture->width = ti.width;
    texture->height = ti.height;
    texture->image.resize(ti.image.length());
    memcpy(&texture->image[0], ti.image.get_buffer(), ti.image.length());
    return true;
}

void loadShape(GLshape *shape, 
               OpenHRP::ShapeSetInfo_ptr i_ssinfo, 
               const OpenHRP::TransformedShapeIndex &i_tsi)
{
    ShapeInfoSequence_var sis = i_ssinfo->shapes();
    AppearanceInfoSequence_var ais = i_ssinfo->appearances();
    MaterialInfoSequence_var mis = i_ssinfo->materials();
    shape->setTransform(i_tsi.transformMatrix);
    ShapeInfo& si = sis[i_tsi.shapeIndex];
    shape->setVertices(si.vertices.length()/3, si.vertices.get_buffer());
    shape->setTriangles(si.triangles.length()/3, 
                        (int *)si.triangles.get_buffer());
    const AppearanceInfo& ai = ais[si.appearanceIndex];
    shape->setNormals(ai.normals.length()/3, ai.normals.get_buffer());
    shape->setNormalIndices(ai.normalIndices.length(), 
                            (int *)ai.normalIndices.get_buffer()); 
    shape->setTextureCoordinates(ai.textureCoordinate.length()/2, 
                                 ai.textureCoordinate.get_buffer());
    shape->setTextureCoordIndices(ai.textureCoordIndices.length(),
                                  (int *)ai.textureCoordIndices.get_buffer());
    if (ai.textureIndex >=0){
        if (i_ssinfo->textures()->length() <= ai.textureIndex){
            std::cerr << "invalid texture index(" << ai.textureIndex << ")"
                      << std::endl;
        }else{
            TextureInfo &ti = (*i_ssinfo->textures())[ai.textureIndex];
            GLtexture *texture = new GLtexture();
            if (loadTextureFromTextureInfo(texture, ti)){
                shape->setTexture(texture);
            }else{
                delete texture;
            }
        }
    }
    if (ai.colors.length()){
        shape->setDiffuseColor(ai.colors[0], ai.colors[1], ai.colors[2], 1.0);
    }else if (ai.materialIndex >= 0){ 
        const MaterialInfo& mi = mis[ai.materialIndex];
        shape->setDiffuseColor(mi.diffuseColor[0], mi.diffuseColor[1], mi.diffuseColor[2], 1.0-mi.transparency);
        shape->setShininess(mi.shininess);
        shape->setSpecularColor(mi.specularColor[0], mi.specularColor[1], mi.specularColor[2]);
    }else{
        std::cout << "no material" << std::endl;
    }
    shape->normalPerVertex(ai.normalPerVertex);
    shape->solid(ai.solid);
    shape->compile();
}


void loadCube(GLshape *shape, double x, double y, double z)
{
    double hx = x/2, hy = y/2, hz = z/2;
    float vertices[] = {hx,hy,hz,
                        -hx,hy,hz,
                        -hx,-hy,hz,
                        hx,-hy,hz,
                        hx,hy,-hz,
                        -hx,hy,-hz,
                        -hx,-hy,-hz,
                        hx,-hy,-hz};
    int triangles[] = {0,2,3,//+z
                       0,1,2,//+z
                       4,3,7,//+x
                       4,0,3,//+x
                       0,4,5,//+y
                       5,1,0,//+y
                       1,5,6,//-x
                       1,6,2,//-x
                       2,6,7,//-y
                       2,7,3,//-y
                       7,6,4,//-z
                       5,4,6};//-z
    float normals[] = {1,0,0,
                       0,1,0,
                       0,0,1,
                       -1,0,0,
                       0,-1,0,
                       0,0,-1};
    int normalIndices[] = {2,2,0,0,1,1,3,3,4,4,5,5};
    shape->setVertices(8, vertices);
    shape->setTriangles(12, triangles);
    shape->setNormals(6, normals);
    shape->setNormalIndices(12, normalIndices);
    shape->setDiffuseColor(0.8, 0.8, 0.8, 1.0);
    shape->normalPerVertex(false);
    shape->solid(true);
    shape->compile();
}

void loadShapeFromBodyInfo(GLbody *body, BodyInfo_var i_binfo,
                           GLshape *(*shapeFactory)())
{
    LinkInfoSequence_var lis = i_binfo->links();
    
    for (unsigned int i=0; i<lis->length(); i++){
        hrp::Link *l = body->link(std::string(lis[i].name));
        if (l){
            loadShapeFromLinkInfo((GLlink *)l, lis[i], i_binfo, shapeFactory);
        }else{
            std::cout << "can't find a link named " << lis[i].name 
                      << std::endl;
        }
    }
}

void loadShapeFromSceneInfo(GLlink *link, SceneInfo_var i_sinfo, 
                            GLshape *(*shapeFactory)())
{
    TransformedShapeIndexSequence_var tsis = i_sinfo->shapeIndices();
    for (size_t i = 0; i<tsis->length(); i++){
        GLshape *shape = shapeFactory ? shapeFactory() : new GLshape();
        loadShape(shape, i_sinfo, tsis[i]);
        link->addShape(shape);
    }
}

void loadShapeFromLinkInfo(GLlink *link, const LinkInfo &i_li, ShapeSetInfo_ptr i_ssinfo, GLshape *(*shapeFactory)()){
    Vector3 axis;
    Matrix33 R;
    
    for (int i=0; i<3; i++) axis[i] = i_li.rotation[i];
    hrp::calcRodrigues(R, axis, i_li.rotation[3]);

    double trans[16];
    trans[ 0]=R(0,0);trans[ 1]=R(0,1);trans[ 2]=R(0,2);trans[3]=i_li.translation[0]; 
    trans[ 4]=R(1,0);trans[ 5]=R(1,1);trans[ 6]=R(1,2);trans[7]=i_li.translation[1]; 
    trans[ 8]=R(2,0);trans[ 9]=R(2,1);trans[10]=R(2,2);trans[11]=i_li.translation[2]; 
    link->setTransform(trans);
    link->setQ(0);
    link->computeAbsTransform();

    for (size_t i = 0; i<i_li.shapeIndices.length(); i++){
        GLshape *shape = shapeFactory ? shapeFactory() : new GLshape();
        loadShape(shape, i_ssinfo, i_li.shapeIndices[i]);
        link->addShape(shape);
    }

    const SensorInfoSequence& sensors = i_li.sensors;
    for (unsigned int i=0; i<sensors.length(); i++){
        const SensorInfo& si = sensors[i];
        std::string type(si.type);
        if (type == "Vision"){
            //std::cout << si.name << std::endl;
            link->addCamera(new GLcamera(si,i_ssinfo, link));
        }else{
            Vector3 p;
            p[0] = si.translation[0];
            p[1] = si.translation[1];
            p[2] = si.translation[2];
            Matrix33 R;
            Vector3 axis;
            axis[0] = si.rotation[0];
            axis[1] = si.rotation[1];
            axis[2] = si.rotation[2];
            hrp::calcRodrigues(R, axis, si.rotation[3]);
    
            for (size_t i=0; i<si.shapeIndices.length(); i++){
                GLshape *shape = shapeFactory ? shapeFactory() : new GLshape();
                loadShape(shape, i_ssinfo, si.shapeIndices[i]);
                Vector3 newp = R*shape->getPosition()+p;
                shape->setPosition(newp[0], newp[1], newp[2]);
                Matrix33 newR = R*shape->getRotation();
                shape->setRotation(newR);
                link->addShape(shape);
            }
        }
    }
}

