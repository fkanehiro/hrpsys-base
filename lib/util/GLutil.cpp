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

class shapeLoader{
public:
    void setShapeSetInfo(ShapeSetInfo_ptr i_ssinfo);
    void loadShapeFromBodyInfo(GLbody *body, BodyInfo_var i_binfo, GLshape *(*shapeFactory)());
    void loadShapeFromLinkInfo(GLlink *link, const LinkInfo &i_li, GLshape *(*shapeFactory)());
    void loadShape(GLshape *shape, 
                   const OpenHRP::TransformedShapeIndex &i_tsi);
    void loadCamera(GLcamera *camera, const OpenHRP::SensorInfo &i_si); 
    ShapeInfoSequence_var sis;
    AppearanceInfoSequence_var ais;
    MaterialInfoSequence_var mis;
    TextureInfoSequence_var txs;
};

void shapeLoader::setShapeSetInfo(ShapeSetInfo_ptr i_ssinfo)
{
    sis = i_ssinfo->shapes();
    ais = i_ssinfo->appearances();
    mis = i_ssinfo->materials(); 
    txs = i_ssinfo->textures();
}

void shapeLoader::loadShapeFromBodyInfo(GLbody *body, BodyInfo_var i_binfo,
                                        GLshape *(*shapeFactory)())
{
    LinkInfoSequence_var lis = i_binfo->links();
    for (unsigned int i=0; i<lis->length(); i++){
        hrp::Link *l = body->link(std::string(lis[i].name));
        if (l){
	    loadShapeFromLinkInfo((GLlink *)l, lis[i], shapeFactory);
        }else{
            std::cout << "can't find a link named " << lis[i].name 
                      << std::endl;
        }
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

bool loadTextureFromTextureInfo(GLtexture *texture, TextureInfo &ti)
{
    if (ti.image.length() == 0){
        std::cerr << "texture image is not loaded(" << ti.url << ")" << std::endl;
        return false;
    }else if (ti.numComponents != 3 && ti.numComponents != 4){
        std::cerr << "texture image which has "
                  << ti.numComponents 
                  << " components is not supported(" << ti.url << ")"
                  << std::endl;
        return false;
    }
    texture->repeatS = ti.repeatS;
    texture->repeatT = ti.repeatT;
    texture->numComponents = ti.numComponents;
    texture->url = ti.url;
    texture->width = ti.width;
    texture->height = ti.height;
    texture->image.resize(ti.image.length());
    memcpy(&texture->image[0], ti.image.get_buffer(), ti.image.length());
    return true;
}

void shapeLoader::loadShape(GLshape *shape, 
                            const OpenHRP::TransformedShapeIndex &i_tsi)
{
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
    shape->setColors(ai.colors.length()/3, ai.colors.get_buffer());
    if (ai.textureIndex >=0){
        if (txs->length() <= (unsigned int)ai.textureIndex){
            std::cerr << "invalid texture index(" << ai.textureIndex << ")"
                      << std::endl;
        }else{
            TextureInfo &ti = txs[ai.textureIndex];
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
        //std::cout << "no material" << std::endl;
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
    shapeLoader loader;
    loader.setShapeSetInfo(i_binfo);
    loader.loadShapeFromBodyInfo(body, i_binfo, shapeFactory);
}

void loadShapeFromSceneInfo(GLlink *link, SceneInfo_var i_sinfo, 
                            GLshape *(*shapeFactory)())
{
    shapeLoader loader;
    loader.setShapeSetInfo(i_sinfo);
    TransformedShapeIndexSequence_var tsis = i_sinfo->shapeIndices();
    for (size_t i = 0; i<tsis->length(); i++){
        GLshape *shape = shapeFactory ? shapeFactory() : new GLshape();
        loader.loadShape(shape, tsis[i]);
        link->addShape(shape);
    }
}

void loadShapeFromLinkInfo(GLlink *link, 
                           const OpenHRP::LinkInfo &i_li, 
                           OpenHRP::ShapeSetInfo_ptr i_ssinfo,
                           GLshape *(*shapeFactory)())
{
    shapeLoader loader;
    loader.setShapeSetInfo(i_ssinfo);
    loader.loadShapeFromLinkInfo(link, i_li, shapeFactory);
}

void shapeLoader::loadShapeFromLinkInfo(GLlink *link, const LinkInfo &i_li, GLshape *(*shapeFactory)()){
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
        loadShape(shape, i_li.shapeIndices[i]);
        link->addShape(shape);
    }

    const SensorInfoSequence& sensors = i_li.sensors;
    for (unsigned int i=0; i<sensors.length(); i++){
        const SensorInfo& si = sensors[i];
        std::string type(si.type);
        if (type == "Vision"){
            //std::cout << si.name << std::endl;
            GLcamera *camera = new GLcamera(si.specValues[4], si.specValues[5],
                                            si.specValues[0], si.specValues[1],
                                            si.specValues[2],
                                            link, si.id);
            loadCamera(camera, si);
            link->addCamera(camera);
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
                loadShape(shape, si.shapeIndices[i]);
                Vector3 newp = R*shape->getPosition()+p;
                shape->setPosition(newp[0], newp[1], newp[2]);
                Matrix33 newR = R*shape->getRotation();
                shape->setRotation(newR);
                link->addShape(shape);
            }
        }
    }
}

void shapeLoader::loadCamera(GLcamera *i_camera, const SensorInfo &i_si)
{
    i_camera->name(std::string(i_si.name));
    i_camera->setPosition(i_si.translation[0], i_si.translation[1],
                          i_si.translation[2]);
    i_camera->setRotation(i_si.rotation[0], i_si.rotation[1],
                          i_si.rotation[2], i_si.rotation[3]);

    for (size_t i=0; i<i_si.shapeIndices.length(); i++){
        GLshape *shape = new GLshape();
        loadShape(shape, i_si.shapeIndices[i]);
        i_camera->addShape(shape);
    }
}
