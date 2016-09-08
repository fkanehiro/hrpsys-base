#include <iostream>
extern "C" {
#include <qhull/qhull_a.h>
}
#include <hrpModel/Link.h>
#include "BVutil.h"

using namespace hrp;

void convertToAABB(hrp::BodyPtr i_body)
{
    for (unsigned int i=0; i<i_body->numLinks(); i++) convertToAABB(i_body->link(i));
}

void convertToAABB(hrp::Link *i_link)
{
    if (!i_link->coldetModel || !i_link->coldetModel->getNumVertices()) return;

    int ptype = i_link->coldetModel->getPrimitiveType();
    if (ptype == ColdetModel::SP_PLANE || ptype == ColdetModel::SP_SPHERE){
        return;
    }

    std::vector<Vector3> boundingBoxData;
    i_link->coldetModel->getBoundingBoxData(0, boundingBoxData);
    if (boundingBoxData.size() != 2){
        std::cerr << "unexpected bounding box data size(" << i_link->name 
                  << ", " << boundingBoxData.size() << ")" << std::endl;
        return;
    }
    const Vector3 &c = boundingBoxData[0];
    const Vector3 &s = boundingBoxData[1];
    ColdetModelPtr coldetModel(new ColdetModel());
    coldetModel->setName(std::string(i_link->name.c_str()));
    coldetModel->setPrimitiveType(ColdetModel::SP_BOX);
    coldetModel->setNumPrimitiveParams(3);
    for (int i=0; i<3; i++){
        coldetModel->setPrimitiveParam(i, s[i]);
    }
    double R[] = {1,0,0,0,1,0,0,0,1};
    coldetModel->setPrimitivePosition(R, c.data());
    coldetModel->setNumVertices(8);
    coldetModel->setVertex(0, c[0]+s[0], c[1]+s[1], c[2]+s[2]);
    coldetModel->setVertex(1, c[0]-s[0], c[1]+s[1], c[2]+s[2]);
    coldetModel->setVertex(2, c[0]-s[0], c[1]-s[1], c[2]+s[2]);
    coldetModel->setVertex(3, c[0]+s[0], c[1]-s[1], c[2]+s[2]);
    coldetModel->setVertex(4, c[0]+s[0], c[1]+s[1], c[2]-s[2]);
    coldetModel->setVertex(5, c[0]-s[0], c[1]+s[1], c[2]-s[2]);
    coldetModel->setVertex(6, c[0]-s[0], c[1]-s[1], c[2]-s[2]);
    coldetModel->setVertex(7, c[0]+s[0], c[1]-s[1], c[2]-s[2]);
    int numTriangles = 12;
    coldetModel->setNumTriangles(numTriangles);
    int triangles[] = {0,2,3,
                       0,1,2,
                       4,3,7,
                       4,0,3,
                       0,4,5,
                       5,1,0,
                       1,5,6,
                       1,6,2,
                       2,6,7,
                       2,7,3,
                       7,6,4,
                       5,4,6};
    for(int j=0; j < numTriangles; ++j){
        int t0 = triangles[j*3];
        int t1 = triangles[j*3+1];
        int t2 = triangles[j*3+2];
        coldetModel->setTriangle(j, t0, t1, t2);
    }
    coldetModel->build();
    i_link->coldetModel = coldetModel;
}

void convertToConvexHull(hrp::BodyPtr i_body)
{
    for (unsigned int i=0; i<i_body->numLinks(); i++){
        convertToConvexHull(i_body->link(i));
    }
}

void convertToConvexHull(hrp::Link *i_link)
{
    if (!i_link->coldetModel || !i_link->coldetModel->getNumVertices()) return;

    int ptype = i_link->coldetModel->getPrimitiveType();
    if (ptype == ColdetModel::SP_PLANE || ptype == ColdetModel::SP_SPHERE){
        return;
    }

    ColdetModelPtr coldetModel(new ColdetModel());
    coldetModel->setName(i_link->name.c_str());
    coldetModel->setPrimitiveType(ColdetModel::SP_MESH);
    // qhull
    int numVertices = i_link->coldetModel->getNumVertices();
    double points[numVertices*3];
    float v[3];
    for (int i=0; i<numVertices; i++){
        i_link->coldetModel->getVertex(i, v[0],v[1],v[2]);
        points[i*3+0] = v[0];
        points[i*3+1] = v[1];
        points[i*3+2] = v[2];
    }
    char flags[250];
    boolT ismalloc = False;
    sprintf(flags,"qhull Qt Tc");
    if (qh_new_qhull (3,numVertices,points,ismalloc,flags,NULL,stderr)) return;

    qh_triangulate();
    qh_vertexneighbors();
    
    coldetModel->setNumVertices(qh num_vertices);
    coldetModel->setNumTriangles(qh num_facets);
    int index[numVertices];
    int vertexIndex = 0;
    vertexT *vertex;
    FORALLvertices {
        int p = qh_pointid(vertex->point);
        index[p] = vertexIndex;
        coldetModel->setVertex(vertexIndex++, points[p*3+0], points[p*3+1], points[p*3+2]);
    }
    facetT *facet;
    int num = qh num_facets;
    int triangleIndex = 0;
    FORALLfacets {
        int j = 0, p[3];
        setT *vertices = qh_facet3vertex (facet);
        vertexT **vertexp;
        FOREACHvertexreverse12_ (vertices) {
            if (j<3) {
                p[j] = index[qh_pointid(vertex->point)];
            } else {
                fprintf(stderr, "extra vertex %d\n",j);
            }
            j++;
        }
        coldetModel->setTriangle(triangleIndex++, p[0], p[1], p[2]);
    }

    coldetModel->build();
    i_link->coldetModel =  coldetModel;
    
    qh_freeqhull(!qh_ALL);
    int curlong, totlong;
    qh_memfreeshort (&curlong, &totlong);
    if (curlong || totlong) {
        fprintf(stderr, "convhulln: did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
    }
    //
    //std::cerr << i_link->name << " reduce triangles from " << numTriangles << " to " << num << std::endl;
}
