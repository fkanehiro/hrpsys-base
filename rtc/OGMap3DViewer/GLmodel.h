#include <hrpCorba/ModelLoader.hh>
#include <hrpUtil/Tvmet3d.h>
#include <vector>

class GLlink;

class GLcamera
{
public:
    GLcamera(const OpenHRP::SensorInfo &i_si, GLlink *i_link);
    GLcamera(int i_width, int i_height, double i_near, double i_far, double i_fovy);
    const std::string& name() const;
    void setView();
    void computeAbsTransform(double o_trans[16]);
    void setTransform(double i_trans[16]);
    void getAbsTransform(double o_trans[16]);
    double near() { return m_near; }
    double far() { return m_far; }
    double fovy() { return m_fovy; }
    int width() { return m_width; }
    int height() { return m_height; }
    void getDepthOfLine(int i_row, float *o_depth);
private:
    std::string m_name;
    double m_trans[16], m_absTrans[16];
    GLlink *m_link;
    double m_near, m_far, m_fovy;
    int m_width, m_height;
};

class GLlink
{
public:
    GLlink(const OpenHRP::LinkInfo &i_li, OpenHRP::BodyInfo_var i_binfo);

    void draw();

    void setParent(GLlink *i_parent);

    void addChild(GLlink *i_child);

    void setQ(double i_q);

    void setTransform(double i_trans[16]);

    int jointId();

    GLcamera *findCamera(const char *i_name);

    void computeAbsTransform(double o_trans[16]);

private:
    GLlink *m_parent;
    std::vector<GLlink *> m_children;
    std::vector<GLcamera *> m_cameras;
    hrp::Vector3 m_axis;
    double m_trans[16], m_T_j[16];
    int m_list, m_jointId;
};

class GLbody
{
public:
    GLbody(OpenHRP::BodyInfo_var i_binfo);

    ~GLbody();

    void setPosture(double *i_angles, double *i_pos, double *i_rpy);

    void draw();

    GLcamera *findCamera(const char *i_name);
private:
    GLlink *m_root;
    std::vector<GLlink *> m_links;
};

class GLscene
{
public:
    void addBody(GLbody *i_body);
    unsigned int numBodies() const;
    GLbody *body(unsigned int i_rank);
    void draw(bool swap=true);
    void save(const char *i_fname);
    void capture(unsigned char *o_image);
    void init();
    void setCamera(GLcamera *i_camera);
    GLcamera *getCamera();

    static GLscene *getInstance();
private:
    GLscene();
    ~GLscene();

    static GLscene *m_scene;
    std::vector<GLbody *> m_bodies; 
    GLcamera *m_camera, *m_default_camera;
};

void mulTrans(const double i_m1[16], const double i_m2[16], double o_m[16]);
void printMatrix(double mat[16]);
