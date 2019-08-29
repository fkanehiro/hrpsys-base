#include <sys/param.h>
#include <iostream>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <SDL/SDL_timer.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "hrpsys/util/GLlink.h"
#include "hrpsys/util/GLbody.h"
#include "hrpsys/util/GLutil.h"
#include "hrpsys/util/SDLUtil.h"
#include "OnlineViewer_impl.h"
#include "GLscene.h"

using namespace OpenHRP;

void print_usage(char* progname)
{
    std::cerr << "Usage:" << progname << " [model file] [options]" << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << " -size [pixels]     : specify window size in pixels" << std::endl;
    std::cerr << " -no-default-lights : disable ambient light (simulation environment will be dark)" << std::endl;
    std::cerr << " -max-edge-length [value] : specify maximum length of polygon edge (if exceed, polygon will be divided to improve rendering quality)" << std::endl;
    std::cerr << " -bg [r] [g] [b]    : specify background color" << std::endl;
    std::cerr << " -h --help          : show this help message" << std::endl;
    std::cerr << "Example:" << std::endl;
    std::cerr << " run the view server and wait for the client to connect"<< std::endl;
    std::cerr << "  $ " << progname << std::endl;
    std::cerr << " run the view server, load the pa10 robot model, set the background to green and wait for the client to connect"<< std::endl;
    std::cerr << "  $ " << progname << " /usr/share/OpenHRP-3.1/sample/model/PA10/pa10.main.wrl -bg 0 0.3 0"<< std::endl;
}

int main(int argc, char *argv[])
{
    int wsize=0;
    double maxEdgeLen=0.0;
    bool useDefaultLights=true;
    float bgColor[3]={0,0,0};
    
    for (int i=1; i<argc; i++){
        if (strcmp(argv[i], "-size")==0){
            wsize = atoi(argv[++i]);
        }else if(strcmp(argv[i], "-max-edge-length")==0){
            maxEdgeLen = atof(argv[++i]);
        }else if(strcmp(argv[i], "-no-default-lights")==0){
            useDefaultLights=false;
        }else if(strcmp(argv[i], "-bg")==0){
            bgColor[0] = atof(argv[++i]);
            bgColor[1] = atof(argv[++i]);
            bgColor[2] = atof(argv[++i]);
        }else if(strcmp(argv[i], "-h")==0 || strcmp(argv[i], "--help")==0){
            print_usage(argv[0]);
            return 1;
        }
    }
    
    CORBA::ORB_var orb = CORBA::ORB::_nil();
    
    try {
        
        orb = CORBA::ORB_init(argc, argv);
        
        CORBA::Object_var obj;
        
        obj = orb->resolve_initial_references("RootPOA");
        PortableServer::POA_var poa = PortableServer::POA::_narrow(obj);
        if(CORBA::is_nil(poa)){
            throw std::string("error: failed to narrow root POA.");
        }
        
        PortableServer::POAManager_var poaManager = poa->the_POAManager();
        if(CORBA::is_nil(poaManager)){
            throw std::string("error: failed to narrow root POA manager.");
        }
        
        LogManager<OpenHRP::WorldState> log;
        GLscene scene(&log);
        scene.setBackGroundColor(bgColor);
        scene.maxEdgeLen(maxEdgeLen);
        
        OnlineViewer_impl* OnlineViewerImpl 
            = new OnlineViewer_impl(orb, poa, &scene, &log);
        poa->activate_object(OnlineViewerImpl);
        OnlineViewer_var OnlineViewer = OnlineViewerImpl->_this();
        OnlineViewerImpl->_remove_ref();
        
        obj = orb->resolve_initial_references("NameService");
        CosNaming::NamingContext_var namingContext = CosNaming::NamingContext::_narrow(obj);
        if(CORBA::is_nil(namingContext)){
            throw std::string("error: failed to narrow naming context.");
        }
        
        CosNaming::Name name;
        name.length(1);
        name[0].id = CORBA::string_dup("OnlineViewer");
        name[0].kind = CORBA::string_dup("");
        namingContext->rebind(name, OnlineViewer);

        poaManager->activate();
        
        if (argc >= 2 && argv[1][0] != '-'){
            OpenHRP::ModelLoader_var ml = hrp::getModelLoader(namingContext);
            if (CORBA::is_nil(ml)){
                std::cerr << "openhrp-model-loader is not running" << std::endl;
                return 1;
            }
            OpenHRP::ModelLoader::ModelLoadOption opt;
            opt.readImage = true;
            opt.AABBdata.length(0);
            opt.AABBtype = OpenHRP::ModelLoader::AABB_NUM;
            GLbody *glbody = new GLbody();
            std::string url = argv[1];
            if (argv[1][0] != '/'){
                char buf[MAXPATHLEN];
                std::string cwd = getcwd(buf, MAXPATHLEN);
                url = cwd + '/' + url;
            }
            hrp::BodyPtr body(glbody);
            body->setName("model");
            OpenHRP::BodyInfo_var binfo = ml->getBodyInfoEx(url.c_str(), opt);
            hrp::loadBodyFromBodyInfo(body, binfo, false, GLlinkFactory);
            loadShapeFromBodyInfo(glbody, binfo);
            scene.addBody(body);
        }
        
        GLlink::useAbsTransformToDraw();
        GLbody::useAbsTransformToDraw();

        SDLwindow window(&scene, &log);
        window.init(wsize, wsize);
        if (!useDefaultLights) scene.defaultLights(false);
        
        while(window.oneStep());
        
    }
    catch(OpenHRP::ModelLoader::ModelLoaderException ex){
        std::cerr << ex.description << std::endl;
    }
    catch (CORBA::SystemException& ex) {
        std::cerr << ex._rep_id() << std::endl;
    }
    catch (const std::string& error){
        std::cerr << error << std::endl;
    }
    
    try {
        orb->destroy();
    }
    catch(...){
        
    }
    
    return 0;
}
