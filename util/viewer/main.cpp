#include <sys/param.h>
#include <iostream>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <SDL/SDL_timer.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "util/GLlink.h"
#include "util/GLbody.h"
#include "util/GLutil.h"
#include "util/SDLUtil.h"
#include "OnlineViewer_impl.h"
#include "GLscene.h"

using namespace OpenHRP;

int main(int argc, char *argv[])
{
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

        int wsize=0;
        bool useDefaultLights=true;
        for (int i=1; i<argc; i++){
            if (strcmp(argv[i], "-size")==0){
                wsize = atoi(argv[++i]);
            }else if(strcmp(argv[i], "-max-edge-length")==0){
                scene.maxEdgeLen(atof(argv[++i]));
            }else if(strcmp(argv[i], "-no-default-lights")==0){
                useDefaultLights=false;
            }
        }
        if (argc >= 2 && argv[1][0] != '-'){
            OpenHRP::ModelLoader_var ml = hrp::getModelLoader(namingContext);
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
