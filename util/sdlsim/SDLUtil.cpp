#include <math.h>
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif
#include <SDL.h>
#include "GLmodel.h"
#include "SDLUtil.h"


SDLwindow::SDLwindow(GLscene* i_scene) :
    scene(i_scene),
    width(640), height(480),
    aspect(((double)width)/height),
    pan(M_PI/4), tilt(M_PI/16), radius(5),
    button(-1),
    isShiftPressed(false), isControlPressed(false),
    xCenter(0), yCenter(0), zCenter(0.8)
{
}

bool SDLwindow::init()
{
    if(SDL_Init(SDL_INIT_VIDEO)<0) {
        fprintf(stderr,"failed to initialize SDL.\n");
        return false;
    }

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER,1);
    SDL_Surface *screen;
    screen=SDL_SetVideoMode(width,height,32,SDL_HWSURFACE | SDL_GL_DOUBLEBUFFER | SDL_OPENGL | SDL_RESIZABLE);
    if(!screen) {
        fprintf(stderr,"failed to set video mode to %dx%dx32.\n",width,height);
        SDL_Quit();
        return false;
    }
    SDL_WM_SetCaption("sdlsim", NULL);
    SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY,SDL_DEFAULT_REPEAT_INTERVAL);
    return true;
}

bool SDLwindow::processEvents()
{
    SDL_Event event;
    while(SDL_PollEvent(&event)){
        switch(event.type){
        case SDL_QUIT:
            return false;
        case SDL_KEYDOWN:
        {
            //printf("%d\n", event.key.keysym.sym);
            int delta = isShiftPressed ? 10 : 1;
            switch(event.key.keysym.sym){
            case SDLK_q:
                return false;
            case SDLK_p:
                scene->play(); 
                break;
            case SDLK_KP_PLUS:
                scene->faster();
                break;
            case SDLK_KP_MINUS:
                scene->slower();
                break;
            case SDLK_r:
                scene->record();
                break;
            case SDLK_RIGHT:
                if (isControlPressed){
                    scene->tail();
                }else{
                    scene->next(delta);
                }
                break;
            case SDLK_LEFT:
                if (isControlPressed){
                    scene->head();
                }else{
                    scene->prev(delta);
                }
                break;
            case SDLK_LSHIFT:
            case SDLK_RSHIFT:
                isShiftPressed = true;
                break;
            case SDLK_LCTRL:
            case SDLK_RCTRL:
                isControlPressed = true;
                break;
            }
            break;
        }
        case SDL_KEYUP:
            switch(event.key.keysym.sym){
            case SDLK_LSHIFT:
            case SDLK_RSHIFT:
                    isShiftPressed = false;
                    break;
            case SDLK_LCTRL:
            case SDLK_RCTRL:
                isControlPressed = false;
                break;
            }
            break;
            
        case SDL_MOUSEBUTTONDOWN:
            button = event.button.button;
            switch(event.button.button){
            case SDL_BUTTON_LEFT:
                break;
            case SDL_BUTTON_MIDDLE:
                break;
            case SDL_BUTTON_RIGHT:
                break;
            case SDL_BUTTON_WHEELUP:
                break;
            case SDL_BUTTON_WHEELDOWN:
                break;
            }
            break;
            
        case SDL_MOUSEBUTTONUP:
            button = -1;
            switch(event.button.button){
            case SDL_BUTTON_LEFT:
                break;
            case SDL_BUTTON_MIDDLE:
                break;
            case SDL_BUTTON_RIGHT:
                break;
            case SDL_BUTTON_WHEELUP:
                break;
            case SDL_BUTTON_WHEELDOWN:
                break;
            }
            break;
        case SDL_MOUSEMOTION:
        {
            int dx = event.motion.xrel;
            int dy = event.motion.yrel;
            if (button == SDL_BUTTON_LEFT){
                if (isShiftPressed){
                    radius *= (1+ 0.1*dy);
                    if (radius < 0.1) radius = 0.1; 
                }else{
                    pan  -= 0.05*dx;
                    tilt += 0.05*dy;
                    if (tilt >  M_PI/2) tilt =  M_PI/2;
                    if (tilt < -M_PI/2) tilt = -M_PI/2;
                }
            }else if (button == SDL_BUTTON_RIGHT){
                xCenter += sin(pan)*dx*0.01;
                yCenter -= cos(pan)*dx*0.01;
                zCenter += dy*0.01;
            }
        }
        break;
        case SDL_VIDEORESIZE:
            width = event.resize.w;
            height = event.resize.h;
            SDL_SetVideoMode(width,height,32,SDL_HWSURFACE | SDL_GL_DOUBLEBUFFER | SDL_OPENGL | SDL_RESIZABLE);
            glViewport(0, 0, width, height);
            aspect = ((double)width)/height;
            break;
        }
    }
    return true;
}

void SDLwindow::draw()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(30,aspect, 0.1, 100);
    
    double xEye = xCenter + radius*cos(tilt)*cos(pan);
    double yEye = yCenter + radius*cos(tilt)*sin(pan);
    double zEye = zCenter + radius*sin(tilt);
    
    gluLookAt(xEye, yEye, zEye,
              xCenter, yCenter, zCenter,
              0,0,1);
    
    glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);
    
    scene->draw();
}

void SDLwindow::swapBuffers()
{
    SDL_GL_SwapBuffers();
}
