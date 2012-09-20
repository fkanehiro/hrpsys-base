
/////////////////////////////////////////////////////////////
//
// Source file for HpanedUI
//
//    This class implements the user interface created in 
//    RapidApp.
//
//    Restrict changes to those sections between
//    the "//--- Start/End editable code block" markers
//
//    This will allow RapidApp to integrate changes more easily
//
//    This class is a ViewKit user interface "component".
//    For more information on how components are used, see the
//    "ViewKit Programmers' Manual", and the RapidApp
//    User's Guide.
//
//
/////////////////////////////////////////////////////////////


#include "HpanedUI.h" // Generated header file for this class

#include <Sgm/HPanedW.h> 
#include <Xm/BulletinB.h> 
#include <Xm/Scale.h> 
#include <Xm/ToggleB.h> 
#include <Vk/VkResource.h>


// Externally defined classes referenced by this class: 

#include <Inventor/Xt/viewers/SoXtExaminerViewer.h>
#include <Inventor/So.h> // Includes ALL Inventor headers
                         // Replace for efficiency and faster compilation
//---- Start editable code block: headers and declarations


//---- End editable code block: headers and declarations


// These are default resources for widgets in objects of this class
// All resources will be prepended by *<name> at instantiation,
// where <name> is the name of the specific instance, as well as the
// name of the baseWidget. These are only defaults, and may be overriden
// in a resource file by providing a more specific resource name

String  HpanedUI::_defaultHpanedUIResources[] = {
        "*autoToggle.labelString:  auto move",
        "*speedScale.titleString:  speed",

        //---- Start editable code block: HpanedUI Default Resources


        //---- End editable code block: HpanedUI Default Resources

        (char*)NULL
};

HpanedUI::HpanedUI ( const char *name ) : VkComponent ( name ) 
{ 
    // No widgets are created by this constructor.
    // If an application creates a component using this constructor,
    // It must explictly call create at a later time.
    // This is mostly useful when adding pre-widget creation
    // code to a derived class constructor.

    //---- Start editable code block: Hpaned constructor 2


    //---- End editable code block: Hpaned constructor 2


}    // End Constructor




HpanedUI::HpanedUI ( const char *name, Widget parent ) : VkComponent ( name ) 
{ 
    //---- Start editable code block: Hpaned pre-create


    //---- End editable code block: Hpaned pre-create



    // Call creation function to build the widget tree.

     create ( parent );

    //---- Start editable code block: Hpaned constructor


    //---- End editable code block: Hpaned constructor


}    // End Constructor


HpanedUI::~HpanedUI() 
{
    delete _viewer;


    //---- Start editable code block: HpanedUI destructor


    //---- End editable code block: HpanedUI destructor
}    // End destructor



void HpanedUI::create ( Widget parent )
{
    Arg      args[11];
    Cardinal count;
    count = 0;

    // Load any class-defaulted resources for this object

    setDefaultResources ( parent, _defaultHpanedUIResources  );


    // Create an unmanaged widget as the top of the widget hierarchy

    _baseWidget = _hpaned = XtVaCreateWidget ( _name,
                                               sgHorzPanedWindowWidgetClass,
                                               parent, 
                                               (XtPointer) NULL ); 

    // install a callback to guard against unexpected widget destruction

    installDestroyHandler();


    // Create widgets used in this component
    // All variables are data members of this class

    _viewer = new SoXtExaminerViewer( _baseWidget, "viewer");
    _viewer->show();


    _bulletinBoard = XtVaCreateManagedWidget  ( "bulletinBoard",
                                                 xmBulletinBoardWidgetClass,
                                                 _baseWidget, 
                                                 XmNresizePolicy, XmRESIZE_GROW, 
                                                 XmNwidth, 132, 
                                                 XmNheight, 566, 
                                                 (XtPointer) NULL ); 


    _speedScale = XtVaCreateManagedWidget  ( "speedScale",
                                          xmScaleWidgetClass,
                                          _bulletinBoard, 
                                              XmNsliderVisual, XmSHADOWED, 
                                              XmNslidingMode, XmSLIDER, 
                                              XmNvalue, 50, 
                                          XmNorientation, XmVERTICAL, 
                                          XmNshowValue, True, 
                                              XmNx, 12, 
                                          XmNy, 55, 
                                              XmNwidth, 110, 
                                          XmNheight, 330, 
                                          (XtPointer) NULL ); 

    XtAddCallback ( _speedScale,
                    XmNvalueChangedCallback,
                    &HpanedUI::speedScaleCBCallback,
                    (XtPointer) this ); 

    XtAddCallback ( _speedScale,
                    XmNdragCallback,
                    &HpanedUI::speedScaleCBCallback,
                    (XtPointer) this ); 


    _scale = XtVaCreateManagedWidget  ( "scale",
                                         xmScaleWidgetClass,
                                         _bulletinBoard, 
                                         XmNorientation, XmHORIZONTAL, 
                                         XmNshowValue, True, 
                                         XmNx, 630, 
                                         XmNy, 85, 
                                         XmNwidth, 40, 
                                         XmNheight, 34, 
                                         (XtPointer) NULL ); 


    _autoToggle = XtVaCreateManagedWidget  ( "autoToggle",
                                              xmToggleButtonWidgetClass,
                                              _bulletinBoard, 
                                              XmNalignment, XmALIGNMENT_BEGINNING, 
                                              XmNlabelType, XmSTRING, 
                                              XmNx, 10, 
                                              XmNy, 470, 
                                              XmNwidth, 105, 
                                              XmNheight, 26, 
                                              (XtPointer) NULL ); 

    XtAddCallback ( _autoToggle,
                    XmNvalueChangedCallback,
                    &HpanedUI::autoToggleCBCallback,
                    (XtPointer) this ); 


    XtVaSetValues ( _viewer->getWidget(),
                    XmNwidth, 588, 
                    XmNheight, 566, 
                    (XtPointer) NULL );

    //---- Start editable code block: HpanedUI create

    //_viewer->setAutoRedraw(FALSE);  // we'll do this manually

    //---- End editable code block: HpanedUI create
}

const char * HpanedUI::className()
{
    return ("HpanedUI");
}    // End className()




/////////////////////////////////////////////////////////////// 
// The following functions are static member functions used to 
// interface with Motif.
/////////////////////////////////// 

void HpanedUI::autoToggleCBCallback ( Widget    w,
                                      XtPointer clientData,
                                      XtPointer callData ) 
{ 
    HpanedUI* obj = ( HpanedUI * ) clientData;
    obj->autoToggleCB ( w, callData );
}

void HpanedUI::speedScaleCBCallback ( Widget    w,
                                      XtPointer clientData,
                                      XtPointer callData ) 
{ 
    HpanedUI* obj = ( HpanedUI * ) clientData;
    obj->speedScaleCB ( w, callData );
}



/////////////////////////////////////////////////////////////// 
// The following functions are called from the menu items 
// in this window.
/////////////////////////////////// 

void HpanedUI::autoToggleCB ( Widget, XtPointer ) 
{
    // This virtual function is called from autoToggleCBCallback.
    // This function is normally overriden by a derived class.

}

void HpanedUI::speedScaleCB ( Widget, XtPointer ) 
{
    // This virtual function is called from speedScaleCBCallback.
    // This function is normally overriden by a derived class.

}



//---- Start editable code block: End of generated code


//---- End editable code block: End of generated code
