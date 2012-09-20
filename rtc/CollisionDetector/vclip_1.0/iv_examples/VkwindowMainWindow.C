
//////////////////////////////////////////////////////////////
//
// Source file for VkwindowMainWindow
//
//    This class is a subclass of VkWindow
//
//
// Normally, very little in this file should need to be changed.
// Create/add/modify menus using RapidApp.
//
// Try to restrict any changes to the bodies of functions
// corresponding to menu items, the constructor and destructor.
//
// Restrict changes to those sections between
// the "//--- Start/End editable code block" markers
//
// Doing so will allow you to make changes using RapidApp
// without losing any changes you may have made manually
//
// Avoid gratuitous reformatting and other changes that might
// make it difficult to integrate changes made using RapidApp
//////////////////////////////////////////////////////////////
#include "VkwindowMainWindow.h"

#include <Vk/VkApp.h>
#include <Vk/VkFileSelectionDialog.h>
#include <Vk/VkSubMenu.h>
#include <Vk/VkRadioSubMenu.h>
#include <Vk/VkMenuItem.h>
#include <Vk/VkMenuBar.h>
#include <Vk/VkResource.h>


// Externally defined classes referenced by this class: 

#include "Hpaned.h"

extern void VkUnimplemented ( Widget, const char * );

//---- Start editable code block: headers and declarations

#include <Inventor/Xt/viewers/SoXtExaminerViewer.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoSelection.h>

//---- End editable code block: headers and declarations



// These are default resources for widgets in objects of this class
// All resources will be prepended by *<name> at instantiation,
// where <name> is the name of the specific instance, as well as the
// name of the baseWidget. These are only defaults, and may be overriden
// in a resource file by providing a more specific resource name

String  VkwindowMainWindow::_defaultVkwindowMainWindowResources[] = {
        "*exitButton.accelerator:  Ctrl<Key>Q",
        "*exitButton.acceleratorText:  Ctrl+Q",
        "*exitButton.labelString:  Exit",
        "*exitButton.mnemonic:  x",
        "*filePane.labelString:  File",
        "*filePane.mnemonic:  F",
        "*helpPane.labelString:  Help",
        "*helpPane.mnemonic:  H",
        "*help_click_for_help.accelerator:  Shift<Key>F1",
        "*help_click_for_help.acceleratorText:  Shift+F1",
        "*help_click_for_help.labelString:  Click For Help",
        "*help_click_for_help.mnemonic:  C",
        "*help_index.labelString:  Index",
        "*help_index.mnemonic:  I",
        "*help_keys_and_short.labelString:  Keys and Shortcuts",
        "*help_keys_and_short.mnemonic:  K",
        "*help_overview.labelString:  Overview",
        "*help_overview.mnemonic:  O",
        "*help_prod_info.labelString:  Product Information",
        "*help_prod_info.mnemonic:  P",
        "*openButton.accelerator:  Ctrl<Key>O",
        "*openButton.acceleratorText:  Ctrl+O",
        "*openButton.labelString:  Open...",
        "*openButton.mnemonic:  O",
        "*title:  V-Clip Collision Detection",

        //---- Start editable code block: VkwindowMainWindow Default Resources


        //---- End editable code block: VkwindowMainWindow Default Resources

        (char*)NULL
};


//---- Class declaration

VkwindowMainWindow::VkwindowMainWindow ( const char *name,
                                        ArgList args,
                                        Cardinal argCount) : 
                                  VkWindow ( name, args, argCount )
{
    // Load any class-default resources for this object

    setDefaultResources ( baseWidget(), _defaultVkwindowMainWindowResources  );


    // Create the view component contained by this window

    _hpaned = new Hpaned ( "hpaned",mainWindowWidget() );


    XtVaSetValues ( _hpaned->baseWidget(),
                    XmNwidth, 734, 
                    XmNheight, 572, 
                    (XtPointer) NULL );

    // Add the component as the main view

    addView ( _hpaned );

    // Create the panes of this window's menubar. The menubar itself
    // is created automatically by ViewKit


    // The filePane menu pane

    _filePane =  addMenuPane ( "filePane" );

    _openButton =  _filePane->addAction ( "openButton", 
                                        &VkwindowMainWindow::openFileCallback, 
                                        (XtPointer) this );

    _separator1 =  _filePane->addSeparator();

    _exitButton =  _filePane->addAction ( "exitButton", 
                                        &VkwindowMainWindow::quitCallback, 
                                        (XtPointer) this );


    //---- Start editable code block: VkwindowMainWindow constructor


    extern SoSelection *root;
    root = new SoSelection;
    viewer()->setSceneGraph(root);


    //---- End editable code block: VkwindowMainWindow constructor


}    // End Constructor


VkwindowMainWindow::~VkwindowMainWindow()
{
    delete _hpaned;
    //---- Start editable code block: VkwindowMainWindow destructor


    //---- End editable code block: VkwindowMainWindow destructor
}    // End destructor


const char *VkwindowMainWindow::className()
{
    return ("VkwindowMainWindow");
}    // End className()


Boolean VkwindowMainWindow::okToQuit()
{
    //---- Start editable code block: VkwindowMainWindow okToQuit



    // This member function is called when the user quits by calling
    // theApplication->terminate() or uses the window manager close protocol
    // This function can abort the operation by returning FALSE, or do some.
    // cleanup before returning TRUE. The actual decision is normally passed on
    // to the view object


    // The view object is an Inventor component, which does
    // not currently support the okToQuit protocol

    return ( TRUE );
    //---- End editable code block: VkwindowMainWindow okToQuit
}    // End okToQuit()



/////////////////////////////////////////////////////////////// 
// The following functions are static member functions used to 
// interface with Motif.
/////////////////////////////////// 

void VkwindowMainWindow::openFileCallback ( Widget    w,
                                            XtPointer clientData,
                                            XtPointer callData ) 
{ 
    VkwindowMainWindow* obj = ( VkwindowMainWindow * ) clientData;
    obj->openFile ( w, callData );
}

void VkwindowMainWindow::quitCallback ( Widget    w,
                                        XtPointer clientData,
                                        XtPointer callData ) 
{ 
    VkwindowMainWindow* obj = ( VkwindowMainWindow * ) clientData;
    obj->quit ( w, callData );
}





/////////////////////////////////////////////////////////////// 
// The following functions are called from callbacks 
/////////////////////////////////// 

void VkwindowMainWindow::openFile ( Widget, XtPointer ) 
{
    //---- Start editable code block: VkwindowMainWindow openFile

    // This virtual function is called from openFileCallback
    // Use the blocking mode of the file selection dialog
    // to get a file

  theFileSelectionDialog->setFilterPattern("*.txt");

    if(theFileSelectionDialog->postAndWait() == VkDialogManager::OK)
    {

      //::VkUnimplemented ( NULL, "VkwindowMainWindow::openFile" );

      const char *filename = theFileSelectionDialog->fileName();

      void initPtrees(const char *demoFname);
      initPtrees(filename);
      viewer()->viewAll();

    }

    //---- End editable code block: VkwindowMainWindow openFile

}    // End VkwindowMainWindow::openFile()

void VkwindowMainWindow::quit ( Widget, XtPointer ) 
{
    // Exit via quitYourself() to allow the application
    // to go through its normal shutdown routines, checking with
    // all windows, views, and so on.

    theApplication->quitYourself();

}    // End VkwindowMainWindow::quit()




//---- Start editable code block: End of generated code

SoXtExaminerViewer *VkwindowMainWindow::viewer() {return _hpaned->viewer();}

//---- End editable code block: End of generated code


