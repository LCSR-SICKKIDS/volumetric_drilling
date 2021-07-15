//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar

    \author    <pkunjam1@jhu.edu>
    \author    Punit Kunjam
*/
//==============================================================================

#include "volumetric_drilling.h"
#include <boost/program_options.hpp>
#include <mutex>

using namespace std;

cVoxelObject* g_volObject;

cToolCursor* g_targetToolCursor;

int g_renderingMode = 0;

double g_opticalDensity;

mutex g_mutexVoxel;

cCollisionAABBBox g_volumeUpdate;

cColorb g_zeroColor(0x00, 0x00, 0x00, 0x00);

bool g_flagStart = true;

int counter = 0;

cGenericObject* g_selectedObject = NULL;

cTransform g_tool_T_object;

// a haptic device handler
cHapticDeviceHandler* g_deviceHandler;

// a pointer to the current haptic device
cGenericHapticDevicePtr g_hapticDevice;

bool g_flagMarkVolumeForUpdate = false;

afRigidBodyPtr g_drillRigidBody;

cShapeSphere* g_burrMesh;

// tool's rotation matrix
cMatrix3d g_toolRotMat;

// rate of drill movement
double g_drillRate = 0.020f;

// Local offset between shaft tool cursors
double g_dX = 0.03;

// camera to render the world
afCameraPtr g_mainCamera;

bool g_showDrill = true;

bool g_showGoalProxySpheres = false;

// list of tool cursors
vector<cToolCursor*> g_toolCursorList(8);

// radius of tool cursors
vector<double> g_toolCursorRadius{0.02, 0.013, 0.015, 0.017, 0.019, 0.021, 0.023, 0.025};

// warning pop-up panel
cPanel* g_warningPopup;
cLabel* g_warningText;

// panel to display current drill size
cPanel* g_drillSizePanel;
cLabel* g_drillSizeText;

// current and maximum distance between proxy and goal spheres
double g_currError = 0;
double g_maxError = 0;

// for storing index of follow sphere
int g_targetToolCursorIdx = 0;

// toggles whether the drill mesh should move slowly towards the followSphere
// or make a sudden jump
bool g_suddenJump = true;

// index of current drill size
int g_drillSizeIdx = 0;

// current drill size
int g_currDrillSize = 2;

// color property of bone
cColorb g_boneColor(255, 249, 219, 255);

// get color of voxels at (x,y,z)
cColorb g_storedColor(0x00, 0x00, 0x00, 0x00);

enum HapticStates
{
    HAPTIC_IDLE,
    HAPTIC_SELECTION
};

HapticStates state = HAPTIC_IDLE;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// Initialize tool cursors
void toolCursorInit(const afWorldPtr);

void incrementDevicePos(cVector3d a_pos);

void incrementDeviceRot(cVector3d a_rot);

// update position of shaft tool cursors
void shaftToolCursorsPosUpdate(cTransform a_devicePose);

// check for shaft collision
void checkShaftCollision(void);

// update position of drill mesh
void drillPosUpdate(void);

// toggles size of the drill burr
void changeDrillSize(void);

//------------------------------------------------------------------------------


void afVolmetricDrillingPlugin::init(int argc, char **argv, const afWorldPtr a_afWorld){

    m_worldPtr = a_afWorld;

    // Get first camera
    g_mainCamera = m_worldPtr->getCameras()[0];

    double maxStiffness = 10.0;

    // Initializing tool's rotation matrix as an identity matrix
    g_toolRotMat.identity();
    g_toolRotMat = g_mainCamera->getLocalRot() * g_toolRotMat;

    // importing drill model
    g_drillRigidBody = m_worldPtr->getRigidBody("mastoidectomy_drill");
    if (!g_drillRigidBody){
        cerr << "ERROR! FAILED TO FIND DRILL RIGID BODY NAMED MASTOIDECTOMY_DRILL" << endl;
        return;
    }
    else{
        g_burrMesh = new cShapeSphere(0.02);
        g_burrMesh->setRadius(0.02);
        g_burrMesh->m_material->setGrayDark();
        g_burrMesh->setShowEnabled(true);
        g_drillRigidBody->addChildSceneObject(g_burrMesh, cTransform());
        m_worldPtr->addSceneObjectToWorld(g_burrMesh);
    }

    afVolumePtr volume;
    volume = m_worldPtr->getVolume("mastoidectomy_volume");
    if (!volume){
        cerr << "ERROR! FAILED TO FIND DRILL VOLUME NAMED MASTOIDECTOMY_DRILL" << endl;
        return;
    }
    else{
        g_volObject = volume->getInternalVolume();
    }

    // create a haptic device handler
    g_deviceHandler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    g_deviceHandler->getDevice(g_hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = g_hapticDevice->getSpecifications();

    // Initializing tool cursors
    toolCursorInit(a_afWorld);

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = g_toolCursorList[0]->getWorkspaceScaleFactor();

    // stiffness properties
    maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI40();

    // A warning pop-up that shows up while drilling at critical region
    g_warningPopup = new cPanel();
    g_warningPopup->set(g_mainCamera->m_width/2, g_mainCamera->m_height/5);
    g_warningPopup->setColor(cColorf(0.6,0,0));
    g_warningPopup->setLocalPos(g_mainCamera->m_width*0.3, g_mainCamera->m_height*0.6, 0);
    g_mainCamera->getFrontLayer()->addChild(g_warningPopup);
    g_warningPopup->setShowPanel(false);

    g_warningText = new cLabel(font);
    g_warningText->setLocalPos(0.31 * g_mainCamera->m_width, 0.67 * g_mainCamera->m_height, 0.5);
    g_warningText->m_fontColor.setWhite();
    g_warningText->setFontScale(1.0);
    g_warningText->setText("WARNING! Critical Region Detected");
    g_mainCamera->getFrontLayer()->addChild(g_warningText);
    g_warningText->setShowEnabled(false);

    // A panel to display current drill size
    g_drillSizePanel = new cPanel();
    g_drillSizePanel->setSize(170, 50);
    g_drillSizePanel->setCornerRadius(10, 10, 10, 10);
    g_drillSizePanel->setLocalPos(40,60);
    g_drillSizePanel->setColor(cColorf(1, 1, 1));
    g_drillSizePanel->setTransparencyLevel(0.8);
    g_mainCamera->getFrontLayer()->addChild(g_drillSizePanel);

    g_drillSizeText = new cLabel(font);
    g_drillSizeText->setLocalPos(50,70);
    g_drillSizeText->m_fontColor.setBlack();
    g_drillSizeText->setFontScale(.75);
    g_drillSizeText->setText("Drill Size: " + cStr(g_currDrillSize) + " mm");
    g_mainCamera->getFrontLayer()->addChild(g_drillSizeText);
}

void afVolmetricDrillingPlugin::graphicsUpdate(){

    // update region of voxels to be updated
    if (g_flagMarkVolumeForUpdate)
    {
        g_mutexVoxel.lock();
        cVector3d min = g_volumeUpdate.m_min;
        cVector3d max = g_volumeUpdate.m_max;
        g_volumeUpdate.setEmpty();
        g_mutexVoxel.unlock();
        ((cTexture3d*)g_volObject->m_texture.get())->markForPartialUpdate(min, max);
        g_flagMarkVolumeForUpdate = false;
    }
}

void afVolmetricDrillingPlugin::physicsUpdate(double dt){

    m_worldPtr->getChaiWorld()->computeGlobalPositions(true);


//    g_toolCursorList[0]->setLocalRot(g_mainCamera->getLocalRot());

//    for (int i = 0 ; i < g_toolCursorList.size() ; i++){
//        g_toolCursorList[i]->setLocalRot(g_mainCamera->getLocalRot());
//    }

    //Updates position of shaft tool cursors
//    cTransform T_d = g_toolCursorList[0]->getDeviceLocalTransform();
//    T_d.setLocalRot(g_mainCamera->getLocalRot() * T_d.getLocalRot());
//    shaftToolCursorsPosUpdate(T_d);


    // updates position of drill burr/tip tool cursor
    g_toolCursorList[0]->updateFromDevice();

    shaftToolCursorsPosUpdate(g_toolCursorList[0]->getDeviceLocalTransform());

    // check for shaft collision
    checkShaftCollision();

    // updates position of drill mesh
    drillPosUpdate();


     // read user switch
    int userSwitches = g_toolCursorList[0]->getUserSwitches();

    if (g_toolCursorList[0]->isInContact(g_volObject) && g_targetToolCursorIdx == 0 /*&& (userSwitches == 2)*/)
    {

        // retrieve contact event
        cCollisionEvent* contact = g_toolCursorList[0]->m_hapticPoint->getCollisionEvent(0);

        cVector3d orig(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ);
        cVector3d ray = orig;

        g_volObject->m_texture->m_image->getVoxelColor(uint(ray.x()), uint(ray.y()), uint(ray.z()), g_storedColor);

        //if the tool comes in contact with the critical region, instantiate the warning message
        if(g_storedColor != g_boneColor && g_storedColor != g_zeroColor)
        {
            g_warningPopup->setShowPanel(true);
            g_warningText->setShowEnabled(true);
        }

        g_volObject->m_texture->m_image->setVoxelColor(uint(ray.x()), uint(ray.y()), uint(ray.z()), g_zeroColor);

        g_mutexVoxel.lock();
        g_volumeUpdate.enclose(cVector3d(uint(ray.x()), uint(ray.y()), uint(ray.z())));
        g_mutexVoxel.unlock();
        // mark voxel for update

        g_flagMarkVolumeForUpdate = true;
    }

    // remove warning panel
    else
    {
        g_warningPopup->setShowPanel(false);
        g_warningText->setShowEnabled(false);
    }


    for(auto tool : g_toolCursorList)
    {
        // compute interaction forces
        tool->computeInteractionForces();

        // check if device remains stuck inside voxel object
        cVector3d force = tool->getDeviceGlobalForce();

        if (g_flagStart)
        {
            if (force.length() != 0.0)
            {

                tool->initialize();
                counter = 0;
            }
            else
            {
                counter++;
                if (counter > 10)
                    g_flagStart = false;
            }
        }
        else
        {
            if (force.length() > 10.0)
            {
                g_flagStart = true;
            }
        }


        /////////////////////////////////////////////////////////////////////////
        // MANIPULATION
        /////////////////////////////////////////////////////////////////////////

        // compute transformation from world to tool (haptic device)
        cTransform world_T_tool = tool->getDeviceGlobalTransform();

        // get status of user switch
        bool button = tool->getUserSwitch(0);

        //
        // STATE 1:
        // Idle mode - user presses the user switch
        //
        if ((state == HAPTIC_IDLE) && (button == true))
        {
            // check if at least one contact has occurred
            if (tool->m_hapticPoint->getNumCollisionEvents() > 0)
            {
                // get contact event
                cCollisionEvent* collisionEvent = tool->m_hapticPoint->getCollisionEvent(0);

                // get object from contact event
                g_selectedObject = collisionEvent->m_object;
            }
            else
            {
                g_selectedObject = g_volObject;
            }

            // get transformation from object
            cTransform world_T_object = g_selectedObject->getGlobalTransform();

            // compute inverse transformation from contact point to object
            cTransform tool_T_world = world_T_tool;
            tool_T_world.invert();

            // store current transformation tool
            g_tool_T_object = tool_T_world * world_T_object;

            // update state
            state = HAPTIC_SELECTION;
        }


            //
            // STATE 2:
            // Selection mode - operator maintains user switch enabled and moves object
            //
        else if ((state == HAPTIC_SELECTION) && (button == true))
        {
            // compute new transformation of object in global coordinates
            cTransform world_T_object = world_T_tool * g_tool_T_object;

            // compute new transformation of object in local coordinates
            cTransform parent_T_world = g_selectedObject->getParent()->getLocalTransform();
            parent_T_world.invert();
            cTransform parent_T_object = parent_T_world * world_T_object;

            // assign new local transformation to object
            g_selectedObject->setLocalTransform(parent_T_object);

            // set zero forces when manipulating objects
            tool->setDeviceGlobalForce(0.0, 0.0, 0.0);

            tool->initialize();
        }

            //
            // STATE 3:
            // Finalize Selection mode - operator releases user switch.
            //
        else
        {
            state = HAPTIC_IDLE;
        }


        cVector3d localPos = g_drillRigidBody->getLocalPos();
        cQuaternion localRot;
        localRot.fromRotMat(g_drillRigidBody->getLocalRot());

        /////////////////////////////////////////////////////////////////////////
        // FINALIZE
        /////////////////////////////////////////////////////////////////////////

        // send forces to haptic device
//            tool->applyToDevice();
    }

}

///
/// \brief This method initializes the tool cursors.
/// \param a_afWorld    A world that contains all objects of the virtual environment
/// \return
///
void toolCursorInit(const afWorldPtr a_afWorld){

    for(int i=0; i<g_toolCursorList.size(); i++)
    {
        g_toolCursorList[i] = new cToolCursor(a_afWorld->getChaiWorld());

        a_afWorld->addSceneObjectToWorld(g_toolCursorList[i]);

        if(i == 0)
        {
            g_toolCursorList[i]->setHapticDevice(g_hapticDevice);

            // map the physical workspace of the haptic device to a larger virtual workspace.

            g_toolCursorList[i]->setWorkspaceRadius(1.0);
            g_toolCursorList[i]->setWaitForSmallForce(true);
            g_toolCursorList[i]->start();
            g_toolCursorList[i]->m_hapticPoint->m_sphereProxy->setShowFrame(false);

            g_toolCursorList[i]->m_name = "mastoidectomy_drill";
            g_toolCursorList[i]->m_hapticPoint->setShow(g_showGoalProxySpheres, g_showGoalProxySpheres);
            g_toolCursorList[i]->m_hapticPoint->m_sphereProxy->m_material->setRedCrimson();
            g_toolCursorList[i]->m_hapticPoint->m_sphereGoal->m_material->setBlueAquamarine();

            // if the haptic device has a gripper, enable it as a user switch
            g_hapticDevice->setEnableGripperUserSwitch(true);
        }
        else
        {
            g_toolCursorList[i]->setShowContactPoints(g_showGoalProxySpheres, g_showGoalProxySpheres);
            g_toolCursorList[i]->m_hapticPoint->m_sphereProxy->m_material->setGreenChartreuse();
            g_toolCursorList[i]->m_hapticPoint->m_sphereGoal->m_material->setOrangeCoral();
        }

        g_toolCursorList[i]->setRadius(g_toolCursorRadius[i]);
     }

    // Initialize the start pose of the tool cursors
    cTransform T_d = g_drillRigidBody->getLocalTransform();
    g_toolCursorList[0]->setDeviceLocalTransform(T_d);
    shaftToolCursorsPosUpdate(T_d);
    for (int i = 0 ;  i < g_toolCursorList.size() ; i++){
        g_toolCursorList[i]->initialize();
    }
}


///
/// \brief incrementDevicePos
/// \param a_vel
///
void incrementDevicePos(cVector3d a_vel){
    cVector3d P = g_toolCursorList[0]->getDeviceLocalPos() + a_vel;
    g_toolCursorList[0]->setDeviceLocalPos(P);
}


///
/// \brief incrementDeviceRot
/// \param a_rot
///
void incrementDeviceRot(cVector3d a_rot){
    cMatrix3d R_cmd;
    R_cmd.setExtrinsicEulerRotationDeg(a_rot(0), a_rot(1), a_rot(2), C_EULER_ORDER_XYZ);
    cMatrix3d R = g_toolCursorList[0]->getDeviceLocalRot() * R_cmd;
    g_toolCursorList[0]->setDeviceLocalRot(R);
}

///
/// \brief This method updates the position of the shaft tool cursors
/// which eventually updates the position of the whole tool.
///
void shaftToolCursorsPosUpdate(cTransform a_devicePose){
    cVector3d n_x = a_devicePose.getLocalRot().getCol0() * g_dX;
    for (int i = 1 ; i < g_toolCursorList.size() ; i++){
        cVector3d P = a_devicePose.getLocalPos() + n_x * i;
        g_toolCursorList[i]->setDeviceLocalPos(P);
        g_toolCursorList[i]->setDeviceLocalRot(a_devicePose.getLocalRot());
    }
}

///
/// \brief This method checks for collision between the tool shaft and the volume.
/// The error between the proxy and goal position of each of the shaft tool cursors is constantly
/// computed. The shaft tool cursor having the maximum error is set as g_targetToolCursor. Further, the
/// position of the drill mesh is set such that it follows the proxy position of the g_targetToolCursor.
/// If there's no collision, the drill mesh follows the proxy position of the shaft tool cursor which is
/// closest to the tip tool cursor.
///
void checkShaftCollision(){

    g_maxError = 0;
    g_targetToolCursor = g_toolCursorList[0];
    g_targetToolCursorIdx = 0;
    for(int i=0; i<g_toolCursorList.size(); i++)
    {

        g_currError = cDistance(g_toolCursorList[i]->m_hapticPoint->getGlobalPosProxy(), g_toolCursorList[i]->m_hapticPoint->getGlobalPosGoal());

        if(abs(g_currError) > abs(g_maxError + 0.00001))
        {
            g_maxError = g_currError;
            g_targetToolCursor = g_toolCursorList[i];
            g_targetToolCursorIdx = i;
        }
    }
}


///
/// \brief This method updates the position of the drill mesh.
/// After obtaining g_targetToolCursor, the drill mesh adjust it's position and rotation
/// such that it follows the proxy position of the g_targetToolCursor.
///
void drillPosUpdate(){

    if(g_targetToolCursorIdx == 0){
        cTransform T_tip;
        T_tip.setLocalPos(g_toolCursorList[0]->m_hapticPoint->m_sphereProxy->getLocalPos());
        T_tip.setLocalRot(g_toolCursorList[0]->getDeviceLocalRot());
        g_drillRigidBody->setLocalTransform(T_tip);
    }
    else if(cDistance(g_targetToolCursor->m_hapticPoint->getGlobalPosProxy(), g_targetToolCursor->m_hapticPoint->getGlobalPosGoal()) <= 0.001)
    {
        // direction of positive x-axis of drill mesh
        cVector3d xDir = g_drillRigidBody->getLocalRot().getCol0();

        cVector3d newDrillPos;
        cMatrix3d newDrillRot;

        // drill mesh will make a sudden jump towards the followSphere
        if(!g_suddenJump)
        {
            newDrillPos = (g_targetToolCursor->m_hapticPoint->getGlobalPosProxy() - xDir * g_dX * g_targetToolCursorIdx);
        }

        // drill mesh slowly moves towards the followSphere
        else
        {
            newDrillPos = g_drillRigidBody->getLocalPos() + ((g_targetToolCursor->m_hapticPoint->getGlobalPosProxy() - xDir * g_dX * g_targetToolCursorIdx) - g_drillRigidBody->getLocalPos()) * 0.04;
        }

        cVector3d L = g_targetToolCursor->m_hapticPoint->getGlobalPosProxy() - g_toolCursorList[0]->getDeviceLocalPos();

//        cerr << "Colliding Cursor " << g_targetToolCursorIdx << " Error " << L.str(2) << endl;
//        if ( L.length() < 0.01){
//            newDrillRot = g_toolCursorList[0]->getDeviceLocalRot();
//        }
//        else{
//            newDrillRot = afUtils::getRotBetweenVectors<cMatrix3d>(L, cVector3d(1, 0, 0));
//        }

        newDrillRot = g_toolCursorList[0]->getDeviceLocalRot();

        cTransform trans;
        trans.setLocalPos(newDrillPos);
        trans.setLocalRot(newDrillRot);

//        g_drillRigidBody->setLocalPos(g_drillRigidBody->getLocalPos() + newDrillPos);
        g_drillRigidBody->setLocalTransform(trans);
    }
}


///
/// \brief This method changes the size of the tip tool cursor.
/// Currently, the size of the tip tool cursor can be set to 2mm, 4mm, and 6mm.
///
void changeDrillSize(){

    g_drillSizeIdx++;

    if(g_drillSizeIdx > 2)
    {
        g_drillSizeIdx = 0;
    }

    switch(g_drillSizeIdx)
    {
        case 0:
            g_toolCursorList[0]->setRadius(0.02);
            g_burrMesh->setRadius(0.02);
            cout << "Drill Size changed to 2 mm" << endl;
            g_currDrillSize = 2;
            g_drillSizeText->setText("Drill Size: " + cStr(g_currDrillSize) + " mm");
            break;

        case 1:
            g_toolCursorList[0]->setRadius(0.04);
            g_burrMesh->setRadius(0.04);
            cout << "Drill Size changed to 4 mm" << endl;
            g_currDrillSize = 4;
            g_drillSizeText->setText("Drill Size: " + cStr(g_currDrillSize) + " mm");
            break;

        case 2:
            g_toolCursorList[0]->setRadius(0.06);
            g_burrMesh->setRadius(0.06);
            cout << "Drill Size changed to 6 mm" << endl;
            g_currDrillSize = 6;
            g_drillSizeText->setText("Drill Size: " + cStr(g_currDrillSize) + " mm");
            break;

        default:
            break;
    }
}

void afVolmetricDrillingPlugin::keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods) {
    if (a_mods == GLFW_MOD_CONTROL){


        // controls linear motion of tool
        if (a_key == GLFW_KEY_W) {

            cVector3d dir = g_mainCamera->getUpVector() * g_drillRate;
            incrementDevicePos(dir);
        }

        else if (a_key == GLFW_KEY_D) {

            cVector3d dir = g_mainCamera->getRightVector() * g_drillRate;
            incrementDevicePos(dir);

        }

        else if (a_key == GLFW_KEY_S) {

            cVector3d dir = g_mainCamera->getUpVector() * g_drillRate;
            incrementDevicePos(-dir);

        }

        else if (a_key == GLFW_KEY_A) {

            cVector3d dir = g_mainCamera->getRightVector() * g_drillRate;
            incrementDevicePos(-dir);

        }

        else if (a_key == GLFW_KEY_K) {

            cVector3d dir = g_mainCamera->getLookVector() * g_drillRate;
            incrementDevicePos(-dir);

        }

        else if (a_key == GLFW_KEY_I) {

            cVector3d dir = g_mainCamera->getLookVector() * g_drillRate;
            incrementDevicePos(dir);
        }

        else if (a_key == GLFW_KEY_C) {
            g_showGoalProxySpheres = !g_showGoalProxySpheres;
            for (int i = 0 ; i < g_toolCursorList.size() ; i++){
                g_toolCursorList[i]->m_hapticPoint->setShow(g_showGoalProxySpheres, g_showGoalProxySpheres);
            }
        }

        // option - polygonize model and save to file
        else if (a_key == GLFW_KEY_P) {
            cMultiMesh *surface = new cMultiMesh;
            g_volObject->polygonize(surface, 0.01, 0.01, 0.01);
            double SCALE = 0.1;
            double METERS_TO_MILLIMETERS = 1000.0;
            surface->scale(SCALE * METERS_TO_MILLIMETERS);
            surface->setUseVertexColors(true);
            surface->saveToFile("volume.obj");
            cout << "> Volume has been polygonized and saved to disk                            \r";
            delete surface;
        }
    }
    else{

        // option - reduce size along X axis
        if (a_key == GLFW_KEY_4) {
            double value = cClamp((g_volObject->m_maxCorner.x() - 0.005), 0.01, 0.5);
            g_volObject->m_maxCorner.x(value);
            g_volObject->m_minCorner.x(-value);
            g_volObject->m_maxTextureCoord.x(0.5 + value);
            g_volObject->m_minTextureCoord.x(0.5 - value);
            cout << "> Reduce size along X axis.                            \r";
        }

        // option - increase size along X axis
        else if (a_key == GLFW_KEY_5) {
            double value = cClamp((g_volObject->m_maxCorner.x() + 0.005), 0.01, 0.5);
            g_volObject->m_maxCorner.x(value);
            g_volObject->m_minCorner.x(-value);
            g_volObject->m_maxTextureCoord.x(0.5 + value);
            g_volObject->m_minTextureCoord.x(0.5 - value);
            cout << "> Increase size along X axis.                            \r";
        }

        // option - reduce size along Y axis
        else if (a_key == GLFW_KEY_6) {
            double value = cClamp((g_volObject->m_maxCorner.y() - 0.005), 0.01, 0.5);
            g_volObject->m_maxCorner.y(value);
            g_volObject->m_minCorner.y(-value);
            g_volObject->m_maxTextureCoord.y(0.5 + value);
            g_volObject->m_minTextureCoord.y(0.5 - value);
            cout << "> Reduce size along Y axis.                            \r";
        }

        // option - increase size along Y axis
        else if (a_key == GLFW_KEY_7) {
            double value = cClamp((g_volObject->m_maxCorner.y() + 0.005), 0.01, 0.5);
            g_volObject->m_maxCorner.y(value);
            g_volObject->m_minCorner.y(-value);
            g_volObject->m_maxTextureCoord.y(0.5 + value);
            g_volObject->m_minTextureCoord.y(0.5 - value);
            cout << "> Increase size along Y axis.                            \r";
        }

        // option - reduce size along Z axis
        else if (a_key == GLFW_KEY_8) {
            double value = cClamp((g_volObject->m_maxCorner.z() - 0.005), 0.01, 0.5);
            g_volObject->m_maxCorner.z(value);
            g_volObject->m_minCorner.z(-value);
            g_volObject->m_maxTextureCoord.z(0.5 + value);
            g_volObject->m_minTextureCoord.z(0.5 - value);
            cout << "> Reduce size along Z axis.                            \r";
        }

        // option - increase size along Z axis
        else if (a_key == GLFW_KEY_9) {
            double value = cClamp((g_volObject->m_maxCorner.z() + 0.005), 0.01, 0.5);
            g_volObject->m_maxCorner.z(value);
            g_volObject->m_minCorner.z(-value);
            g_volObject->m_maxTextureCoord.z(0.5 + value);
            g_volObject->m_minTextureCoord.z(0.5 - value);
            cout << "> Increase size along Z axis.                            \r";
        }
        // option - decrease quality of graphic rendering
        else if (a_key == GLFW_KEY_L) {
            double value = g_volObject->getQuality();
            g_volObject->setQuality(value - 0.01);
            cout << "> Quality set to " << cStr(g_volObject->getQuality(), 1) << "                            \r";
        }

        // option - increase quality of graphic rendering
        else if (a_key == GLFW_KEY_U) {
            double value = g_volObject->getQuality();
            g_volObject->setQuality(value + 0.01);
            cout << "> Quality set to " << cStr(g_volObject->getQuality(), 1) << "                            \r";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_UP) {
            double value = g_volObject->getOpacityThreshold();
            g_volObject->setOpacityThreshold(value + 0.01);
            cout << "> Opacity Threshold set to " << cStr(g_volObject->getOpacityThreshold(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_DOWN) {
            double value = g_volObject->getOpacityThreshold();
            g_volObject->setOpacityThreshold(value - 0.01);
            cout << "> Opacity Threshold set to " << cStr(g_volObject->getOpacityThreshold(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_RIGHT) {
            double value = g_volObject->getIsosurfaceValue();
            g_volObject->setIsosurfaceValue(value + 0.01);
            cout << "> Isosurface Threshold set to " << cStr(g_volObject->getIsosurfaceValue(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_LEFT) {
            double value = g_volObject->getIsosurfaceValue();
            g_volObject->setIsosurfaceValue(value - 0.01);
            cout << "> Isosurface Threshold set to " << cStr(g_volObject->getIsosurfaceValue(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_ENTER) {
            g_renderingMode++;
            if (g_renderingMode > 7) {
                g_renderingMode = 0;
            }
            switch (g_renderingMode) {
            case 0:
                g_volObject->setRenderingModeBasic();
                std::cerr << "setRenderingModeBasic" << std::endl;
                break;
            case 1:
                g_volObject->setRenderingModeVoxelColors();
                std::cerr << "setRenderingModeVoxelColors" << std::endl;
                break;
            case 2:
                g_volObject->setRenderingModeVoxelColorMap();
                std::cerr << "setRenderingModeVoxelColorMap" << std::endl;
                break;
            case 3:
                g_volObject->setRenderingModeIsosurfaceColors();
                std::cerr << "setRenderingModeIsosurfaceColors" << std::endl;
                break;
            case 4:
                g_volObject->setRenderingModeIsosurfaceMaterial();
                std::cerr << "setRenderingModeIsosurfaceMaterial" << std::endl;
                break;
            case 5:
                g_volObject->setRenderingModeIsosurfaceColorMap();
                std::cerr << "setRenderingModeIsosurfaceColorMap" << std::endl;
                break;
            case 6:
                g_volObject->setRenderingModeDVRColorMap();
                std::cerr << "setRenderingModeDVRColorMap" << std::endl;
                break;
            case 7:
                g_volObject->setRenderingModeCustom();
                std::cerr << "setRenderingModeCustom" << std::endl;
                break;
            default:
                break;
            }
        } else if (a_key == GLFW_KEY_PAGE_UP) {
            g_opticalDensity += 0.1;
            g_volObject->setOpticalDensity(g_opticalDensity);
            cout << "> Optical Density set to " << cStr(g_opticalDensity, 1) << "                            \n";
        } else if (a_key == GLFW_KEY_PAGE_DOWN) {
            g_opticalDensity -= 0.1;
            g_volObject->setOpticalDensity(g_opticalDensity);
            cout << "> Optical Density set to " << cStr(g_opticalDensity, 1) << "                            \n";
        } else if (a_key == GLFW_KEY_HOME) {
            float val = g_volObject->getOpacityThreshold();
            g_volObject->setOpacityThreshold(val + 0.1);
            cout << "> Optical Threshold set to " << cStr(g_volObject->getOpacityThreshold(), 1)
                 << "                            \n";
        } else if (a_key == GLFW_KEY_END) {
            float val = g_volObject->getOpacityThreshold();
            g_volObject->setOpacityThreshold(val - 0.1);
            cout << "> Optical Threshold set to " << cStr(g_volObject->getOpacityThreshold(), 1)
                 << "                            \n";
        }

        // controls rotational motion of tool
        else if(a_key == GLFW_KEY_KP_5) {

            cVector3d rotDir(0, 1, 0) ;
            incrementDeviceRot(rotDir);
        }

        else if(a_key == GLFW_KEY_KP_8) {

            cVector3d rotDir(0, -1, 0);
            incrementDeviceRot(rotDir);
        }

        else if(a_key == GLFW_KEY_KP_4) {

            cVector3d rotDir(0, 0, -1);
            incrementDeviceRot(rotDir);
        }

        else if(a_key == GLFW_KEY_KP_6) {

            cVector3d rotDir(0, 0, 1);
            incrementDeviceRot(rotDir);
        }

        // toggles the functionality of sudden jumping of drill mesh towards the followSphere
        else if(a_key == GLFW_KEY_X){

            if(g_suddenJump)
            {
                g_suddenJump = false;
            }

            else
            {
                g_suddenJump = true;
            }
        }

        // toggles the visibility of drill mesh in the scene
        else if (a_key == GLFW_KEY_B){
            g_showDrill = !g_showDrill;
            g_drillRigidBody->m_visualMesh->setShowEnabled(g_showDrill);
            g_burrMesh->setShowEnabled(g_showDrill);

        }

        // toggles size of drill burr/tip tool cursor
        else if (a_key == GLFW_KEY_C){
            changeDrillSize();
        }
    }

}


void afVolmetricDrillingPlugin::mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes){

}

void afVolmetricDrillingPlugin::mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos){

}

void afVolmetricDrillingPlugin::reset(){

}

bool afVolmetricDrillingPlugin::close()
{
    for(auto tool : g_toolCursorList)
    {
        tool->stop();
    }

    delete g_deviceHandler;
}
