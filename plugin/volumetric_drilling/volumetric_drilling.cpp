//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2022, AMBF
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

using namespace std;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace p_opt = boost::program_options;


afVolmetricDrillingPlugin::afVolmetricDrillingPlugin(){

}

int afVolmetricDrillingPlugin::init(int argc, char **argv, const afWorldPtr a_afWorld){
    p_opt::options_description cmd_opts("drilling_simulator Command Line Options");
    cmd_opts.add_options()
            ("info", "Show Info")
            ("nt", p_opt::value<int>()->default_value(8), "Number Tool Cursors to Load. Default 8")
            ("ds", p_opt::value<float>()->default_value(0.026), "Offset between shaft tool cursors. Default 0.026")
            ("vm", p_opt::value<string>()->default_value("00ShinyWhite.jpg"), "Volume's Matcap Filename (Should be placed in the ./resources/matcap/ folder)")
            ("dm", p_opt::value<string>()->default_value("dark_metal_brushed.jpg"), "Drill's Matcap Filename (Should be placed in ./resources/matcap/ folder)")
            ("fp", p_opt::value<string>()->default_value("/dev/input/js0"), "Footpedal joystick input file description E.g. /dev/input/js0)")
            ("mute", p_opt::value<bool>()->default_value(false), "Mute")
            ("gcdr", p_opt::value<double>()->default_value(30.0), "Gaze Calibration Marker Motion Duration");

    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);

    if(var_map.count("info")){
        std::cout<< cmd_opts << std::endl;
        return -1;
    }

    string file_path = __FILE__;
    string current_filepath = file_path.substr(0, file_path.rfind("/"));

    string volume_matcap = var_map["vm"].as<string>();
    string footpedal_fd = var_map["fp"].as<string>();

    m_zeroColor = cColorb(0x00, 0x00, 0x00, 0x00);

    m_boneColor = cColorb(255, 249, 219, 255);

    m_storedColor = cColorb(0x00, 0x00, 0x00, 0x00);

    m_worldPtr = a_afWorld;

    // Get first camera
    m_mainCamera = findAndAppendCamera("main_camera");
    m_cameraL = findAndAppendCamera("cameraL");
    m_cameraR = findAndAppendCamera("cameraR");
    m_stereoCamera = findAndAppendCamera("stereoLR");

    if (m_cameras.size() == 0){
        cerr << "ERROR! NO CAMERAS FOUND. " << endl;
        return -1;
    }

    if (!m_mainCamera){
        cerr << "INFO! FAILED TO LOAD main_camera, taking the first camera from world " << endl;
        m_mainCamera = m_worldPtr->getCameras()[0];
    }

    if (m_stereoCamera){
        makeVRWindowFullscreen(m_stereoCamera);
    }

    m_panelManager.addCamera(m_mainCamera);
    if (m_stereoCamera){
        m_stereoCamera->getInternalCamera()->m_stereoOffsetW = 0.1;
        m_panelManager.addCamera(m_stereoCamera);
    }

    if (!m_drillManager.init(a_afWorld, &m_panelManager, var_map)){
        return -1;
    }

    m_volumeObject = m_worldPtr->getVolume("mastoidectomy_volume");
    if (!m_volumeObject){
        cerr << "ERROR! FAILED TO FIND VOLUME NAMED " << "mastoidectomy_volume" << endl;
        return -1;
    }
    else{
        m_voxelObj = m_volumeObject->getInternalVolume();

        m_maxVolCorner = m_voxelObj->m_maxCorner;
        m_minVolCorner = m_voxelObj->m_minCorner;
        m_maxTexCoord = m_voxelObj->m_maxTextureCoord;
        m_minTexCoord = m_voxelObj->m_minTextureCoord;

        m_textureCoordScale(0) = (m_maxTexCoord.x() - m_minTexCoord.x()) / (m_maxVolCorner.x() - m_minVolCorner.x());
        m_textureCoordScale(1) = (m_maxTexCoord.y() - m_minTexCoord.y()) / (m_maxVolCorner.y() - m_minVolCorner.y());
        m_textureCoordScale(2) = (m_maxTexCoord.z() - m_minTexCoord.z()) / (m_maxVolCorner.z() - m_minVolCorner.z());
    }

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = m_drillManager.m_toolCursorList[0]->getWorkspaceScaleFactor();

    // stiffness properties
    double maxStiffness = m_drillManager.m_hapticDevice->getSpecifications().m_maxLinearStiffness / workspaceScaleFactor;

    // Set voxels surface contact properties
    m_voxelObj->m_material->setStiffness(2.0*maxStiffness);
    m_voxelObj->m_material->setDamping(0.0);
    m_voxelObj->m_material->setDynamicFriction(0.0);
    m_voxelObj->setUseMaterial(true);

    initializeLabels();

    // Publish Volume Info
    cTransform volPose = m_volumeObject->getLocalTransform();
    cVector3d dims = m_volumeObject->getDimensions();
    cVector3d count = m_volumeObject->getVoxelCount();
    m_drillManager.m_drillingPub->setVolumeInfo(volPose, dims, count);
    m_drillManager.m_drillingPub->publishVolumeInfo(m_worldPtr->getCurrentTimeStamp());

    string volumeMatcapFilepath = current_filepath + "/../../resources/matcap/" + volume_matcap;
    cTexture2dPtr volMatCap = cTexture2d::create();
    if(volMatCap->loadFromFile(volumeMatcapFilepath)){
        m_volumeObject->getInternalVolume()->m_aoTexture = volMatCap;
        m_volumeObject->getInternalVolume()->m_aoTexture->setTextureUnit(GL_TEXTURE5);
        cerr << "SUCCESFULLY LOADED VOLUME'S MATCAP TEXTURE" << endl;
    }
    else{
        cerr << "FAILED TO LOAD VOLUME'S MATCAP TEXTURE" << endl;
    }



    cBackground* background = new cBackground();
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.6f, 0.6f, 0.6f),
                                cColorf(0.6f, 0.6f, 0.6f));
    m_mainCamera->getBackLayer()->addChild(background);

    if (m_footpedal.init(footpedal_fd) != -1){
        cerr << "INFO! FOUND FOOTPEDAL INTERFACE \n";
    }
    else{
        cerr << "WARNING! NO FOOTPEDAL INTERFACE FOUND \n";
    }

    m_gazeMarkerController.init(m_worldPtr, &m_panelManager, var_map);

    return 1;
}

void afVolmetricDrillingPlugin::graphicsUpdate(){

    // update region of voxels to be updated
    if (m_flagMarkVolumeForUpdate)
    {
        m_mutexVoxel.acquire();
        cVector3d min = m_volumeUpdate.m_min;
        cVector3d max = m_volumeUpdate.m_max;
        m_volumeUpdate.setEmpty();
        m_mutexVoxel.release();
        ((cTexture3d*)m_voxelObj->m_texture.get())->markForPartialUpdate(min, max);
        m_flagMarkVolumeForUpdate = false;
    }
    m_volumeObject->getShaderProgram()->setUniformi("uMatcapMap", C_TU_AO);
    m_volumeObject->getShaderProgram()->setUniformi("shadowMap", C_TU_SHADOWMAP);

    static double last_time = 0.0;

    double dt = m_worldPtr->getWallTime() - last_time;
    last_time = m_worldPtr->getWallTime();

    m_gazeMarkerController.update(dt);
    updateButtons();
    m_panelManager.update();
}

void afVolmetricDrillingPlugin::physicsUpdate(double dt){

    m_worldPtr->getChaiWorld()->computeGlobalPositions(true);

    m_drillManager.update(dt);

    if (m_drillManager.m_toolCursorList[0]->isInContact(m_voxelObj) && m_drillManager.m_targetToolCursorIdx == 0 /*&& (userSwitches == 2)*/)
    {

        // retrieve contact event
        cCollisionEvent* contact = m_drillManager.m_toolCursorList[0]->m_hapticPoint->getCollisionEvent(0);

        cVector3d orig(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ); // This is the closest voxel index to the drill tip

        m_voxelObj->m_texture->m_image->getVoxelColor(uint(orig.x()), uint(orig.y()), uint(orig.z()), m_storedColor);

        if (m_storedColor != m_zeroColor){
            if (m_drillManager.m_isOn){
                int removalCount = cMin(m_drillManager.m_activeDrill->getVoxelRemovalThreshold(), (int)contact->m_events.size());
                m_drillManager.m_drillingPub->clearVoxelMsg();
                m_mutexVoxel.acquire();
                for (int cIdx = 0 ; cIdx < removalCount ; cIdx++){
                    cVector3d ct(contact->m_events[cIdx].m_voxelIndexX, contact->m_events[cIdx].m_voxelIndexY, contact->m_events[cIdx].m_voxelIndexZ);
                    cColorb colorb;
                    m_voxelObj->m_texture->m_image->getVoxelColor(uint(ct.x()), uint(ct.y()), uint(ct.z()), colorb);
                    cColorf colorf = colorb.getColorf();
                    m_drillManager.m_drillingPub->appendToVoxelMsg(ct, colorf);
                    m_voxelObj->m_texture->m_image->setVoxelColor(uint(ct.x()), uint(ct.y()), uint(ct.z()), m_zeroColor);
                    m_volumeUpdate.enclose(cVector3d(uint(ct.x()), uint(ct.y()), uint(ct.z())));
                }
                m_mutexVoxel.release();

                //Publisher for voxels removed
                m_drillManager.m_drillingPub->publishVoxelMsg(m_worldPtr->getCurrentTimeStamp());
            }

            //Publisher for voxels removed
            if(m_storedColor != m_boneColor)
            {
                m_panelManager.setVisible(m_warningLabel, true);
            }
        }

        // mark voxel for update

        m_flagMarkVolumeForUpdate = true;
    }
    // remove warning panel
    else
    {
        m_panelManager.setVisible(m_warningLabel, false);
    }
    // compute interaction forces
    for(int i = 0 ; i < m_drillManager.m_toolCursorList.size() ; i++){
        m_drillManager.m_toolCursorList[i]->computeInteractionForces();
    }

    // check if device remains stuck inside voxel object
    // Also orient the force to match the camera rotation
    cVector3d force = cTranspose(m_mainCamera->getLocalRot()) * m_drillManager.m_targetToolCursor->getDeviceLocalForce();
    if (m_drillManager.m_isOn){
        force += (cVector3d(1.0, 1.0, 1.0) * m_waveGenerator.generate(dt));
    }
    m_drillManager.m_toolCursorList[0]->setDeviceLocalForce(force);
    m_drillManager.m_drillingPub->publishForceFeedback(force, force, m_worldPtr->getCurrentTimeStamp());
    double maxF = m_drillManager.m_hapticDevice->getSpecifications().m_maxLinearForce;
    double force_mag = cClamp(force.length(), 0.0, maxF);
    m_drillManager.setAudioPitch(3.0 - force_mag / maxF);

    if (m_flagStart)
    {
        if (force.length() != 0.0)
        {
            m_counter = 0;
        }
        else
        {
            m_counter++;
            if (m_counter > 10)
                m_flagStart = false;
        }
    }
    else
    {
        if (force.length() > 10.0)
        {
            m_flagStart = true;
        }
    }

    /////////////////////////////////////////////////////////////////////////
    // FINALIZE
    /////////////////////////////////////////////////////////////////////////

    // send forces to haptic device
    if (m_drillManager.getOverrideControl() == false){
        m_drillManager.m_toolCursorList[0]->applyToDevice();
    }

}



void afVolmetricDrillingPlugin::sliceVolume(int axisIdx, double delta)
{
    string axis_str = "";
    if (axisIdx == 0){
        axis_str = "X";
    }
    else if (axisIdx == 1){
        axis_str = "Y";
    }
    else if (axisIdx == 2){
        axis_str = "Z";
    }
    else{
        cerr << "ERROR! Volume axis index should be either 0, 1 or 2" << endl;
        return;
    }

    string delta_dir_str = "";
    if (delta > 0){
        delta_dir_str = "Increase";
    }
    else{
        delta_dir_str = "Decrease";
    }

    double value = cClamp((m_voxelObj->m_maxCorner(axisIdx) + delta), 0.01, m_maxVolCorner(axisIdx));
    m_voxelObj->m_maxCorner(axisIdx) = value;
    m_voxelObj->m_minCorner(axisIdx) = -value;
    m_voxelObj->m_maxTextureCoord(axisIdx) = 0.5 + value * m_textureCoordScale(axisIdx);
    m_voxelObj->m_minTextureCoord(axisIdx) = 0.5 - value * m_textureCoordScale(axisIdx);

    cerr << "> " << delta_dir_str << " Volume size along " << axis_str << " axis.                            \r";
}


void afVolmetricDrillingPlugin::makeVRWindowFullscreen(afCameraPtr vrCam, int monitor_number)
{
    int count;
    GLFWmonitor** monitors = glfwGetMonitors(&count);

    if (monitor_number >= 0){
        if (monitor_number >= count){
            cerr << "WARNING! SPECIFIED MONITOR NUMBER " << monitor_number <<
                    " FOR VR IS GREATER THAN NUMBER OF DISPLAYS. IGNORING" << endl;
            return;
        }
        vrCam->m_monitor = monitors[monitor_number];
    }

    for (int cnt = 0 ; cnt < count ; cnt++){
        string monitor_name = glfwGetMonitorName(monitors[cnt]);
        cerr << "\t Monitor Number: " << cnt << " | Name: " << monitor_name << endl;
    }


    const GLFWvidmode* mode = glfwGetVideoMode(vrCam->m_monitor);
    int w = 0.9 * mode->width;
    int h = 0.9 * mode->height;
    int x = 0.7 * (mode->width - w);
    int y = 0.7 * (mode->height - h);
    int xpos, ypos;
    glfwGetMonitorPos(vrCam->m_monitor, &xpos, &ypos);
    x += xpos; y += ypos;
    glfwSetWindowPos(vrCam->m_window, x, y);
    glfwSetWindowSize(vrCam->m_window, w, h);
    vrCam->m_width = w;
    vrCam->m_height = h;
    glfwSwapInterval(0);
    cerr << "\t\t Setting the Window on the VR Monitor to fullscreen \n" ;
}

void afVolmetricDrillingPlugin::updateButtons(){
    m_footpedal.poll();

    if (m_footpedal.isChangeBurrSizePressed()){
        m_drillManager.cycleDrillTypes();
    }

    m_drillManager.m_isOn = m_footpedal.isDrillOn();

    m_drillManager.m_camClutch = false;
    m_drillManager.m_deviceClutch = false;

    if (m_footpedal.isAvailable()){
        m_drillManager.m_camClutch = m_footpedal.isCamClutchPressed();
        m_drillManager.m_deviceClutch = m_footpedal.isDeviceClutchPressed();

    }

    if (m_drillManager.m_hapticDevice->isDeviceAvailable()){
        bool val1, val2;
        m_drillManager.m_hapticDevice->getUserSwitch(1, val1);
        m_drillManager.m_hapticDevice->getUserSwitch(0, val2);

        m_drillManager.m_camClutch |= val1;
        m_drillManager.m_deviceClutch |= val2;
    }
}

afCameraPtr afVolmetricDrillingPlugin::findAndAppendCamera(string cam_name){
    afCameraPtr cam = m_worldPtr->getCamera(cam_name);
    if (cam){
        cerr << "INFO! GOT CAMERA: " << cam->getName() << endl;
        m_cameras[cam_name] = cam;
    }
    else{
        cerr << "WARNING! CAMERA NOT FOUND " << cam_name << endl;
    }
    return cam;
}

void afVolmetricDrillingPlugin::initializeLabels()
{
    // create a font
    cFontPtr font = NEW_CFONTCALIBRI40();

    m_warningLabel = new cLabel(font);
    m_warningLabel->m_fontColor.setWhite();
    m_warningLabel->setText("WARNING! Critical Region Detected");
    m_warningLabel->setCornerRadius(5, 5, 5, 5);
    m_warningLabel->setShowPanel(true);
    m_warningLabel->setColor(cColorf(0.6, 0., 0., 1.0));
    m_warningLabel->setTransparencyLevel(0.6);

    m_panelManager.addPanel(m_warningLabel, 0.5, 0.5, PanelReferenceOrigin::CENTER, PanelReferenceType::NORMALIZED);

    m_volumeSmoothingLabel = new cLabel(font);
    m_volumeSmoothingLabel->m_fontColor.setRed();
    m_volumeSmoothingLabel->setFontScale(.5);
    m_volumeSmoothingLabel->setColor(cColorf(0.6, 0., 0., 1.0));
    m_volumeSmoothingLabel->setText("[ALT+S] Volume Smoothing: DISABLED");

    m_panelManager.addPanel(m_volumeSmoothingLabel, 20, 10, PanelReferenceOrigin::LOWER_LEFT, PanelReferenceType::PIXEL);
}


void afVolmetricDrillingPlugin::keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods) {
    if (a_mods == GLFW_MOD_CONTROL){

        double rate = m_drillManager.m_drillRate;
        // controls linear motion of tool
        if (a_key == GLFW_KEY_W) {

            cVector3d dir = m_mainCamera->getUpVector() * rate;
            m_drillManager.incrementDevicePos(dir);
        }

        else if (a_key == GLFW_KEY_D) {

            cVector3d dir = m_mainCamera->getRightVector() * rate;
            m_drillManager.incrementDevicePos(dir);

        }

        else if (a_key == GLFW_KEY_S) {

            cVector3d dir = m_mainCamera->getUpVector() * rate;
            m_drillManager.incrementDevicePos(-dir);

        }

        else if (a_key == GLFW_KEY_A) {

            cVector3d dir = m_mainCamera->getRightVector() * rate;
            m_drillManager.incrementDevicePos(-dir);

        }

        else if (a_key == GLFW_KEY_K) {

            cVector3d dir = m_mainCamera->getLookVector() * rate;
            m_drillManager.incrementDevicePos(-dir);

        }

        else if (a_key == GLFW_KEY_I) {

            cVector3d dir = m_mainCamera->getLookVector() * rate;
            m_drillManager.incrementDevicePos(dir);
        }

        else if (a_key == GLFW_KEY_O) {

            m_drillManager.setOverrideControl(!m_drillManager.getOverrideControl());
        }

        else if (a_key == GLFW_KEY_C) {
            m_drillManager.m_showGoalProxySpheres = !m_drillManager.m_showGoalProxySpheres;
            for (int i = 0 ; i < m_drillManager.m_toolCursorList.size() ; i++){
                m_drillManager.m_toolCursorList[i]->m_hapticPoint->setShow(m_drillManager.m_showGoalProxySpheres, m_drillManager.m_showGoalProxySpheres);
            }
        }

        // option - polygonize model and save to file
        else if (a_key == GLFW_KEY_P) {
            cMultiMesh *surface = new cMultiMesh;
            m_voxelObj->polygonize(surface, 0.01, 0.01, 0.01);
            double SCALE = 0.1;
            double METERS_TO_MILLIMETERS = 1000.0;
            surface->scale(SCALE * METERS_TO_MILLIMETERS);
            surface->setUseVertexColors(true);
            surface->saveToFile("volume.obj");
            cout << "> Volume has been polygonized and saved to disk                            \r";
            delete surface;
        }

        // toggles size of drill burr/tip tool cursor
        else if (a_key == GLFW_KEY_N){
            cerr << "INFO! RESETTING THE VOLUME" << endl;
            m_volumeObject->reset();
        }

        else if (a_key == GLFW_KEY_G){
            m_gazeMarkerController.restart();
        }

        else if (a_key == GLFW_KEY_E){
            static int enableShadow = 0;
            enableShadow = ! enableShadow;
            cerr << "INFO! TOGGLING SHADOW MAP " << enableShadow << endl;
            m_volumeObject->getShaderProgram()->setUniformi("uEnableShadow", enableShadow);
        }

        else if (a_key == GLFW_KEY_PAGE_UP){
            if(m_stereoCamera){
                m_stereoCamera->getInternalCamera()->m_stereoOffsetW += 0.05;
                cerr << "INFO! SETTING STEREO OFFSET W " <<
                        m_stereoCamera->getInternalCamera()->m_stereoOffsetW << endl;
            }
        }

        else if (a_key == GLFW_KEY_PAGE_DOWN){
            if(m_stereoCamera){
                m_stereoCamera->getInternalCamera()->m_stereoOffsetW -= 0.05;
                cerr << "INFO! SETTING STEREO OFFSET W " <<
                        m_stereoCamera->getInternalCamera()->m_stereoOffsetW << endl;
            }
        }

        else if (a_key == GLFW_KEY_T){
            int val = m_drillManager.m_activeDrill->getVoxelRemovalThreshold() + 1;
            m_drillManager.m_activeDrill->setVoxelRemvalThreshold(val);
            cerr << "INFO! REMOVAL THRESHOLD " << val << endl;
        }

        // Reset the drill pose
        if (a_key == GLFW_KEY_R){
            cerr << "INFO! RESETTING THE DRILL" << endl;
            m_drillManager.reset();
        }
    }
    else if(a_mods == GLFW_MOD_ALT){
        // Toggle Volume Smoothing
        if (a_key == GLFW_KEY_S){
            m_enableVolumeSmoothing = !m_enableVolumeSmoothing;
            cerr << "INFO! ENABLE VOLUME SMOOTHING: " << m_enableVolumeSmoothing << endl;
            m_volumeObject->getShaderProgram()->setUniformi("uSmoothVolume", m_enableVolumeSmoothing);
            m_volumeObject->getShaderProgram()->setUniformi("uSmoothingLevel", m_volumeSmoothingLevel);

        }
        else if(a_key == GLFW_KEY_UP){
            m_volumeSmoothingLevel = cClamp(m_volumeSmoothingLevel+1, 1, 10);
            cerr << "INFO! SETTING SMOOTHING LEVEL " << m_volumeSmoothingLevel << endl;
            m_volumeObject->getShaderProgram()->setUniformi("uSmoothingLevel", m_volumeSmoothingLevel);
        }
        else if(a_key == GLFW_KEY_DOWN){
            m_volumeSmoothingLevel = cClamp(m_volumeSmoothingLevel-1, 1, 10);
            cerr << "INFO! SETTING SMOOTHING LEVEL " << m_volumeSmoothingLevel << endl;
            m_volumeObject->getShaderProgram()->setUniformi("uSmoothingLevel", m_volumeSmoothingLevel);
        }

        else if (a_key == GLFW_KEY_T){
            int val = cMax(1, m_drillManager.m_activeDrill->getVoxelRemovalThreshold() - 1);
            m_drillManager.m_activeDrill->setVoxelRemvalThreshold(val);
            cerr << "INFO! REMOVAL THRESHOLD " << val << endl;
        }

        std::string text = "[ALT+S] Volume Smoothing: " + std::string(m_enableVolumeSmoothing ? "ENABLED" : "DISABLED");
        cColorf color;
        color.setRed();
        if (m_enableVolumeSmoothing){
            text += " ( LEVEL: " + to_string(m_volumeSmoothingLevel) + ")";
            color.setGreen();
        }
        m_panelManager.setText(m_volumeSmoothingLabel, text);
        m_panelManager.setFontColor(m_volumeSmoothingLabel, color);
    }
    else{

        // option - reduce size along X axis
        if (a_key == GLFW_KEY_4) {
            sliceVolume(0, -0.005);
        }

        // option - increase size along X axis
        else if (a_key == GLFW_KEY_5) {
            sliceVolume(0, 0.005);
        }

        // option - reduce size along Y axis
        else if (a_key == GLFW_KEY_6) {
            sliceVolume(1, -0.005);
        }

        // option - increase size along Y axis
        else if (a_key == GLFW_KEY_7) {
            sliceVolume(1, 0.005);
        }

        // option - reduce size along Z axis
        else if (a_key == GLFW_KEY_8) {
            sliceVolume(2, -0.005);
        }

        // option - increase size along Z axis
        else if (a_key == GLFW_KEY_9) {
            sliceVolume(2, 0.005);
        }
        // option - decrease quality of graphic rendering
        else if (a_key == GLFW_KEY_L) {
            double value = m_voxelObj->getQuality();
            m_voxelObj->setQuality(value - 0.01);
            cout << "> Quality set to " << cStr(m_voxelObj->getQuality(), 1) << "                            \r";
        }

        // option - increase quality of graphic rendering
        else if (a_key == GLFW_KEY_U) {
            double value = m_voxelObj->getQuality();
            m_voxelObj->setQuality(value + 0.01);
            cout << "> Quality set to " << cStr(m_voxelObj->getQuality(), 1) << "                            \r";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_UP) {
            double value = m_voxelObj->getOpacityThreshold();
            m_voxelObj->setOpacityThreshold(value + 0.01);
            cout << "> Opacity Threshold set to " << cStr(m_voxelObj->getOpacityThreshold(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_DOWN) {
            double value = m_voxelObj->getOpacityThreshold();
            m_voxelObj->setOpacityThreshold(value - 0.01);
            cout << "> Opacity Threshold set to " << cStr(m_voxelObj->getOpacityThreshold(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_RIGHT) {
            double value = m_voxelObj->getIsosurfaceValue();
            m_voxelObj->setIsosurfaceValue(value + 0.01);
            cout << "> Isosurface Threshold set to " << cStr(m_voxelObj->getIsosurfaceValue(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_LEFT) {
            double value = m_voxelObj->getIsosurfaceValue();
            m_voxelObj->setIsosurfaceValue(value - 0.01);
            cout << "> Isosurface Threshold set to " << cStr(m_voxelObj->getIsosurfaceValue(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_ENTER) {
            m_renderingMode++;
            if (m_renderingMode > 7) {
                m_renderingMode = 0;
            }
            switch (m_renderingMode) {
            case 0:
                m_voxelObj->setRenderingModeBasic();
                std::cerr << "setRenderingModeBasic" << std::endl;
                break;
            case 1:
                m_voxelObj->setRenderingModeVoxelColors();
                std::cerr << "setRenderingModeVoxelColors" << std::endl;
                break;
            case 2:
                m_voxelObj->setRenderingModeVoxelColorMap();
                std::cerr << "setRenderingModeVoxelColorMap" << std::endl;
                break;
            case 3:
                m_voxelObj->setRenderingModeIsosurfaceColors();
                std::cerr << "setRenderingModeIsosurfaceColors" << std::endl;
                break;
            case 4:
                m_voxelObj->setRenderingModeIsosurfaceMaterial();
                std::cerr << "setRenderingModeIsosurfaceMaterial" << std::endl;
                break;
            case 5:
                m_voxelObj->setRenderingModeIsosurfaceColorMap();
                std::cerr << "setRenderingModeIsosurfaceColorMap" << std::endl;
                break;
            case 6:
                m_voxelObj->setRenderingModeDVRColorMap();
                std::cerr << "setRenderingModeDVRColorMap" << std::endl;
                break;
            case 7:
                m_voxelObj->setRenderingModeCustom();
                std::cerr << "setRenderingModeCustom" << std::endl;
                break;
            default:
                break;
            }
        } else if (a_key == GLFW_KEY_PAGE_UP) {
            m_opticalDensity += 0.1;
            m_voxelObj->setOpticalDensity(m_opticalDensity);
            cout << "> Optical Density set to " << cStr(m_opticalDensity, 1) << "                            \n";
        } else if (a_key == GLFW_KEY_PAGE_DOWN) {
            m_opticalDensity -= 0.1;
            m_voxelObj->setOpticalDensity(m_opticalDensity);
            cout << "> Optical Density set to " << cStr(m_opticalDensity, 1) << "                            \n";
        } else if (a_key == GLFW_KEY_HOME) {
            float val = m_voxelObj->getOpacityThreshold();
            m_voxelObj->setOpacityThreshold(val + 0.1);
            cout << "> Optical Threshold set to " << cStr(m_voxelObj->getOpacityThreshold(), 1)
                 << "                            \n";
        } else if (a_key == GLFW_KEY_END) {
            float val = m_voxelObj->getOpacityThreshold();
            m_voxelObj->setOpacityThreshold(val - 0.1);
            cout << "> Optical Threshold set to " << cStr(m_voxelObj->getOpacityThreshold(), 1)
                 << "                            \n";
        }

        // controls rotational motion of tool
        else if(a_key == GLFW_KEY_KP_5) {

            cVector3d rotDir(0, 1, 0) ;
            m_drillManager.incrementDeviceRot(rotDir);
        }

        else if(a_key == GLFW_KEY_KP_8) {

            cVector3d rotDir(0, -1, 0);
            m_drillManager.incrementDeviceRot(rotDir);
        }

        else if(a_key == GLFW_KEY_KP_4) {

            cVector3d rotDir(0, 0, -1);
            m_drillManager.incrementDeviceRot(rotDir);
        }

        else if(a_key == GLFW_KEY_KP_6) {

            cVector3d rotDir(0, 0, 1);
            m_drillManager.incrementDeviceRot(rotDir);
        }

        // toggles the functionality of sudden jumping of drill mesh towards the followSphere
        else if(a_key == GLFW_KEY_X){

            m_drillManager.m_suddenJump = !m_drillManager.m_suddenJump;
        }

        // toggles the visibility of drill mesh in the scene
        else if (a_key == GLFW_KEY_B){
            m_drillManager.m_show = !m_drillManager.m_show;
            m_drillManager.m_activeDrill->m_rigidBody->m_visualMesh->setShowEnabled(m_drillManager.m_show);
            m_drillManager.m_burrMesh->setShowEnabled(m_drillManager.m_show);

        }

        // toggles size of drill burr/tip tool cursor
        else if (a_key == GLFW_KEY_C){
            m_drillManager.cycleDrillTypes();
        }

        else if (a_key == GLFW_KEY_KP_ADD){
            if (m_cameraL){
                m_cameraL->setLocalPos(m_cameraL->getLocalPos() - cVector3d(0., 0.001, 0.));
            }
            if (m_cameraR){
                m_cameraR->setLocalPos(m_cameraR->getLocalPos() + cVector3d(0., 0.001, 0.));
            }

            if (m_stereoCamera){
                double stereo_sep = m_stereoCamera->getInternalCamera()->getStereoEyeSeparation();
                m_stereoCamera->getInternalCamera()->setStereoEyeSeparation(stereo_sep + 0.002);
            }
        }
        else if (a_key == GLFW_KEY_KP_SUBTRACT){
            if (m_cameraL){
                m_cameraL->setLocalPos(m_cameraL->getLocalPos() + cVector3d(0., 0.001, 0.));
            }
            if (m_cameraR){
                m_cameraR->setLocalPos(m_cameraR->getLocalPos() - cVector3d(0., 0.001, 0.));
            }

            if (m_stereoCamera){
                double stereo_sep = m_stereoCamera->getInternalCamera()->getStereoEyeSeparation();
                m_stereoCamera->getInternalCamera()->setStereoEyeSeparation(stereo_sep - 0.002);
            }
        }
    }

}


void afVolmetricDrillingPlugin::mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes){

}

void afVolmetricDrillingPlugin::mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos){

}

void afVolmetricDrillingPlugin::reset(){
    cerr << "INFO! PLUGIN RESET CALLED" << endl;
    m_drillManager.reset();
}

bool afVolmetricDrillingPlugin::close()
{
    m_drillManager.cleanup();
    return true;
}
