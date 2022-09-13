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

string g_current_filepath;

DrillManager::DrillManager(){
    m_units_mmToSim = 0.01007;
}

void DrillManager::cleanup()
{
    for(auto tool : m_toolCursorList)
        {
            tool->stop();
        }

        if (m_audioSource){
            delete m_audioSource;
        }
        if (m_audioBuffer){
            delete m_audioBuffer;
        }
        if (m_audioDevice){
            m_camera->getInternalCamera()->detachAudioDevice();
            delete m_audioDevice;
        }
        delete m_deviceHandler;
}

int DrillManager::init(afWorldPtr a_worldPtr, afCameraPtr a_cameraPtr, p_opt::variables_map& var_map){
    // importing drill model

    int nt = var_map["nt"].as<int>();
    bool mute = var_map["mute"].as<bool>();

    m_camera = a_cameraPtr;

    if (nt > 0 && nt <= 8){
        m_toolCursorList.resize(nt);
    }
    else{
        cerr << "ERROR! VALID NUMBER OF TOOL CURSORS ARE BETWEEN 1 - 8. Specified value = " << nt << endl;
        return -1;
    }

    m_drillReferenceBody = a_worldPtr->getRigidBody("mastoidectomy_drill");

    if (!m_drillReferenceBody){
        cerr << "ERROR! FAILED TO FIND REFERENCE DRILL RIGID BODY " << "mastoidectomy_drill" << endl;
        return -1;
    }

    vector<int> drillTypes = {2, 4, 6};

    for (int i = 0 ; i < drillTypes.size() ; i++){
        string drillName = to_string(drillTypes[i]) + "mm";
        afRigidBodyPtr drillRB = a_worldPtr->getRigidBody(drillName);
        if (drillRB){
            Drill* drill = new Drill();
            drill->m_name = drillName;
            drill->m_rigidBody = drillRB;
            drill->m_size = drillTypes[i] * m_units_mmToSim;
            m_drills.push_back(drill);
        }
    }

    if (m_drills.size() == 0){
        cerr << "ERROR! FAILED TO FIND DRILLS WITH PREFIX " << "mastoidectomy_drill" << endl;
        return -1;
    }

    m_activeDrillIdx = 0;
    m_activeDrill = m_drills[m_activeDrillIdx];
    showOnlyActive();

    m_dX = var_map["ds"].as<float>();

    string drill_matcap = var_map["dm"].as<string>();

    m_burrMesh = new cShapeSphere(m_activeDrill->m_size);
    m_burrMesh->setRadius(m_activeDrill->m_size);
    m_burrMesh->m_material->setBlack();
    m_burrMesh->m_material->setShininess(0);
    m_burrMesh->m_material->m_specular.set(0, 0, 0);
    m_burrMesh->setShowEnabled(true);

    a_worldPtr->addSceneObjectToWorld(m_burrMesh);

    initializeLabels();

    updateLabelPositions();

    // Get drills initial pose
    m_T_d_init = m_drillReferenceBody->getLocalTransform();

    m_T_d = m_T_d_init;

    // Set up voxels_removed publisher
    m_drillingPub = new DrillingPublisher("ambf", "volumetric_drilling");

    cTexture2dPtr drillMatCap = cTexture2d::create();
    string drillMatcapFilepath = g_current_filepath + "/../resources/matcap/" + drill_matcap;
    if(drillMatCap->loadFromFile(drillMatcapFilepath)){
        cerr << "SUCCESFULLY LOADED DRILL'S MATCAP TEXTURE" << endl;
        for (int di = 0 ; di < m_drills.size() ; di++){
            if (m_drills[di]->m_rigidBody->getShaderProgram()){
                for (int mi = 0 ; mi < m_drills[di]->m_rigidBody->getVisualObject()->getNumMeshes(); mi++){
                    m_drills[di]->m_rigidBody->getVisualObject()->getMesh(mi)->m_metallicTexture = drillMatCap;
                    m_drills[di]->m_rigidBody->getVisualObject()->getMesh(mi)->m_metallicTexture->setTextureUnit(GL_TEXTURE3);
                }
                m_drills[di]->m_rigidBody->getShaderProgram()->setUniformi("uMatcapMap", C_TU_METALLIC);
            }
        }
    }
    else{
        cerr << "FAILED TO LOAD DRILL'S MATCAP TEXTURE" << endl;
    }

    if (!mute){
        m_audioDevice = new cAudioDevice();
        a_cameraPtr->getInternalCamera()->attachAudioDevice(m_audioDevice);

        m_audioBuffer = new cAudioBuffer();
        string drillAudioFilepath = g_current_filepath + "/../resources/sounds/drill.wav";
        if (m_audioBuffer->loadFromFile(drillAudioFilepath)){
            m_audioSource = new cAudioSource();
            //        m_drillAudioBuffer->convertToMono();
            m_audioSource->setAudioBuffer(m_audioBuffer);
            m_audioSource->setLoop(true);
            m_audioSource->setGain(5.0);
            m_audioState = AudioState::STOPPED;
        }
        else{
            delete m_audioSource;
            delete m_audioBuffer;
            m_audioSource = nullptr;
            m_audioBuffer = nullptr;
            cerr << "FAILED TO LOAD DRILL AUDIO FROM " << drillAudioFilepath << endl;
        }
    }

    // create a haptic device handler
    m_deviceHandler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    m_deviceHandler->getDevice(m_hapticDevice, 0);

    // Initializing tool cursors
    toolCursorInit(a_worldPtr);

    return 1;
}

void DrillManager::update(double dt)
{

    cTransform T_c_w = m_camera->getLocalTransform();

    // If a valid haptic device is found, then it should be available
    if (getOverrideControl()){
        m_T_d = m_drillReferenceBody->getLocalTransform();
    }
    else if(m_hapticDevice->isDeviceAvailable()){
        m_hapticDevice->getTransform(m_T_i);
        m_hapticDevice->getLinearVelocity(m_V_i);
        m_V_i = T_c_w.getLocalRot() * (m_V_i / m_toolCursorList[0]->getWorkspaceScaleFactor());
        m_T_d.setLocalPos(m_T_d.getLocalPos() + (m_V_i * 0.4 * !m_deviceClutch * !m_camClutch));
        m_T_d.setLocalRot(T_c_w.getLocalRot() * m_T_i.getLocalRot());

        // set zero forces when manipulating objects
        if (m_deviceClutch || m_camClutch){
            if (m_camClutch){
                m_camera->setView(T_c_w.getLocalPos() + m_V_i * !m_deviceClutch, m_camera->getTargetPosLocal(), m_camera->getUpVector());
            }
            m_toolCursorList[0]->setDeviceLocalForce(0.0, 0.0, 0.0);
        }
        m_drillReferenceBody->setLocalTransform(m_T_d);
    }

    toolCursorsPosUpdate(m_T_d);

    // check for shaft collision
    checkShaftCollision();

//    if (getOverrideControl() == false){
        // updates position of drill mesh
        updatePoseFromCursors();
//    }

    m_burrMesh->setLocalTransform(m_activeDrill->m_rigidBody->getLocalTransform());

    if (m_isOn && m_audioSource){
        if (m_audioState == AudioState::STOPPED){
            m_audioSource->play();
            m_audioState = AudioState::PLAYING;
        }
    }
    else{
        if (m_audioState == AudioState::PLAYING){
            m_audioSource->stop();
            m_audioState = AudioState::STOPPED;
        }
    }
}

void DrillManager::initializeLabels(){
    // create a font
    cFontPtr font = NEW_CFONTCALIBRI32();

    m_sizeLabel = new cLabel(font);
    m_sizeLabel->setLocalPos(10, 10); // Relative to Panel
    m_sizeLabel->m_fontColor.setBlack();
    m_sizeLabel->setText("Drill Type: " + m_activeDrill->m_name);

    // A panel to display current drill size
    m_sizePanel = new cPanel();
    m_sizePanel->set(m_sizeLabel->getTextWidth() + m_sizeLabel->getLocalPos().x() * 2,
                     m_sizeLabel->getTextHeight() + m_sizeLabel->getLocalPos().y() * 2,
                     10, 10, 10, 10);
    m_sizePanel->setColor(cColorf(1, 1, 1));
    m_sizePanel->setTransparencyLevel(0.8);
    m_camera->getFrontLayer()->addChild(m_sizePanel);

    m_sizePanel->addChild(m_sizeLabel);

    m_controlModeLabel = new cLabel(font);
    m_controlModeLabel->m_fontColor.setGreen();
    m_controlModeLabel->setFontScale(.5);
    m_controlModeLabel->setText("[CTRL+O] Drill Control Mode = Haptic Device / Keyboard");
    m_camera->getFrontLayer()->addChild(m_controlModeLabel);
}

void DrillManager::updateLabelPositions(){
    m_sizePanel->setLocalPos(15,60);

    m_controlModeLabel->setLocalPos(20,35);
}

void DrillManager::setOverrideControl(bool val){
    m_overrideControl = val;
    if (getOverrideControl()){
        m_controlModeLabel->m_fontColor.setRed();
        m_controlModeLabel->setText("[CTRL+O] Drill Control Mode = External afComm");
    }
    else{
        m_controlModeLabel->m_fontColor.setGreen();
        m_controlModeLabel->setText("[CTRL+O] Drill Control Mode = Haptic Device / Keyboard");
    }
}

void DrillManager::setAudioPitch(double pitch)
{
    if (m_audioSource){
        m_audioSource->setPitch(pitch);
    }
}

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
            ("gcdr", p_opt::value<double>()->default_value(15.0), "Gaze Calibration Marker Motion Duration");

    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);

    if(var_map.count("info")){
        std::cout<< cmd_opts << std::endl;
        return -1;
    }

    string file_path = __FILE__;
    g_current_filepath = file_path.substr(0, file_path.rfind("/"));

    string volume_matcap = var_map["vm"].as<string>();
    string footpedal_fd = var_map["fp"].as<string>();

    m_zeroColor = cColorb(0x00, 0x00, 0x00, 0x00);

    m_boneColor = cColorb(255, 249, 219, 255);

    m_storedColor = cColorb(0x00, 0x00, 0x00, 0x00);

    m_worldPtr = a_afWorld;

    // Get first camera
    m_mainCamera = findAndAppendCamera("main_camera");
    m_stereoCameraL = findAndAppendCamera("cameraL");
    m_stereoCameraR = findAndAppendCamera("cameraR");
    m_stereoCameraLandR = findAndAppendCamera("stereoLR");

    if (m_cameras.size() == 0){
        cerr << "ERROR! NO CAMERAS FOUND. " << endl;
        return -1;
    }

    if (!m_mainCamera){
        cerr << "INFO! FAILED TO LOAD main_camera, taking the first camera from world " << endl;
        m_mainCamera = m_worldPtr->getCameras()[0];
    }

    if (m_stereoCameraLandR){
        makeVRWindowFullscreen(m_stereoCameraLandR);
    }

    if (!m_drillManager.init(a_afWorld, m_mainCamera, var_map)){
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

    updateLabelPositions();

    // Volume Properties
    float dim[3];
    dim[0] = m_volumeObject->getDimensions().get(0);
    dim[1]= m_volumeObject->getDimensions().get(1);
    dim[2] = m_volumeObject->getDimensions().get(2);

    int voxelCount[3];
    voxelCount[0] = m_volumeObject->getVoxelCount().get(0);
    voxelCount[1]= m_volumeObject->getVoxelCount().get(1);
    voxelCount[2] = m_volumeObject->getVoxelCount().get(2);

    m_drillManager.m_drillingPub->volumeProp(dim, voxelCount);

    string volumeMatcapFilepath = g_current_filepath + "/../resources/matcap/" + volume_matcap;
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
        cerr << "SUCCESFULLY FOUND FOOTPEDAL \n";
    }

    m_gazeMarkerController.init(m_worldPtr, m_mainCamera, var_map);

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

    updateButtons();
}

void afVolmetricDrillingPlugin::physicsUpdate(double dt){

    m_gazeMarkerController.moveGazeMarker(dt);

    m_worldPtr->getChaiWorld()->computeGlobalPositions(true);

    m_drillManager.update(dt);

    if (m_drillManager.m_toolCursorList[0]->isInContact(m_voxelObj) && m_drillManager.m_targetToolCursorIdx == 0 /*&& (userSwitches == 2)*/)
    {

        for (int ci = 0 ; ci < 3 ; ci++){
            // retrieve contact event
            cCollisionEvent* contact = m_drillManager.m_toolCursorList[0]->m_hapticPoint->getCollisionEvent(ci);

            cVector3d orig(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ);

            m_voxelObj->m_texture->m_image->getVoxelColor(uint(orig.x()), uint(orig.y()), uint(orig.z()), m_storedColor);

            //if the tool comes in contact with the critical region, instantiate the warning message
            if(m_storedColor != m_boneColor && m_storedColor != m_zeroColor)
            {
                m_warningPanel->setShowEnabled(true);
            }

            if (m_drillManager.m_isOn){
                m_voxelObj->m_texture->m_image->setVoxelColor(uint(orig.x()), uint(orig.y()), uint(orig.z()), m_zeroColor);
            }

            //Publisher for voxels removed
            if(m_storedColor != m_zeroColor)
            {
                double sim_time = m_worldPtr->getCurrentTimeStamp();

                double voxel_array[3] = {orig.get(0), orig.get(1), orig.get(2)};

                cColorf color_glFloat = m_storedColor.getColorf();
                float color_array[4];
                color_array[0] = color_glFloat.getR();
                color_array[1] = color_glFloat.getG();
                color_array[2] = color_glFloat.getB();
                color_array[3] = color_glFloat.getA();


                m_drillManager.m_drillingPub->voxelsRemoved(voxel_array,color_array,sim_time);
            }

            m_mutexVoxel.acquire();
            m_volumeUpdate.enclose(cVector3d(uint(orig.x()), uint(orig.y()), uint(orig.z())));
            m_mutexVoxel.release();
            // mark voxel for update
        }

        m_flagMarkVolumeForUpdate = true;
    }
    // remove warning panel
    else
    {
        m_warningPanel->setShowEnabled(false);
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

///
/// \brief This method initializes the tool cursors.
/// \param a_afWorld    A world that contains all objects of the virtual environment
/// \return
///
void DrillManager::toolCursorInit(const afWorldPtr a_afWorld){

    for(int i=0; i < m_toolCursorList.size(); i++)
    {
        m_toolCursorList[i] = new cToolCursor(a_afWorld->getChaiWorld());

        a_afWorld->addSceneObjectToWorld(m_toolCursorList[i]);

        if(i == 0)
        {
            m_toolCursorList[i]->setHapticDevice(m_hapticDevice);

            // map the physical workspace of the haptic device to a larger virtual workspace.

            m_toolCursorList[i]->setWorkspaceRadius(5.0);
            m_toolCursorList[i]->setWaitForSmallForce(true);
            m_toolCursorList[i]->start();
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->setShowFrame(false);

            m_toolCursorList[i]->m_name = "mastoidectomy_drill";
            m_toolCursorList[i]->m_hapticPoint->setShow(m_showGoalProxySpheres, m_showGoalProxySpheres);
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->m_material->setRedCrimson();
            m_toolCursorList[i]->m_hapticPoint->m_sphereGoal->m_material->setBlueAquamarine();

            // if the haptic device has a gripper, enable it as a user switch
            m_hapticDevice->setEnableGripperUserSwitch(true);
            m_toolCursorList[i]->setRadius(m_activeDrill->m_size); // Set the correct radius for the tip which is not from the list of cursor radii
        }
        else
        {
            m_toolCursorList[i]->setShowContactPoints(m_showGoalProxySpheres, m_showGoalProxySpheres);
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->m_material->setGreenChartreuse();
            m_toolCursorList[i]->m_hapticPoint->m_sphereGoal->m_material->setOrangeCoral();
            m_toolCursorList[i]->setRadius(m_toolCursorRadius[i]);
        }
     }

    // Initialize the start pose of the tool cursors
    toolCursorsPosUpdate(m_T_d);
    toolCursorsInitialize();
}


///
/// \brief incrementDevicePos
/// \param a_vel
///
void DrillManager::incrementDevicePos(cVector3d a_vel){
    m_T_d.setLocalPos(m_T_d.getLocalPos() + a_vel);
}


///
/// \brief incrementDeviceRot
/// \param a_rot
///
void DrillManager::incrementDeviceRot(cVector3d a_rot){
    cMatrix3d R_cmd;
    R_cmd.setExtrinsicEulerRotationDeg(a_rot(0), a_rot(1), a_rot(2), C_EULER_ORDER_XYZ);
    R_cmd = m_T_d.getLocalRot() * R_cmd;
    m_T_d.setLocalRot(R_cmd);
}

///
/// \brief afVolmetricDrillingPlugin::toolCursorsInitialize
///
void DrillManager::toolCursorsInitialize(){
    for (int i = 0 ;  i < m_toolCursorList.size() ; i++){
        m_toolCursorList[i]->initialize();
    }
}

///
/// \brief This method updates the position of the shaft tool cursors
/// which eventually updates the position of the whole tool.
///
void DrillManager::toolCursorsPosUpdate(cTransform a_targetPose){
    cVector3d n_x = a_targetPose.getLocalRot().getCol0() * m_dX;
    for (int i = 0 ; i < m_toolCursorList.size() ; i++){
        cVector3d P = a_targetPose.getLocalPos() + n_x * i;
        m_toolCursorList[i]->setDeviceLocalPos(P);
        m_toolCursorList[i]->setDeviceLocalRot(a_targetPose.getLocalRot());
    }
}

void DrillManager::reset(){
    m_hapticDevice->setForce(cVector3d(0., 0., 0.));
    m_T_d = m_T_d_init;
    toolCursorsPosUpdate(m_T_d);
    toolCursorsInitialize();
    updatePoseFromCursors();
}

///
/// \brief This method checks for collision between the tool shaft and the volume.
/// The error between the proxy and goal position of each of the shaft tool cursors is constantly
/// computed. The shaft tool cursor having the maximum error is set as g_targetToolCursor. Further, the
/// position of the drill mesh is set such that it follows the proxy position of the g_targetToolCursor.
/// If there's no collision, the drill mesh follows the proxy position of the shaft tool cursor which is
/// closest to the tip tool cursor.
///
void DrillManager::checkShaftCollision(){

    m_maxError = 0;
    m_targetToolCursor = m_toolCursorList[0];
    m_targetToolCursorIdx = 0;
    for(int i=0; i<m_toolCursorList.size(); i++)
    {

        m_currError = cDistance(m_toolCursorList[i]->m_hapticPoint->getLocalPosProxy(), m_toolCursorList[i]->m_hapticPoint->getLocalPosGoal());

        if(abs(m_currError) > abs(m_maxError + 0.00001))
        {
            m_maxError = m_currError;
            m_targetToolCursor = m_toolCursorList[i];
            m_targetToolCursorIdx = i;
        }
    }
}


///
/// \brief This method updates the position of the drill mesh.
/// After obtaining g_targetToolCursor, the drill mesh adjust it's position and rotation
/// such that it follows the proxy position of the g_targetToolCursor.
///
void DrillManager::updatePoseFromCursors(){
    cMatrix3d newDrillRot;
    newDrillRot = m_toolCursorList[0]->getDeviceLocalRot();
//    cerr << newDrillRot.str(2) << endl;

    if(m_targetToolCursorIdx == 0){
        cTransform T_tip;
        T_tip.setLocalPos(m_toolCursorList[0]->m_hapticPoint->getLocalPosProxy());
        T_tip.setLocalRot(newDrillRot);
        m_activeDrill->m_rigidBody->setLocalTransform(T_tip);
    }
    else if(cDistance(m_targetToolCursor->m_hapticPoint->getLocalPosProxy(), m_targetToolCursor->m_hapticPoint->getLocalPosGoal()) <= 0.001)
    {
        // direction of positive x-axis of drill mesh
        cVector3d xDir = m_activeDrill->m_rigidBody->getLocalRot().getCol0();

        cVector3d newDrillPos;

        // drill mesh will make a sudden jump towards the followSphere
        if(!m_suddenJump)
        {
            newDrillPos = (m_targetToolCursor->m_hapticPoint->getLocalPosProxy() - xDir * m_dX * m_targetToolCursorIdx);
        }

        // drill mesh slowly moves towards the followSphere
        else
        {
            newDrillPos = m_activeDrill->m_rigidBody->getLocalPos() + ((m_targetToolCursor->m_hapticPoint->getLocalPosProxy() - xDir * m_dX * m_targetToolCursorIdx) - m_activeDrill->m_rigidBody->getLocalPos()) * 0.04;
        }

//        cVector3d L = g_targetToolCursor->m_hapticPoint->getLocalPosProxy() - g_toolCursorList[0]->getDeviceLocalPos();

//        cerr << "Colliding Cursor " << g_targetToolCursorIdx << " Error " << L.str(2) << endl;
//        if ( L.length() < 0.01){
//            newDrillRot = g_toolCursorList[0]->getDeviceLocalRot();
//        }
//        else{
//            newDrillRot = afUtils::getRotBetweenVectors<cMatrix3d>(L, cVector3d(1, 0, 0));
//        }

        cTransform trans;
        trans.setLocalPos(newDrillPos);
        trans.setLocalRot(newDrillRot);

//        g_drillRigidBody->setLocalPos(g_drillRigidBody->getLocalPos() + newDrillPos);
        m_activeDrill->m_rigidBody->setLocalTransform(trans);
        if (m_audioSource){
            m_audioSource->setSourcePos(newDrillPos);
        }
    }
}

void DrillManager::cycleDrillTypes(){
    m_activeDrillIdx = (m_activeDrillIdx + 1) % m_drills.size();
    showOnlyActive();
    m_activeDrill = m_drills[m_activeDrillIdx];
    m_toolCursorList[0]->setRadius(m_activeDrill->m_size);
    m_burrMesh->setRadius(m_activeDrill->m_size);
    cout << "Drill Type changed to " << m_activeDrill->m_name << endl;
    m_sizeLabel->setText("Drill Type: " + m_activeDrill->m_name);

    double sim_time = m_activeDrill->m_rigidBody->getCurrentTimeStamp();
    m_drillingPub->burrChange(m_activeDrill->m_size, sim_time);
}

void DrillManager::showOnlyActive(){
    for (int di = 0 ; di < m_drills.size() ; di++){
        bool show = false;
        if (di == m_activeDrillIdx){
            show = true;
        }
        m_drills[di]->m_rigidBody->getVisualObject()->setShowEnabled(show);
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
    m_warningLabel->setLocalPos(40, 40); // Relative to the panel

    // A warning pop-up that shows up while drilling at critical region
    m_warningPanel = new cPanel();
    m_warningPanel->set(m_warningLabel->getTextWidth() + m_warningLabel->getLocalPos().x() * 2.0,
                        m_warningLabel->getTextHeight() + m_warningLabel->getLocalPos().y() * 2.0,
                        10, 10, 10, 10);
    m_warningPanel->setColor(cColorf(0.6,0,0));
    m_warningPanel->setTransparencyLevel(0.6);
    m_mainCamera->getFrontLayer()->addChild(m_warningPanel);
    m_warningPanel->setShowEnabled(true);

    m_warningPanel->addChild(m_warningLabel);

    m_volumeSmoothingLabel = new cLabel(font);
    m_volumeSmoothingLabel->m_fontColor.setRed();
    m_volumeSmoothingLabel->setFontScale(.5);
    m_volumeSmoothingLabel->setText("[ALT+S] Volume Smoothing: DISABLED");
    m_mainCamera->getFrontLayer()->addChild(m_volumeSmoothingLabel);
}

void afVolmetricDrillingPlugin::updateLabelPositions()
{

    m_warningPanel->setLocalPos((m_mainCamera->m_width - m_warningPanel->getWidth()) / 2.0,
                                ((m_mainCamera->m_height - m_warningPanel->getHeight()) / 2.0),
                                0);

    m_volumeSmoothingLabel->setLocalPos(20,10);
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

        std::string text = "[ALT+S] Volume Smoothing: " + std::string(m_enableVolumeSmoothing ? "ENABLED" : "DISABLED");
        m_volumeSmoothingLabel->m_fontColor.setRed();
        if (m_enableVolumeSmoothing){
            text+= " ( LEVEL: " + to_string(m_volumeSmoothingLevel) + ")";
            m_volumeSmoothingLabel->m_fontColor.setGreen();
        }
        m_volumeSmoothingLabel->setText(text);
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
            if (m_stereoCameraL){
                m_stereoCameraL->setLocalPos(m_stereoCameraL->getLocalPos() - cVector3d(0., 0.001, 0.));
            }
            if (m_stereoCameraR){
                m_stereoCameraR->setLocalPos(m_stereoCameraR->getLocalPos() + cVector3d(0., 0.001, 0.));
            }

            if (m_stereoCameraLandR){
                double stereo_sep = m_stereoCameraLandR->getInternalCamera()->getStereoEyeSeparation();
                m_stereoCameraLandR->getInternalCamera()->setStereoEyeSeparation(stereo_sep + 0.002);
            }
        }
        else if (a_key == GLFW_KEY_KP_SUBTRACT){
            if (m_stereoCameraL){
                m_stereoCameraL->setLocalPos(m_stereoCameraL->getLocalPos() + cVector3d(0., 0.001, 0.));
            }
            if (m_stereoCameraR){
                m_stereoCameraR->setLocalPos(m_stereoCameraR->getLocalPos() - cVector3d(0., 0.001, 0.));
            }

            if (m_stereoCameraLandR){
                double stereo_sep = m_stereoCameraLandR->getInternalCamera()->getStereoEyeSeparation();
                m_stereoCameraLandR->getInternalCamera()->setStereoEyeSeparation(stereo_sep - 0.002);
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

bool FootPedal::isDrillOn()
{
    return getPedalState(0) >= 0.0 ? true : false;
}

bool FootPedal::isChangeBurrSizePressed(){
    bool val = false;
    bool burrChangeBtnCurrState = getButtonState(static_cast<int>(FootPedalButtonMap::CHANGE_BURR_SIZE));
    if (m_burrChangeBtnPrevState == false && burrChangeBtnCurrState == true){
        val = true;
    }
    m_burrChangeBtnPrevState = burrChangeBtnCurrState;
    return val;
}

bool FootPedal::isCamClutchPressed(){
    return getButtonState(static_cast<int>(FootPedalButtonMap::CAM_CLUTCH));
}

bool FootPedal::isDeviceClutchPressed(){
    return getButtonState(static_cast<int>(FootPedalButtonMap::DEVICE_CLUTCH));
}

WaveGenerator::WaveGenerator(){

}

double WaveGenerator::generate(double dt){
    m_time += dt;
    return m_amplitude * sin(m_frequency * m_time);
}

GazeMarkerController::GazeMarkerController(){
    m_gazeMarker = nullptr;
}

int GazeMarkerController::init(afWorldPtr a_worldPtr, afCameraPtr camPtr, p_opt::variables_map &var_map){
    m_gazeMarker = a_worldPtr->getRigidBody("GazeMarker");
    if (!m_gazeMarker){
        cerr << "ERROR! GAZE MARKER RIGID BODY NOT FOUND. CAN'T PERFORM GAZE CALIBRATION MOTION" << endl;
        return -1;
    }

    m_duration = var_map["gcdr"].as<double>() + m_textShowDuration;
    m_gazeMarker->scaleSceneObjects(0.5);
    m_camera = camPtr;
    m_radius = 0.;
    m_maxRadius = 0.001;
    m_radiusStep = (m_maxRadius - m_radius) / m_duration;

    m_T_c_w = m_camera->getLocalTransform();
    m_T_m_c = cTransform(cVector3d(-5., 0., 0.), cMatrix3d());

    m_T_m_w = m_T_c_w * m_T_m_c;
    m_gazeMarker->setLocalTransform(m_T_m_w);

    m_textShowDuration = 5.0;
    m_time = m_duration;

    initializeLabels();

    updateLabelPositions();

    return 1;
}

void GazeMarkerController::initializeLabels(){
    cFontPtr font = NEW_CFONTCALIBRI36();
    m_textLabel = new cLabel(font);
    m_textLabel->m_fontColor.setBlack();
    m_textStr = "PLEASE FOCUS ON THE SPIRALLING MARKER \n\n"
                "             SHOWING MARKER IN : ";
    m_textLabel->setText(m_textStr);
    m_textLabel->setLocalPos(10, 10); // Relative to Panel

    m_textPanel = new cPanel();
    m_textPanel->set(m_textLabel->getWidth() + m_textLabel->getLocalPos().x() * 2,
                         m_textLabel->getHeight() + m_textLabel->getLocalPos().y() * 2,
                         10, 10, 10, 10);
    m_textPanel->setColor(cColorf(1., 1., 0.2));
    m_textPanel->setTransparencyLevel(0.8);
    m_camera->getFrontLayer()->addChild(m_textPanel);

    m_textPanel->addChild(m_textLabel);
}

void GazeMarkerController::updateLabelPositions(){
    m_textPanel->setLocalPos((m_camera->m_width - m_textLabel->getTextWidth()) / 2.0,
                             (m_camera->m_height - m_textLabel->getTextHeight()) / 2.0);
}

void GazeMarkerController::moveGazeMarker(double dt){
    if (m_time >= m_duration || m_gazeMarker == nullptr){
        hide(true);
        return;
    }

    if (m_time == 0.){
        updateLabelPositions();
        hide(false);
    }

    m_time += dt;

    if (m_time <= m_textShowDuration){
        string time_str = to_string(int(ceil(m_textShowDuration - m_time)));
        m_textLabel->setText(m_textStr + time_str);
        return;
    }

    m_textPanel->setShowEnabled(false);

    double offset_time = m_time - m_textShowDuration;

    cVector3d dP(0., m_radius * sin(offset_time), m_radius * cos(offset_time));

    m_T_m_w.setLocalPos( m_T_m_w.getLocalPos() + m_T_c_w.getLocalRot() * dP);
    m_T_m_w.setLocalRot(m_T_c_w.getLocalRot());

    m_gazeMarker->setLocalTransform(m_T_m_w);

//    cerr << m_time << ": " << m_radius << " :: " << m_T_m_w.getLocalPos().str(2) << endl;

    m_radius += (m_radiusStep * dt);
}

void GazeMarkerController::hide(bool val){
    if (m_gazeMarker != nullptr){
        m_gazeMarker->getVisualObject()->setShowEnabled(!val);
        m_textPanel->setShowEnabled(!val);
    }
}

void GazeMarkerController::restart(){
    if (m_gazeMarker != nullptr){
        cerr << "Restarting Gaze Marker Motion" << endl;
        m_radius = 0.;
        m_time = 0.;
        m_gazeMarker->reset();
        m_T_c_w = m_camera->getLocalTransform();
        m_T_m_w = m_T_c_w * m_T_m_c;;
    }
}
