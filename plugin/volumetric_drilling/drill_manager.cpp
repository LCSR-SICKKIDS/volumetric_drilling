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

    \author    <amunawar@jhu.edu>
    \author    Adnan Munawar
*/
//==============================================================================

#include "drill_manager.h"
#include <boost/program_options.hpp>

DrillManager::DrillManager(){
    m_units_mmToSim = 0.001;
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
            m_mainCamera->getInternalCamera()->detachAudioDevice();
            delete m_audioDevice;
        }
        delete m_deviceHandler;
}

int DrillManager::init(afWorldPtr a_worldPtr, CameraPanelManager* a_panelManager, p_opt::variables_map& var_map){
    // importing drill model

    int nt = var_map["nt"].as<int>();
    bool mute = var_map["mute"].as<bool>();

    m_mainCamera = a_worldPtr->getCamera("main_camera");
    m_panelManager = a_panelManager;

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

    vector<int> drillDiameters = {1, 2, 4, 6};
    vector<int> voxelRemovalThresholds = {1, 3, 6, 10};

    for (int i = 0 ; i < drillDiameters.size() ; i++){
        string drillName = to_string(drillDiameters[i]) + "mm";
        afRigidBodyPtr drillRB = a_worldPtr->getRigidBody(drillName);
        if (drillRB){
            Drill* drill = new Drill();
            drill->m_name = drillName;
            drill->m_rigidBody = drillRB;
            drill->m_radius = drillDiameters[i] * m_units_mmToSim / 2.0;
            drill->setVoxelRemvalThreshold(voxelRemovalThresholds[i]);
            m_drills.push_back(drill);
        }
    }

    if (m_drills.size() == 0){
        cerr << "ERROR! FAILED TO FIND DRILLS WITH PREFIX " << "mastoidectomy_drill" << endl;
        return -1;
    }

    m_activeDrillIdx = m_drills.size() - 1;
    m_activeDrill = m_drills[m_activeDrillIdx];
    showOnlyActive();

    m_dX = var_map["ds"].as<float>();

    string drill_matcap = var_map["dm"].as<string>();

    m_burrMesh = new cShapeSphere(m_activeDrill->m_radius);
    m_burrMesh->setRadius(m_activeDrill->m_radius);
    m_burrMesh->m_material->setBlack();
    m_burrMesh->m_material->setShininess(0);
    m_burrMesh->m_material->m_specular.set(0, 0, 0);
    m_burrMesh->setShowEnabled(true);

    a_worldPtr->addSceneObjectToWorld(m_burrMesh);

    initializeLabels();

    // Get drills initial pose
    m_T_d_init = m_drillReferenceBody->getLocalTransform();

    m_T_d = m_T_d_init;

    // Set up voxels_removed publisher
    m_drillingPub = new DrillingPublisher(a_worldPtr->getNamespace(), "/plugin/volumetric_drilling");

    string file_path = __FILE__;
    string current_filepath = file_path.substr(0, file_path.rfind("/"));

    cTexture2dPtr drillMatCap = cTexture2d::create();
    string drillMatcapFilepath = current_filepath + "/../../resources/matcap/" + drill_matcap;
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
        m_mainCamera->getInternalCamera()->attachAudioDevice(m_audioDevice);

        m_audioBuffer = new cAudioBuffer();
        string drillAudioFilepath = current_filepath + "/../../resources/sounds/drill.wav";
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

    m_toolCursorRadii.push_back(0.001);
    m_toolCursorRadii.push_back(0.00065);
    m_toolCursorRadii.push_back(0.00075);
    m_toolCursorRadii.push_back(0.00085);
    m_toolCursorRadii.push_back(0.00095);
    m_toolCursorRadii.push_back(0.00105);
    m_toolCursorRadii.push_back(0.00115);
    m_toolCursorRadii.push_back(0.00125);

    // Initializing tool cursors
    toolCursorInit(a_worldPtr);

    return 1;
}

void DrillManager::update(double dt)
{
    m_toolCursorList[0]->updateFromDevice();
    cTransform T_c_w = m_mainCamera->getLocalTransform();

    // If a valid haptic device is found, then it should be available
    if (getOverrideControl()){
        m_T_d = m_drillReferenceBody->getLocalTransform();
    }
    else if(m_hapticDevice->isDeviceAvailable()){
//        m_hapticDevice->getTransform(m_T_i);
//        m_hapticDevice->getLinearVelocity(m_V_i);
        m_T_i = m_toolCursorList[0]->getDeviceLocalTransform();
        m_V_i = T_c_w.getLocalRot() * m_toolCursorList[0]->getDeviceLocalLinVel();
        m_T_d.setLocalPos(m_T_d.getLocalPos() + (m_V_i * !m_deviceClutch * !m_camClutch));
        m_T_d.setLocalRot(T_c_w.getLocalRot() * m_T_i.getLocalRot());

        // set zero forces when manipulating objects
        if (m_deviceClutch || m_camClutch){
            if (m_camClutch){
                m_mainCamera->setView(T_c_w.getLocalPos() + m_V_i * !m_deviceClutch, m_mainCamera->getTargetPosLocal(), m_mainCamera->getUpVector());
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
    m_sizeLabel->setColor(cColorf(1, 1, 1));
    m_sizeLabel->setTransparencyLevel(0.8);
    m_sizeLabel->setCornerRadius(10, 10, 10, 10);
    m_sizeLabel->setShowPanel(true);

    m_panelManager->addPanel(m_sizeLabel, 15, 60, PanelReferenceOrigin::LOWER_LEFT, PanelReferenceType::PIXEL);
//    m_panelManager->setVisible(m_sizePanel, true);

    m_controlModeLabel = new cLabel(font);
    m_controlModeLabel->m_fontColor.setGreen();
    m_controlModeLabel->setFontScale(.5);
    m_controlModeLabel->setText("[CTRL+O] Drill Control Mode = Haptic Device / Keyboard");
    m_panelManager->addPanel(m_controlModeLabel, 20, 35, PanelReferenceOrigin::LOWER_LEFT, PanelReferenceType::PIXEL);
}

void DrillManager::setOverrideControl(bool val){
    m_overrideControl = val;
    cColorf color;
    if (getOverrideControl()){
        color.setRed();
        m_panelManager->setText(m_controlModeLabel, "[CTRL+O] Drill Control Mode = External afComm");
        m_panelManager->setFontColor(m_controlModeLabel, color);
    }
    else{
        color.setGreen();
        m_panelManager->setText(m_controlModeLabel, "[CTRL+O] Drill Control Mode = Haptic Device / Keyboard");
        m_panelManager->setFontColor(m_controlModeLabel, color);
    }
}

void DrillManager::setAudioPitch(double pitch)
{
    if (m_audioSource){
        m_audioSource->setPitch(pitch);
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

            m_toolCursorList[i]->setWorkspaceRadius(m_units_mmToSim * 0.1);
            m_toolCursorList[i]->setWaitForSmallForce(true);
            m_toolCursorList[i]->start();
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->setShowFrame(false);

            m_toolCursorList[i]->m_name = "mastoidectomy_drill";
            m_toolCursorList[i]->m_hapticPoint->setShow(m_showGoalProxySpheres, m_showGoalProxySpheres);
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->m_material->setRedCrimson();
            m_toolCursorList[i]->m_hapticPoint->m_sphereGoal->m_material->setBlueAquamarine();

            // if the haptic device has a gripper, enable it as a user switch
            m_hapticDevice->setEnableGripperUserSwitch(true);
            m_toolCursorList[i]->setRadius(m_activeDrill->m_radius); // Set the correct radius for the tip which is not from the list of cursor radii
        }
        else
        {
            m_toolCursorList[i]->setShowContactPoints(m_showGoalProxySpheres, m_showGoalProxySpheres);
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->m_material->setGreenChartreuse();
            m_toolCursorList[i]->m_hapticPoint->m_sphereGoal->m_material->setOrangeCoral();
            m_toolCursorList[i]->setRadius(m_toolCursorRadii[i]);
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
    m_toolCursorList[0]->setDeviceGlobalForce(cVector3d(0., 0., 0.));
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
    m_activeDrillIdx = (m_activeDrillIdx - 1) % m_drills.size();
    m_activeDrill = m_drills[m_activeDrillIdx];
    m_toolCursorList[0]->setRadius(m_activeDrill->m_radius);
    m_burrMesh->setRadius(m_activeDrill->m_radius);
    cout << "Drill Type changed to " << m_activeDrill->m_name << endl;
    m_panelManager->setText(m_sizeLabel, "Drill Type: " + m_activeDrill->m_name);
    showOnlyActive();
    updatePoseFromCursors();

    double sim_time = m_activeDrill->m_rigidBody->getCurrentTimeStamp();
    m_drillingPub->publishDrillSize((int)(m_activeDrill->m_radius * 2.0 / m_units_mmToSim), sim_time);
}

void DrillManager::showOnlyActive(){
    for (int di = 0 ; di < m_drills.size() ; di++){
        m_drills[di]->m_rigidBody->getVisualObject()->setShowEnabled(false);
    }
    m_activeDrill->m_rigidBody->getVisualObject()->setShowEnabled(true);
}
