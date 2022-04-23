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

using namespace std;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

afVolmetricDrillingPlugin::afVolmetricDrillingPlugin(){
    m_drillBurrSizes[0] = make_pair<double, string>(0.02014, "2 mm");
    m_drillBurrSizes[1] = make_pair<double, string>(0.04030, "4 mm");
    m_drillBurrSizes[2] = make_pair<double, string>(0.06041, "6 mm");

    // Set the 2nd drill burr type as the active one.
    m_activeBurrIdx = 1;
}

int afVolmetricDrillingPlugin::init(int argc, char **argv, const afWorldPtr a_afWorld){

    namespace p_opt = boost::program_options;
    p_opt::options_description cmd_opts("drilling_simulator Command Line Options");
    cmd_opts.add_options()
            ("info", "Show Info")
            ("nt", p_opt::value<int>()->default_value(8), "Number Tool Cursors to Load. Default 8")
            ("ds", p_opt::value<float>()->default_value(0.026), "Offset between shaft tool cursors. Default 0.026")
            ("vm", p_opt::value<string>()->default_value("00ShinyWhite.jpg"), "Volume's Matcap Filename (Should be placed in the ./resources/matcap/ folder)")
            ("dm", p_opt::value<string>()->default_value("dark_metal_brushed.jpg"), "Drill's Matcap Filename (Should be placed in ./resources/matcap/ folder)")
            ("mute", p_opt::value<bool>()->default_value(false), "Mute");

    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);

    if(var_map.count("info")){
        std::cout<< cmd_opts << std::endl;
        return -1;
    }

    int nt = var_map["nt"].as<int>();
    float ds = var_map["ds"].as<float>();
    string volume_matcap = var_map["vm"].as<string>();
    string drill_matcap = var_map["dm"].as<string>();
    bool mute = var_map["mute"].as<bool>();

    if (nt > 0 && nt <= 8){
        m_toolCursorList.resize(nt);
    }
    else{
        cerr << "ERROR! VALID NUMBER OF TOOL CURSORS ARE BETWEEN 1 - 8. Specified value = " << nt << endl;
        return -1;
    }

    m_zeroColor = cColorb(0x00, 0x00, 0x00, 0x00);

    m_boneColor = cColorb(255, 249, 219, 255);

    m_storedColor = cColorb(0x00, 0x00, 0x00, 0x00);

    m_dX = ds;

    m_worldPtr = a_afWorld;

    // Get first camera
    m_mainCamera = m_worldPtr->getCamera("main_camera");
    if (m_mainCamera){
        cerr << "INFO! GOT CAMERA: " << m_mainCamera->getName() << endl;
    }
    else{
        cerr << "WARNING! COULD NOT FIND main_camera" << endl;
        m_mainCamera = m_worldPtr->getCameras()[0];
    }

    m_stereoCameraL = m_worldPtr->getCamera("cameraL");
    m_stereoCameraR = m_worldPtr->getCamera("cameraR");

    // Initializing tool's rotation matrix as an identity matrix
    m_toolRotMat.identity();
    m_toolRotMat = m_mainCamera->getLocalRot() * m_toolRotMat;

    // importing drill model
    m_drillRigidBody = m_worldPtr->getRigidBody("mastoidectomy_drill");
    if (!m_drillRigidBody){
        cerr << "ERROR! FAILED TO FIND DRILL RIGID BODY NAMED " << "mastoidectomy_drill" << endl;
        return -1;
    }
    else{
        m_burrMesh = new cShapeSphere(m_drillBurrSizes[m_activeBurrIdx].first);
        m_burrMesh->setRadius(m_drillBurrSizes[m_activeBurrIdx].first);
        m_burrMesh->m_material->setBlack();
        m_burrMesh->m_material->setShininess(0);
        m_burrMesh->m_material->m_specular.set(0, 0, 0);
        m_burrMesh->setShowEnabled(true);
        m_drillRigidBody->addChildSceneObject(m_burrMesh, cTransform());
        m_worldPtr->addSceneObjectToWorld(m_burrMesh);
    }

    m_volumeObject = m_worldPtr->getVolume("mastoidectomy_volume");
    if (!m_volumeObject){
        cerr << "ERROR! FAILED TO FIND DRILL VOLUME NAMED " << "mastoidectomy_volume" << endl;
        return -1;
    }
    else{
        m_voxelObj = m_volumeObject->getInternalVolume();
    }

    // create a haptic device handler
    m_deviceHandler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    m_deviceHandler->getDevice(m_hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = m_hapticDevice->getSpecifications();

    // Initializing tool cursors
    toolCursorInit(a_afWorld);

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = m_toolCursorList[0]->getWorkspaceScaleFactor();

    // stiffness properties
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // Set voxels surface contact properties
    m_voxelObj->m_material->setStiffness(2.0*maxStiffness);
    m_voxelObj->m_material->setDamping(0.0);
    m_voxelObj->m_material->setDynamicFriction(0.0);
    m_voxelObj->setUseMaterial(true);

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI40();

    // A warning pop-up that shows up while drilling at critical region
    m_warningPopup = new cPanel();
    m_warningPopup->set(m_mainCamera->m_width/2, m_mainCamera->m_height/5);
    m_warningPopup->setColor(cColorf(0.6,0,0));
    m_warningPopup->setLocalPos(m_mainCamera->m_width*0.3, m_mainCamera->m_height*0.6, 0);
    m_mainCamera->getFrontLayer()->addChild(m_warningPopup);
    m_warningPopup->setShowPanel(false);

    m_warningText = new cLabel(font);
    m_warningText->setLocalPos(0.31 * m_mainCamera->m_width, 0.67 * m_mainCamera->m_height, 0.5);
    m_warningText->m_fontColor.setWhite();
    m_warningText->setFontScale(1.0);
    m_warningText->setText("WARNING! Critical Region Detected");
    m_mainCamera->getFrontLayer()->addChild(m_warningText);
    m_warningText->setShowEnabled(false);

    // A panel to display current drill size
    m_drillSizePanel = new cPanel();
    m_drillSizePanel->setSize(170, 50);
    m_drillSizePanel->setCornerRadius(10, 10, 10, 10);
    m_drillSizePanel->setLocalPos(40,60);
    m_drillSizePanel->setColor(cColorf(1, 1, 1));
    m_drillSizePanel->setTransparencyLevel(0.8);
    m_mainCamera->getFrontLayer()->addChild(m_drillSizePanel);

    m_drillSizeText = new cLabel(font);
    m_drillSizeText->setLocalPos(20,70);
    m_drillSizeText->m_fontColor.setBlack();
    m_drillSizeText->setFontScale(.75);
    m_drillSizeText->setText("Drill Size: " + m_drillBurrSizes[m_activeBurrIdx].second);
    m_mainCamera->getFrontLayer()->addChild(m_drillSizeText);

    m_drillControlModeText = new cLabel(font);
    m_drillControlModeText->setLocalPos(20,30);
    m_drillControlModeText->m_fontColor.setGreen();
    m_drillControlModeText->setFontScale(.5);
    m_drillControlModeText->setText("Drill Control Mode = Haptic Device / Keyboard");
    m_mainCamera->getFrontLayer()->addChild(m_drillControlModeText);

    m_volumeSmoothingText = new cLabel(font);
    m_volumeSmoothingText->setLocalPos(1000,1300);
    m_volumeSmoothingText->m_fontColor.setBlue();
    m_volumeSmoothingText->setFontScale(1.);
    m_volumeSmoothingText->setText("Volume Smoothing: DISABLED");
    m_mainCamera->getFrontLayer()->addChild(m_volumeSmoothingText);

    // Get drills initial pose
    m_T_d_init = m_drillRigidBody->getLocalTransform();
    m_T_d = m_T_d_init;

    // Set up voxels_removed publisher
    m_drillingPub = new DrillingPublisher("ambf", "volumetric_drilling");

    // Volume Properties
    float dim[3];
    dim[0] = m_volumeObject->getDimensions().get(0);
    dim[1]= m_volumeObject->getDimensions().get(1);
    dim[2] = m_volumeObject->getDimensions().get(2);

    int voxelCount[3];
    voxelCount[0] = m_volumeObject->getVoxelCount().get(0);
    voxelCount[1]= m_volumeObject->getVoxelCount().get(1);
    voxelCount[2] = m_volumeObject->getVoxelCount().get(2);

    m_drillingPub->volumeProp(dim, voxelCount);

    string file_path = __FILE__;
    string cur_path = file_path.substr(0, file_path.rfind("/"));
    string volumeMatcapFilepath = cur_path + "/resources/matcap/" + volume_matcap;
    cTexture2dPtr volMatCap = cTexture2d::create();
    if(volMatCap->loadFromFile(volumeMatcapFilepath)){
        m_volumeObject->getInternalVolume()->m_aoTexture = volMatCap;
        m_volumeObject->getInternalVolume()->m_aoTexture->setTextureUnit(GL_TEXTURE5);
        cerr << "SUCCESFULLY LOADED MATCAP TEXTURE" << endl;
    }
    else{
        cerr << "FAILED TO LOAD MATCAP TEXTURE" << endl;
    }

    cTexture2dPtr drillMatCap = cTexture2d::create();
    if (m_drillRigidBody->getShaderProgram()){
        string drillMatcapFilepath = cur_path + "/resources/matcap/" + drill_matcap;
        if(drillMatCap->loadFromFile(drillMatcapFilepath)){
            for (int mi = 0 ; mi < m_drillRigidBody->getVisualObject()->getNumMeshes(); mi++){
                m_drillRigidBody->getVisualObject()->getMesh(mi)->m_metallicTexture = drillMatCap;
                m_drillRigidBody->getVisualObject()->getMesh(mi)->m_metallicTexture->setTextureUnit(GL_TEXTURE3);
            }
            m_drillRigidBody->getShaderProgram()->setUniformi("matcapMap", C_TU_METALLIC);
            cerr << "SUCCESFULLY LOADED MATCAP2 TEXTURE" << endl;
        }
        else{
            cerr << "FAILED TO LOAD MATCAP2 TEXTURE" << endl;
        }
    }

    cBackground* background = new cBackground();
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.6f, 0.6f, 0.6f),
                                cColorf(0.6f, 0.6f, 0.6f));
    m_mainCamera->getBackLayer()->addChild(background);

    m_drillAudioDevice = new cAudioDevice();
    m_mainCamera->getInternalCamera()->attachAudioDevice(m_drillAudioDevice);

    m_drillAudioBuffer = new cAudioBuffer();
    string drillAudioFilepath = cur_path + "/resources/sounds/drill.wav";
    if (m_drillAudioBuffer->loadFromFile(drillAudioFilepath)){
        m_drillAudioSource = new cAudioSource();
//        m_drillAudioBuffer->convertToMono();
        m_drillAudioSource->setAudioBuffer(m_drillAudioBuffer);
        m_drillAudioSource->setLoop(true);
        m_drillAudioSource->setGain(5.0);
        if (!mute) m_drillAudioSource->play();
    }
    else{
        delete m_drillAudioSource;
        delete m_drillAudioBuffer;
        m_drillAudioSource = nullptr;
        m_drillAudioBuffer = nullptr;
        cerr << "FAILED TO LOAD DRILL AUDIO FROM " << drillAudioFilepath << endl;
    }

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
    m_volumeObject->getShaderProgram()->setUniformi("aoMap", C_TU_AO);
}

void afVolmetricDrillingPlugin::physicsUpdate(double dt){

    m_worldPtr->getChaiWorld()->computeGlobalPositions(true);

    cTransform T_c_w = m_mainCamera->getLocalTransform();
    // If a valid haptic device is found, then it should be available
    if (getOverrideDrillControl()){
        m_T_d = m_drillRigidBody->getLocalTransform();
    }
    else if(m_hapticDevice->isDeviceAvailable()){
        bool device_clutch, cam_clutch;
        m_hapticDevice->getUserSwitch(0, device_clutch);
        m_hapticDevice->getUserSwitch(1, cam_clutch);

        m_hapticDevice->getTransform(m_T_i);
        m_hapticDevice->getLinearVelocity(m_V_i);
        m_V_i = T_c_w.getLocalRot() * (m_V_i / m_toolCursorList[0]->getWorkspaceScaleFactor());
        m_T_d.setLocalPos(m_T_d.getLocalPos() + (m_V_i * 0.4 * !device_clutch * !cam_clutch));
        m_T_d.setLocalRot(T_c_w.getLocalRot() * m_T_i.getLocalRot());

        // set zero forces when manipulating objects
        if (device_clutch || cam_clutch){
            if (cam_clutch){
                 m_mainCamera->setView(T_c_w.getLocalPos() + m_V_i * !device_clutch, m_mainCamera->getTargetPosLocal(), m_mainCamera->getUpVector());
            }
            m_toolCursorList[0]->setDeviceLocalForce(0.0, 0.0, 0.0);
        }
    }

    toolCursorsPosUpdate(m_T_d);

    // check for shaft collision
    checkShaftCollision();

    if (getOverrideDrillControl() == false){
        // updates position of drill mesh
        drillPoseUpdateFromCursors();
    }


    if (m_toolCursorList[0]->isInContact(m_voxelObj) && m_targetToolCursorIdx == 0 /*&& (userSwitches == 2)*/)
    {

        for (int ci = 0 ; ci < 3 ; ci++){
            // retrieve contact event
            cCollisionEvent* contact = m_toolCursorList[0]->m_hapticPoint->getCollisionEvent(ci);

            cVector3d orig(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ);

            m_voxelObj->m_texture->m_image->getVoxelColor(uint(orig.x()), uint(orig.y()), uint(orig.z()), m_storedColor);

            //if the tool comes in contact with the critical region, instantiate the warning message
            if(m_storedColor != m_boneColor && m_storedColor != m_zeroColor)
            {
                m_warningPopup->setShowPanel(true);
                m_warningText->setShowEnabled(true);
            }

            m_voxelObj->m_texture->m_image->setVoxelColor(uint(orig.x()), uint(orig.y()), uint(orig.z()), m_zeroColor);

            //Publisher for voxels removed
            if(m_storedColor != m_zeroColor)
            {
                double sim_time = m_drillRigidBody->getCurrentTimeStamp();

                double voxel_array[3] = {orig.get(0), orig.get(1), orig.get(2)};

                cColorf color_glFloat = m_storedColor.getColorf();
                float color_array[4];
                color_array[0] = color_glFloat.getR();
                color_array[1] = color_glFloat.getG();
                color_array[2] = color_glFloat.getB();
                color_array[3] = color_glFloat.getA();


                m_drillingPub->voxelsRemoved(voxel_array,color_array,sim_time);
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
        m_warningPopup->setShowPanel(false);
        m_warningText->setShowEnabled(false);
    }
    // compute interaction forces
    for(int i = 0 ; i < m_toolCursorList.size() ; i++){
        m_toolCursorList[i]->computeInteractionForces();
    }

    // check if device remains stuck inside voxel object
    // Also orient the force to match the camera rotation
    cVector3d force = cTranspose(T_c_w.getLocalRot()) * m_targetToolCursor->getDeviceLocalForce();
    m_toolCursorList[0]->setDeviceLocalForce(force);
    double force_mag = cClamp(force.length(), 0.0, m_hapticDevice->getSpecifications().m_maxLinearForce);
    if (m_drillAudioSource){
        m_drillAudioSource->setPitch(3.0 - force_mag / m_hapticDevice->getSpecifications().m_maxLinearForce);
    }

    if (m_flagStart)
    {
        if (force.length() != 0.0)
        {
        //    resetDrill();
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
    if (getOverrideDrillControl() == false){
        m_toolCursorList[0]->applyToDevice();
    }

}

///
/// \brief This method initializes the tool cursors.
/// \param a_afWorld    A world that contains all objects of the virtual environment
/// \return
///
void afVolmetricDrillingPlugin::toolCursorInit(const afWorldPtr a_afWorld){

    for(int i=0; i<m_toolCursorList.size(); i++)
    {
        m_toolCursorList[i] = new cToolCursor(a_afWorld->getChaiWorld());

        a_afWorld->addSceneObjectToWorld(m_toolCursorList[i]);

        if(i == 0)
        {
            m_toolCursorList[i]->setHapticDevice(m_hapticDevice);

            // map the physical workspace of the haptic device to a larger virtual workspace.

            m_toolCursorList[i]->setWorkspaceRadius(10.0);
            m_toolCursorList[i]->setWaitForSmallForce(true);
            m_toolCursorList[i]->start();
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->setShowFrame(false);

            m_toolCursorList[i]->m_name = "mastoidectomy_drill";
            m_toolCursorList[i]->m_hapticPoint->setShow(m_showGoalProxySpheres, m_showGoalProxySpheres);
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->m_material->setRedCrimson();
            m_toolCursorList[i]->m_hapticPoint->m_sphereGoal->m_material->setBlueAquamarine();

            // if the haptic device has a gripper, enable it as a user switch
            m_hapticDevice->setEnableGripperUserSwitch(true);
            m_toolCursorList[i]->setRadius(m_drillBurrSizes[m_activeBurrIdx].first); // Set the correct radius for the tip which is not from the list of cursor radii
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
void afVolmetricDrillingPlugin::incrementDevicePos(cVector3d a_vel){
    m_T_d.setLocalPos(m_T_d.getLocalPos() + a_vel);
}


///
/// \brief incrementDeviceRot
/// \param a_rot
///
void afVolmetricDrillingPlugin::incrementDeviceRot(cVector3d a_rot){
    cMatrix3d R_cmd;
    R_cmd.setExtrinsicEulerRotationDeg(a_rot(0), a_rot(1), a_rot(2), C_EULER_ORDER_XYZ);
    R_cmd = m_T_d.getLocalRot() * R_cmd;
    m_T_d.setLocalRot(R_cmd);
}

///
/// \brief afVolmetricDrillingPlugin::toolCursorsInitialize
///
void afVolmetricDrillingPlugin::toolCursorsInitialize(){
    for (int i = 0 ;  i < m_toolCursorList.size() ; i++){
        m_toolCursorList[i]->initialize();
    }
}

///
/// \brief This method updates the position of the shaft tool cursors
/// which eventually updates the position of the whole tool.
///
void afVolmetricDrillingPlugin::toolCursorsPosUpdate(cTransform a_targetPose){
    cVector3d n_x = a_targetPose.getLocalRot().getCol0() * m_dX;
    for (int i = 0 ; i < m_toolCursorList.size() ; i++){
        cVector3d P = a_targetPose.getLocalPos() + n_x * i;
        m_toolCursorList[i]->setDeviceLocalPos(P);
        m_toolCursorList[i]->setDeviceLocalRot(a_targetPose.getLocalRot());
    }
}

void afVolmetricDrillingPlugin::resetDrill(){
    m_hapticDevice->setForce(cVector3d(0., 0., 0.));
    m_T_d = m_T_d_init;
    toolCursorsPosUpdate(m_T_d);
    toolCursorsInitialize();
    drillPoseUpdateFromCursors();
}

///
/// \brief This method checks for collision between the tool shaft and the volume.
/// The error between the proxy and goal position of each of the shaft tool cursors is constantly
/// computed. The shaft tool cursor having the maximum error is set as g_targetToolCursor. Further, the
/// position of the drill mesh is set such that it follows the proxy position of the g_targetToolCursor.
/// If there's no collision, the drill mesh follows the proxy position of the shaft tool cursor which is
/// closest to the tip tool cursor.
///
void afVolmetricDrillingPlugin::checkShaftCollision(){

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
void afVolmetricDrillingPlugin::drillPoseUpdateFromCursors(){
    cMatrix3d newDrillRot;
    newDrillRot = m_toolCursorList[0]->getDeviceLocalRot();
//    cerr << newDrillRot.str(2) << endl;

    if(m_targetToolCursorIdx == 0){
        cTransform T_tip;
        T_tip.setLocalPos(m_toolCursorList[0]->m_hapticPoint->getLocalPosProxy());
        T_tip.setLocalRot(newDrillRot);
        m_drillRigidBody->setLocalTransform(T_tip);
    }
    else if(cDistance(m_targetToolCursor->m_hapticPoint->getLocalPosProxy(), m_targetToolCursor->m_hapticPoint->getLocalPosGoal()) <= 0.001)
    {
        // direction of positive x-axis of drill mesh
        cVector3d xDir = m_drillRigidBody->getLocalRot().getCol0();

        cVector3d newDrillPos;

        // drill mesh will make a sudden jump towards the followSphere
        if(!m_suddenJump)
        {
            newDrillPos = (m_targetToolCursor->m_hapticPoint->getLocalPosProxy() - xDir * m_dX * m_targetToolCursorIdx);
        }

        // drill mesh slowly moves towards the followSphere
        else
        {
            newDrillPos = m_drillRigidBody->getLocalPos() + ((m_targetToolCursor->m_hapticPoint->getLocalPosProxy() - xDir * m_dX * m_targetToolCursorIdx) - m_drillRigidBody->getLocalPos()) * 0.04;
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
        m_drillRigidBody->setLocalTransform(trans);
        m_drillAudioSource->setSourcePos(newDrillPos);
    }
}


///
/// \brief This method changes the size of the tip tool cursor.
/// Currently, the size of the tip tool cursor can be set to 2mm, 4mm, and 6mm.
///
void afVolmetricDrillingPlugin::changeBurrSize(int burrType){
    if (m_drillBurrSizes.find(burrType) != m_drillBurrSizes.end()){
        m_toolCursorList[0]->setRadius(m_drillBurrSizes[burrType].first);
        m_burrMesh->setRadius(m_drillBurrSizes[burrType].first);
        cout << "Drill Size changed to " << m_drillBurrSizes[burrType].second << endl;
        m_drillSizeText->setText("Drill Size: " + m_drillBurrSizes[burrType].second);

        double sim_time = m_drillRigidBody->getCurrentTimeStamp();
        m_drillingPub->burrChange(m_drillBurrSizes[burrType].first, sim_time);
    }
    else{
        cerr << "ERROR! DRILL BURR AT INDEX " << burrType << " DOES NOT EXIST" << endl;
    }
}

void afVolmetricDrillingPlugin::keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods) {
    if (a_mods == GLFW_MOD_CONTROL){


        // controls linear motion of tool
        if (a_key == GLFW_KEY_W) {

            cVector3d dir = m_mainCamera->getUpVector() * m_drillRate;
            incrementDevicePos(dir);
        }

        else if (a_key == GLFW_KEY_D) {

            cVector3d dir = m_mainCamera->getRightVector() * m_drillRate;
            incrementDevicePos(dir);

        }

        else if (a_key == GLFW_KEY_S) {

            cVector3d dir = m_mainCamera->getUpVector() * m_drillRate;
            incrementDevicePos(-dir);

        }

        else if (a_key == GLFW_KEY_A) {

            cVector3d dir = m_mainCamera->getRightVector() * m_drillRate;
            incrementDevicePos(-dir);

        }

        else if (a_key == GLFW_KEY_K) {

            cVector3d dir = m_mainCamera->getLookVector() * m_drillRate;
            incrementDevicePos(-dir);

        }

        else if (a_key == GLFW_KEY_I) {

            cVector3d dir = m_mainCamera->getLookVector() * m_drillRate;
            incrementDevicePos(dir);
        }

        else if (a_key == GLFW_KEY_O) {

            setOverrideDrillControl(!getOverrideDrillControl());
            if (getOverrideDrillControl()){
                m_drillControlModeText->m_fontColor.setRed();
                m_drillControlModeText->setText("Drill Control Mode = External afComm");
            }
            else{
                m_drillControlModeText->m_fontColor.setGreen();
                m_drillControlModeText->setText("Drill Control Mode = Haptic Device / Keyboard");
            }
        }

        else if (a_key == GLFW_KEY_C) {
            m_showGoalProxySpheres = !m_showGoalProxySpheres;
            for (int i = 0 ; i < m_toolCursorList.size() ; i++){
                m_toolCursorList[i]->m_hapticPoint->setShow(m_showGoalProxySpheres, m_showGoalProxySpheres);
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

        // Reset the drill pose
        if (a_key == GLFW_KEY_R){
            cerr << "INFO! RESETTING THE DRILL" << endl;
            resetDrill();
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

        std::string text = "Volume Smoothing: " + std::string(m_enableVolumeSmoothing ? "ENABLED" : "DISABLED");
        m_volumeSmoothingText->m_fontColor.setRed();
        if (m_enableVolumeSmoothing){
            text+= " ( LEVEL: " + to_string(m_volumeSmoothingLevel) + ")";
            m_volumeSmoothingText->m_fontColor.setGreen();
        }
        m_volumeSmoothingText->setText(text);
    }
    else{

        // option - reduce size along X axis
        if (a_key == GLFW_KEY_4) {
            double value = cClamp((m_voxelObj->m_maxCorner.x() - 0.005), 0.01, 0.5);
            m_voxelObj->m_maxCorner.x(value);
            m_voxelObj->m_minCorner.x(-value);
            m_voxelObj->m_maxTextureCoord.x(0.5 + value);
            m_voxelObj->m_minTextureCoord.x(0.5 - value);
            cout << "> Reduce size along X axis.                            \r";
        }

        // option - increase size along X axis
        else if (a_key == GLFW_KEY_5) {
            double value = cClamp((m_voxelObj->m_maxCorner.x() + 0.005), 0.01, 0.5);
            m_voxelObj->m_maxCorner.x(value);
            m_voxelObj->m_minCorner.x(-value);
            m_voxelObj->m_maxTextureCoord.x(0.5 + value);
            m_voxelObj->m_minTextureCoord.x(0.5 - value);
            cout << "> Increase size along X axis.                            \r";
        }

        // option - reduce size along Y axis
        else if (a_key == GLFW_KEY_6) {
            double value = cClamp((m_voxelObj->m_maxCorner.y() - 0.005), 0.01, 0.5);
            m_voxelObj->m_maxCorner.y(value);
            m_voxelObj->m_minCorner.y(-value);
            m_voxelObj->m_maxTextureCoord.y(0.5 + value);
            m_voxelObj->m_minTextureCoord.y(0.5 - value);
            cout << "> Reduce size along Y axis.                            \r";
        }

        // option - increase size along Y axis
        else if (a_key == GLFW_KEY_7) {
            double value = cClamp((m_voxelObj->m_maxCorner.y() + 0.005), 0.01, 0.5);
            m_voxelObj->m_maxCorner.y(value);
            m_voxelObj->m_minCorner.y(-value);
            m_voxelObj->m_maxTextureCoord.y(0.5 + value);
            m_voxelObj->m_minTextureCoord.y(0.5 - value);
            cout << "> Increase size along Y axis.                            \r";
        }

        // option - reduce size along Z axis
        else if (a_key == GLFW_KEY_8) {
            double value = cClamp((m_voxelObj->m_maxCorner.z() - 0.005), 0.01, 0.5);
            m_voxelObj->m_maxCorner.z(value);
            m_voxelObj->m_minCorner.z(-value);
            m_voxelObj->m_maxTextureCoord.z(0.5 + value);
            m_voxelObj->m_minTextureCoord.z(0.5 - value);
            cout << "> Reduce size along Z axis.                            \r";
        }

        // option - increase size along Z axis
        else if (a_key == GLFW_KEY_9) {
            double value = cClamp((m_voxelObj->m_maxCorner.z() + 0.005), 0.01, 0.5);
            m_voxelObj->m_maxCorner.z(value);
            m_voxelObj->m_minCorner.z(-value);
            m_voxelObj->m_maxTextureCoord.z(0.5 + value);
            m_voxelObj->m_minTextureCoord.z(0.5 - value);
            cout << "> Increase size along Z axis.                            \r";
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

            if(m_suddenJump)
            {
                m_suddenJump = false;
            }

            else
            {
                m_suddenJump = true;
            }
        }

        // toggles the visibility of drill mesh in the scene
        else if (a_key == GLFW_KEY_B){
            m_showDrill = !m_showDrill;
            m_drillRigidBody->m_visualMesh->setShowEnabled(m_showDrill);
            m_burrMesh->setShowEnabled(m_showDrill);

        }

        // toggles size of drill burr/tip tool cursor
        else if (a_key == GLFW_KEY_C){
            m_activeBurrIdx = (m_activeBurrIdx + 1) % 3;
            changeBurrSize(m_activeBurrIdx);
        }

        else if (a_key == GLFW_KEY_KP_ADD){
            if (m_stereoCameraL){
                m_stereoCameraL->setLocalPos(m_stereoCameraL->getLocalPos() - cVector3d(0., 0.001, 0.));
            }
            if (m_stereoCameraR){
                m_stereoCameraR->setLocalPos(m_stereoCameraR->getLocalPos() + cVector3d(0., 0.001, 0.));
            }
        }
        else if (a_key == GLFW_KEY_KP_SUBTRACT){
            if (m_stereoCameraL){
                m_stereoCameraL->setLocalPos(m_stereoCameraL->getLocalPos() + cVector3d(0., 0.001, 0.));
            }
            if (m_stereoCameraR){
                m_stereoCameraR->setLocalPos(m_stereoCameraR->getLocalPos() - cVector3d(0., 0.001, 0.));
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
    resetDrill();
}

bool afVolmetricDrillingPlugin::close()
{
    for(auto tool : m_toolCursorList)
    {
        tool->stop();
    }

    if (m_drillAudioSource){
        delete m_drillAudioSource;
    }
    if (m_drillAudioBuffer){
        delete m_drillAudioBuffer;
    }
    if (m_drillAudioDevice){
        m_mainCamera->getInternalCamera()->detachAudioDevice();
        delete m_drillAudioDevice;
    }
    delete m_deviceHandler;

    return true;
}
