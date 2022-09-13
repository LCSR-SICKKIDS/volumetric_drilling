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

// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>
#include "collision_publisher.h"
#include "joystick.h"

using namespace std;
using namespace ambf;

namespace boost{
    namespace program_options{
        class variables_map;
    }
}

namespace p_opt = boost::program_options;

enum class FootPedalButtonMap{
    CAM_CLUTCH = 0,
    CHANGE_BURR_SIZE = 1,
    DEVICE_CLUTCH = 2
};

enum class AudioState{
    STOPPED = 0,
    PLAYING = 1
};

class FootPedal: public JoyStick{
public:
    bool isDrillOn();

    bool isChangeBurrSizePressed();

    bool isCamClutchPressed();

    bool isDeviceClutchPressed();

    bool m_burrChangeBtnPrevState = false;
};

class WaveGenerator{
public:
    WaveGenerator();
    double generate(double dt);
    double m_amplitude = 0.08;
    double m_frequency = 500.0;
    double m_time = 0.;
};

class GazeMarkerController{
public:
    GazeMarkerController();

    int init(afWorldPtr a_worldPtr, afCameraPtr camPtr, p_opt::variables_map& var_map);

    void initializeLabels();

    void updateLabelPositions();

    void moveGazeMarker(double dt);

    void hide(bool val);

    void restart();


private:
    double m_radius;
    double m_maxRadius;
    double m_radiusStep;
    cTransform m_T_c_w;
    cTransform m_T_m_w;
    cTransform m_T_m_c;
    double m_time;
    double m_duration;

    afRigidBodyPtr m_gazeMarker;
    afCameraPtr m_camera;

    cPanel* m_textPanel;
    cLabel* m_textLabel;
    string m_textStr;
    double m_textShowDuration;
};


struct Drill{
public:
    afRigidBodyPtr m_rigidBody;
    string m_name;
    double m_size;
};


class DrillManager{
public:

    DrillManager();

    void cleanup();

    int init(afWorldPtr a_worldPtr, afCameraPtr a_cameraPtr, p_opt::variables_map& opts);

    void update(double dt);

    void initializeLabels();

    void updateLabelPositions();

    // Initialize tool cursors
    void toolCursorInit(const afWorldPtr);

    void incrementDevicePos(cVector3d a_pos);

    void incrementDeviceRot(cVector3d a_rot);

    void toolCursorsInitialize();

    // update position of shaft tool cursors
    void toolCursorsPosUpdate(cTransform a_devicePose);

    // check for shaft collision
    void checkShaftCollision(void);

    // cycle between drill types
    void cycleDrillTypes();

    void showOnlyActive();

    bool getOverrideControl(){return m_overrideControl;}

    void setOverrideControl(bool val);

    // update position of drill mesh
    void updatePoseFromCursors(void);

    void reset();

    // a haptic device handler
    cHapticDeviceHandler* m_deviceHandler;

    // a pointer to the current haptic device
    cGenericHapticDevicePtr m_hapticDevice;

    DrillingPublisher* m_drillingPub;

    std::vector<Drill*> m_drills;

    Drill* m_activeDrill = nullptr;

    bool m_overrideControl = false;

    // rate of drill movement
    double m_drillRate = 0.020f;

    afRigidBodyPtr m_drillReferenceBody;

    // A map of drill burr names and their sizes in simulatio
    double m_units_mmToSim;

    cShapeSphere* m_burrMesh;

    bool m_show = true;

    cTransform m_T_d, m_T_d_init; // Drills target pose

    cTransform m_T_i; // Input device transform

    cVector3d m_V_i; // Input device linear velocity

    // current and maximum distance between proxy and goal spheres
    double m_currError = 0;

    double m_maxError = 0;

    bool m_camClutch = false;

    bool m_deviceClutch = false;

    // panel to display current drill size
    cPanel* m_sizePanel;

    cLabel* m_sizeLabel;

    cLabel* m_controlModeLabel;

    cAudioSource* m_audioSource = nullptr;

    cAudioBuffer* m_audioBuffer = nullptr;

    cAudioDevice* m_audioDevice = nullptr;

    AudioState m_audioState;

    void setAudioPitch(double pitch);

    bool m_isOn;

    // toggles whether the drill mesh should move slowly towards the followSphere
    // or make a sudden jump
    bool m_suddenJump = true;

    // index of current drill size
    int m_activeDrillIdx = 0;

    bool m_showGoalProxySpheres = false;

    // list of tool cursors
    vector<cToolCursor*> m_toolCursorList;

    // radius of tool cursors
    vector<double> m_toolCursorRadius{0.02, 0.013, 0.015, 0.017, 0.019, 0.021, 0.023, 0.025};

    // Local offset between shaft tool cursors
    double m_dX = 0.03;

    // for storing index of follow sphere
    int m_targetToolCursorIdx = 0;

    cToolCursor* m_targetToolCursor;

    afCameraPtr m_camera;
};


class afVolmetricDrillingPlugin: public afSimulatorPlugin{
public:
    afVolmetricDrillingPlugin();
    virtual int init(int argc, char** argv, const afWorldPtr a_afWorld) override;
    virtual void keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods) override;
    virtual void mouseBtnsUpdate(GLFWwindow* a_window, int a_button, int a_action, int a_modes) override;
    virtual void mousePosUpdate(GLFWwindow* a_window, double x_pos, double y_pos) override {}
    virtual void mouseScrollUpdate(GLFWwindow* a_window, double x_pos, double y_pos) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;
    virtual void reset() override;
    virtual bool close() override;

protected:
    void sliceVolume(int axisIdx, double delta);

    void makeVRWindowFullscreen(afCameraPtr vrCam, int monitor_number=-1);

    void updateButtons();

    void initializeLabels();

    afCameraPtr findAndAppendCamera(string cam_name);

    void updateLabelPositions();

private:

    cVoxelObject* m_voxelObj;

    int m_renderingMode = 0;

    double m_opticalDensity;

    cMutex m_mutexVoxel;

    cCollisionAABBBox m_volumeUpdate;

    cColorb m_zeroColor;

    bool m_flagStart = true;

    int m_counter = 0;

    cGenericObject* m_selectedObject = NULL;

    bool m_flagMarkVolumeForUpdate = false;

    afVolumePtr m_volumeObject;

    // camera to render the world
    afCameraPtr m_mainCamera, m_stereoCameraL, m_stereoCameraR, m_stereoCameraLandR;

    map<string, afCameraPtr> m_cameras;

    // warning pop-up panel
    cPanel* m_warningPanel;
    cLabel* m_warningLabel;

    // color property of bone
    cColorb m_boneColor;

    // get color of voxels at (x,y,z)
    cColorb m_storedColor;

    bool m_enableVolumeSmoothing = false;

    int m_volumeSmoothingLevel = 2;

    cLabel* m_volumeSmoothingLabel;

    cVector3d m_maxVolCorner, m_minVolCorner;

    cVector3d m_maxTexCoord, m_minTexCoord;

    cVector3d m_textureCoordScale; // Scale between volume corners extent and texture coordinates extent

    DrillManager m_drillManager;

    FootPedal m_footpedal;

    WaveGenerator m_waveGenerator;

    GazeMarkerController m_gazeMarkerController;
};


AF_REGISTER_SIMULATOR_PLUGIN(afVolmetricDrillingPlugin)
