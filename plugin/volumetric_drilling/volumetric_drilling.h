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
#include "footpedal.h"
#include "camera_panel_manager.h"
#include "wave_generator.h"
#include "gaze_marker_controller.h"
#include "drill_manager.h"

using namespace std;
using namespace ambf;


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
    afCameraPtr m_mainCamera, m_cameraL, m_cameraR, m_stereoCamera;

    map<string, afCameraPtr> m_cameras;

    // warning pop-up label
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

    CameraPanelManager m_panelManager;
};


AF_REGISTER_SIMULATOR_PLUGIN(afVolmetricDrillingPlugin)
