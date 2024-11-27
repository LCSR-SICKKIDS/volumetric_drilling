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
#ifndef GAZE_MARKER_CONTROLLER_H
#define GAZE_MARKER_CONTROLLER_H

#include <afFramework.h>
#include "camera_panel_manager.h"
#include "common.h"

using namespace chai3d;
using namespace ambf;

class GazeMarkerController{
public:
    GazeMarkerController();

    int init(afWorldPtr a_worldPtr, CameraPanelManager* a_panelManager, p_opt::variables_map& var_map);

    void initializeLabels();

    void computeCalibrationPattern();

    void update(double dt);

    void showMarker(bool val);

    void restart();

    CameraPanelManager* m_panelManager;


private:
    cTransform m_T_c_w;
    cTransform m_T_m_w;
    cTransform m_T_m_c;
    double m_time;

    afRigidBodyPtr m_gazeMarker;
    afCameraPtr m_mainCamera;
    afVolumePtr m_volumePtr;

    cLabel* m_gazeNotificationLabel;
    string m_textStr;
    double m_textShowDuration;

    int m_posIdx;
    double m_posDur;
    double m_posStartTime;
    double m_gridWidth;
    double m_gridHeight;
    double m_gridCenter;
    double m_depth;

    std::vector<cVector3d> m_P_m_c_list;
    
};

#endif
