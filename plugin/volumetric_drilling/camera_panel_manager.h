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
#ifndef CAMERA_PANEL_MANAGER
#define CAMERA_PANEL_MANAGER

#include <afFramework.h>
#include <chai3d.h>

using namespace chai3d;
using namespace ambf;
using namespace std;


enum class PanelReferenceOrigin{
    LOWER_LEFT=0, // Take the lower left corner of the panel/label as its origin
    CENTER=1 // Take the center of the panel/label as its origin
};

enum class PanelReferenceType{
    NORMALIZED=0, // Specified in window normalized coordinates, i.e. w=0.5, h=0.5  would mean center of window
    PIXEL=1 // Specified in pixel coordinates. i.e. w=50, h=50 would be 50 pixels right from left window corner and 50 pixels high from bottom.
};

class CameraPanel{
public:
    CameraPanel(cPanel* a_object, afCameraPtr a_camera, double a_xpos, double a_ypos, PanelReferenceOrigin a_reference, PanelReferenceType a_ref_type);
    void setFontColor(cColorf a_color);
    void setText(string a_text);
    void setPos(double a_xpos, double a_ypos, PanelReferenceOrigin a_reference, PanelReferenceType a_refType);
    void setVisible(bool a_visible);
    cPanel* m_panel;
    afCameraPtr m_camera;
    double m_xpos;
    double m_ypos;
    PanelReferenceOrigin m_referenceOrigin;
    PanelReferenceType m_referenceType;

    void update();
};

typedef vector<CameraPanel*> CameraPanels;
typedef map<cPanel*, CameraPanels> CameraPanelMap;

class CameraPanelManager{
public:
    CameraPanelManager();
    void addCamera(afCameraPtr a_camera);
    void addPanel(cPanel* a_panel, double a_xpos, double a_ypos, PanelReferenceOrigin a_reference, PanelReferenceType a_refType);
    bool setFontColor(cPanel* a_panel, cColorf a_color);
    bool setText(cPanel* a_panel, string a_text);
    bool setPos(cPanel* a_panel, double a_xpos, double a_ypos, PanelReferenceOrigin a_reference=PanelReferenceOrigin::CENTER, PanelReferenceType a_refType=PanelReferenceType::NORMALIZED);
    bool setVisible(cPanel* a_panel, bool a_visible);

    void update();

    CameraPanels* getCameraPanels(cPanel* a_key);

protected:
    vector<afCameraPtr> m_cameras;
    CameraPanelMap m_cameraPanelMap;

    void addExistingPanelsToCamera(afCameraPtr a_camera);
};

#endif
