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


#include "camera_panel_manager.h"


CameraPanel::CameraPanel(cPanel *a_panel, afCameraPtr a_camera, double a_xpos, double a_ypos, PanelReferenceOrigin a_reference, PanelReferenceType a_refType){
    m_panel = a_panel;
    m_camera = a_camera;
    m_xpos = a_xpos;
    m_ypos = a_ypos;
    m_referenceOrigin = a_reference;
    m_referenceType = a_refType;

    m_camera->getFrontLayer()->addChild(m_panel);
}

void CameraPanel::setFontColor(cColorf a_color){
    ((cLabel*)m_panel)->m_fontColor.set(a_color[0], a_color[1], a_color[2], a_color[3]);
}

void CameraPanel::setText(string a_text)
{
    ((cLabel*)m_panel)->setText(a_text);
}

void CameraPanel::setPos(double a_xpos, double a_ypos, PanelReferenceOrigin a_reference, PanelReferenceType a_refType)
{
    m_xpos = a_xpos;
    m_ypos = a_ypos;
    m_referenceOrigin = a_reference;
    m_referenceType = a_refType;
}

void CameraPanel::setVisible(bool a_visible)
{
    m_panel->setShowEnabled(a_visible);
}

void CameraPanel::update(){
    double xpos, ypos;
    double xoffset, yoffset;

    if (m_referenceOrigin == PanelReferenceOrigin::CENTER){
        xoffset = -(0.5 * m_panel->getWidth());
        yoffset = -(0.5 * m_panel->getHeight());
    }
    else if (m_referenceOrigin == PanelReferenceOrigin::LOWER_LEFT){
        xoffset = 0.;
        yoffset = 0.;
    }

    if (m_referenceType == PanelReferenceType::NORMALIZED){
        xpos = m_xpos * m_camera->m_width + xoffset;
        ypos = m_ypos * m_camera->m_height + yoffset;
    }
    else if (m_referenceType == PanelReferenceType::PIXEL){
        xpos = m_xpos + yoffset;
        ypos = m_ypos + yoffset;
    }

    m_panel->setLocalPos(xpos, ypos);
}

CameraPanelManager::CameraPanelManager(){

}

void CameraPanelManager::addCamera(afCameraPtr a_camera){
    // Check if camera is already added or not
    bool duplicate = false;
    for (int idx = 0 ; idx < m_cameras.size() ; idx++){
        if (a_camera == m_cameras[idx]){
            duplicate = true;
            cerr << "WARNING! WILL NOT ADD DUPLICATE CAMERA TO PANEL MANAGER " << endl;
            break;
        }
    }

    if (!duplicate){
        a_camera->getFrontLayer()->setGhostEnabled(true);
        m_cameras.push_back(a_camera);
        addExistingPanelsToCamera(a_camera);
    }
}

void CameraPanelManager::addPanel(cPanel* a_panel, double a_xpos, double a_ypos, PanelReferenceOrigin a_reference, PanelReferenceType a_refType){
    for (int idx = 0 ; idx < m_cameras.size() ; idx++){
        cPanel* panelCopy = (idx == 0) ? a_panel : a_panel->copy();
        panelCopy->setShowPanel(a_panel->getShowPanel());
        CameraPanel* sceneObject = new CameraPanel(panelCopy, m_cameras[idx], a_xpos, a_ypos, a_reference, a_refType);
        m_cameraPanelMap[a_panel].push_back(sceneObject);
    }
}

bool CameraPanelManager::setFontColor(cPanel *a_panel, cColorf a_color){
    bool res = false;
    CameraPanels* vec = getCameraPanels(a_panel);
    if (vec){
        res = true;
        for (int i = 0 ; i < vec->size() ; i++){
            (*vec)[i]->setFontColor(a_color);
        }
    }
    return res;
}

bool CameraPanelManager::setText(cPanel* a_panel, string a_text){
    bool res = false;
    CameraPanels* vec = getCameraPanels(a_panel);
    if (vec){
        res = true;
        for (int i = 0 ; i < vec->size() ; i++){
            (*vec)[i]->setText(a_text);
        }
    }
    return res;
}

bool CameraPanelManager::setPos(cPanel* a_panel, double a_xpos, double a_ypos, PanelReferenceOrigin a_reference, PanelReferenceType a_refType){
    bool res = false;
    CameraPanels* vec = getCameraPanels(a_panel);
    if (vec){
        res = true;
        for (int i = 0 ; i < vec->size() ; i++){
            (*vec)[i]->setPos(a_xpos, a_ypos, a_reference, a_refType);
        }
    }
    return res;
}

bool CameraPanelManager::setVisible(cPanel *a_panel, bool a_visible){
    bool res = false;
    CameraPanels* vec = getCameraPanels(a_panel);
    if (vec){
        res = true;
        for (int i = 0 ; i < vec->size() ; i++){
            (*vec)[i]->setVisible(a_visible);
        }
    }
    return res;
}

void CameraPanelManager::update(){
    CameraPanelMap::const_iterator it = m_cameraPanelMap.begin();
    for (; it != m_cameraPanelMap.end() ; it++){
        for (int j = 0 ; j < it->second.size() ; j++){
            it->second[j]->update();
        }
    }
}

CameraPanels* CameraPanelManager::getCameraPanels(cPanel* a_key){
     if (m_cameraPanelMap.find(a_key) != m_cameraPanelMap.end()){
         return &m_cameraPanelMap[a_key];
     }
     else{
         return nullptr;
     }
}


void CameraPanelManager::addExistingPanelsToCamera(afCameraPtr a_camera){
    CameraPanelMap::const_iterator it = m_cameraPanelMap.begin();
    for (; it != m_cameraPanelMap.end() ; it++){
        CameraPanel* panCamPair = (it->second)[0];
        double xpos = panCamPair->m_xpos;
        double ypos = panCamPair->m_ypos;
        PanelReferenceOrigin reference = panCamPair->m_referenceOrigin;
        PanelReferenceType refType = panCamPair->m_referenceType;
        CameraPanel* sceneObject = new CameraPanel(panCamPair->m_panel, a_camera, xpos, ypos, reference, refType);
        m_cameraPanelMap[it->first].push_back(sceneObject);
    }
}
