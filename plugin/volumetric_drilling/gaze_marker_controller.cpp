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
#include "gaze_marker_controller.h"
#include <boost/program_options.hpp>

GazeMarkerController::GazeMarkerController(){
    m_gazeMarker = nullptr;
}

int GazeMarkerController::init(afWorldPtr a_worldPtr, CameraPanelManager* a_panelManager, p_opt::variables_map &var_map){
    m_gazeMarker = a_worldPtr->getRigidBody("GazeMarker");
    if (!m_gazeMarker){
        cerr << "ERROR! GAZE MARKER RIGID BODY NOT FOUND. CAN'T PERFORM GAZE CALIBRATION MOTION" << endl;
        return -1;
    }

    m_duration = var_map["gcdr"].as<double>() + m_textShowDuration;
    m_gazeMarker->scaleSceneObjects(0.5);
    m_mainCamera = a_worldPtr->getCamera("main_camera");
    m_panelManager = a_panelManager;
    m_radius = 0.;
    m_maxRadius = 0.0004;
    m_radiusStep = (m_maxRadius - m_radius) / m_duration;

    m_T_c_w = m_mainCamera->getLocalTransform();
    m_T_m_c = cTransform(cVector3d(-5., 0., 0.), cMatrix3d());

    m_T_m_w = m_T_c_w * m_T_m_c;
    m_gazeMarker->setLocalTransform(m_T_m_w);

    m_textShowDuration = 5.0;
    m_time = m_duration;

    initializeLabels();

    return 1;
}

void GazeMarkerController::initializeLabels(){
    cFontPtr font = NEW_CFONTCALIBRI36();
    m_gazeNotificationLabel = new cLabel(font);
    m_gazeNotificationLabel->m_fontColor.setBlack();
    m_textStr = "PLEASE FOCUS ON THE SPIRALLING MARKER \n\n"
                "             SHOWING MARKER IN : ";
    m_gazeNotificationLabel->setText(m_textStr);

    m_gazeNotificationLabel->setCornerRadius(10, 10, 10, 10);
    m_gazeNotificationLabel->setColor(cColorf(1., 1., 0.2));
    m_gazeNotificationLabel->setShowPanel(true);
    m_gazeNotificationLabel->setTransparencyLevel(0.8);
    m_gazeNotificationLabel->setShowEnabled(false);

    m_panelManager->addPanel(m_gazeNotificationLabel, 0.5, 0.5, PanelReferenceOrigin::CENTER, PanelReferenceType::NORMALIZED);
}

void GazeMarkerController::update(double dt){
    if (m_time >= m_duration || m_gazeMarker == nullptr){
        hide(true);
        return;
    }

    if (m_time == 0.){
        hide(false);
        cMatrix3d rot;
        rot.identity();
        cTransform trans(cVector3d(-100, 0, 0), rot);
        m_gazeMarker->setLocalTransform(trans); // Set the marker to be way behind the vol
    }

    m_time += dt;

    if (m_time <= m_textShowDuration){
        string time_str = to_string(int(ceil(m_textShowDuration - m_time)));
        m_panelManager->setText(m_gazeNotificationLabel, m_textStr + time_str);
        return;
    }

    m_panelManager->setVisible(m_gazeNotificationLabel, false);

    double offset_time = m_time - m_textShowDuration;

    cVector3d dP(0., m_radius * sin(offset_time), m_radius * cos(offset_time));

    m_T_m_w.setLocalPos(m_T_m_w.getLocalPos() + m_T_c_w.getLocalRot() * dP);
    m_T_m_w.setLocalRot(m_T_c_w.getLocalRot());

    m_gazeMarker->setLocalTransform(m_T_m_w);

//    cerr << m_time << ": " << m_radius << " :: " << m_T_m_w.getLocalPos().str(2) << endl;

    m_radius += (m_radiusStep * dt);
}

void GazeMarkerController::hide(bool val){
    if (m_gazeMarker){
        m_gazeMarker->getVisualObject()->setShowEnabled(!val);
        m_panelManager->setVisible(m_gazeNotificationLabel, !val);
    }
}

void GazeMarkerController::restart(){
    if (m_gazeMarker){
        cerr << "Restarting Gaze Marker Motion" << endl;
        m_radius = 0.;
        m_time = 0.;
        m_gazeMarker->reset();
        m_T_c_w = m_mainCamera->getLocalTransform();
        m_T_m_w = m_T_c_w * m_T_m_c;;
    }
}
