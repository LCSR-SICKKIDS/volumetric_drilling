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

#include "joystick.h"
#include <string>
#include <iostream>

using namespace std;

JoyState::JoyState(){
    m_buttons.resize(3);
    m_axes.resize(1);

    for (int i = 0 ; i < m_buttons.size(); i++){
        m_buttons[i] = false;
    }

    for (int i = 0 ; i < m_axes.size(); i++){
        m_axes[i] = -1.0;
    }
}

void JoyState::print(){
    for (int i = 0 ; i < m_buttons.size(); i++){
        printf("\tButton %u %s\n", i, m_buttons[i] ? "pressed" : "released");

    }

    for (int i = 0 ; i < m_axes.size(); i++){
    printf("\tAxis at (%6d, %6f)\n", i, m_axes[i]);

    }
}

void JoyState::reset(){
    for (int i = 0 ; i < m_buttons.size(); i++){
        m_buttons[i] = false;
    }

    for (int i = 0 ; i < m_axes.size(); i++){
        m_axes[i] = -1.0;
    }
}

///
/// \brief JoyStick::JoyStick
///
JoyStick::JoyStick(){
}


///
/// \brief JoyStick::~JoyStick
///
JoyStick::~JoyStick(){
    close(m_js);
}


///
/// \brief JoyStick::init
/// \param dev_name
/// \return
///
int JoyStick::init(string dev_name){
    m_js = open(dev_name.c_str(), O_RDONLY | O_NONBLOCK);

    if (m_js == -1){
        // Since the footpedal is not loaded, set the axes to 0.
        m_state.m_axes[0] = 0.;
        perror("Could not open JoyStick");
        return -1;
    }
    else{
        return 1;
    }

}

bool JoyStick::isAvailable(){
    return m_js == -1 ? false : true;
}


///
/// \brief JoyStick::readEvent
/// \param fd
/// \param event
/// \return
///
int JoyStick::readEvent(int fd, struct js_event *event){
    ssize_t bytes;
    bytes = read(fd, event, sizeof(event));

    if (bytes == sizeof(event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}


///
/// \brief JoyStick::getButtonState
/// \param button_index
/// \return
///
bool JoyStick::getButtonState(int button_index){
    return m_state.m_buttons[button_index];
}


///
/// \brief JoyStick::getPedalState
/// \param pedal_index
/// \return
///
double JoyStick::getPedalState(int pedal_index){
    return m_state.m_axes[pedal_index];
}

void JoyStick::poll()
{
    if (readEvent(m_js, &m_event) == 0)
    {
        switch (m_event.type)
        {
        case JS_EVENT_BUTTON:
            printf("Button %u %s\n", m_event.number, m_event.value ? "pressed" : "released");
            m_state.m_buttons[m_event.number] = m_event.value;
            break;
        case JS_EVENT_AXIS:
            printf("Axis at (%6d, %6f)\n", m_event.number, double (m_event.value / 32768.0));
            m_state.m_axes[m_event.number] = double (m_event.value / 32768.0);
            break;
        default:
            /* Ignore init events. */
            break;
        }

        fflush(stdout);
    }
}
