#include "joystick.h"
#include <string>
#include <iostream>


using namespace std;


///
/// \brief JoyStick::JoyStick
///
JoyStick::JoyStick(){
}


///
/// \brief FootPedal::~FootPedal
///
JoyStick::~JoyStick(){
    close(m_js);
}


///
/// \brief FootPedal::init
/// \param dev_name
/// \return
///
int JoyStick::init(string dev_name){
    m_js = open(dev_name.c_str(), O_RDONLY | O_NONBLOCK);

    if (m_js == -1){

        perror("Could not open footpedal");
        return -1;
    }
    else{
        return 1;
    }

}


///
/// \brief FootPedal::readEvent
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
/// \brief FootPedal::getButtonState
/// \param button_index
/// \return
///
bool JoyStick::getButtonState(int button_index){
    bool pressed = false;
    if (m_js != -1){
        // Only if footpedal is initialized, check for button press. Else return false
        if (readEvent(m_js, &m_event) == 0){
            if (m_event.type == JS_EVENT_BUTTON){
                if (m_event.number == button_index ){
                    pressed = m_event.value;
                }
            }
            fflush(stdout);
        }
    }
    return pressed;
}


///
/// \brief FootPedal::getPedalState
/// \param pedal_index
/// \return
///
double JoyStick::getPedalState(int pedal_index){
    if (m_js != -1){
        if (readEvent(m_js, &m_event) == 0){
            if (m_event.type == JS_EVENT_AXIS){
                if (m_event.number == pedal_index){
                    m_pedalValue = m_event.value / double(32768);
                }
            }
            fflush(stdout);
        }
    }
    return m_pedalValue;
}

void JoyStick::poll()
{
    while (readEvent(m_js, &m_event) == 0)
        {
            switch (m_event.type)
            {
                case JS_EVENT_BUTTON:
                    printf("Button %u %s\n", m_event.number, m_event.value ? "pressed" : "released");
                    break;
                case JS_EVENT_AXIS:
                    printf("Axis at (%6d, %6f)\n", m_event.number, double (m_event.value / 32768.0));
                    break;
                default:
                    /* Ignore init events. */
                    break;
            }

            fflush(stdout);
        }
}
