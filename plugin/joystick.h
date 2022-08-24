#ifndef FOOTPEDAL_H
#define FOOTPEDAL_H
/**
 * Author: Jason White
 *
 * Description:
 * Reads joystick/gamepad events and displays them.
 *
 * Compile:
 * gcc joystick.c -o joystick
 *
 * Run:
 * ./joystick [/dev/input/jsX]
 *
 * See also:
 * https://www.kernel.org/doc/Documentation/input/joystick-api.txt
 */
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <string>

class JoyStick{
public:
    JoyStick();
    ~JoyStick();

    int init(std::string dev_name);

    bool getButtonState(int button_index);

    double getPedalState(int pedal_index);

    void poll();

private:
    int readEvent(int fd, struct js_event *event);

    int m_js = -1;

    struct js_event m_event;

    double m_pedalValue = 0.;
};


#endif
