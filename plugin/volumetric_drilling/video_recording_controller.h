//==============================================================================
/*
Script Name: Video Recording Controller
Author: Jonathan Wang
Date Created: 2024-12-03
Last Modified: 2024-12-03
Version: 1.0

Description:
    This script helps control the recording of the simulator 'world' view upon clicking
    the 'Record Study' button on the Drilling Simulator GUI. It stores the video in the
    same directory that contains other recorded metrics (i.e. removed voxels, gaze capture).
    Adapted from Adnan Munawar's script https://github.com/adnanmunawar/ambf_video_recording.

Usage:
    Called within the volumetric_drilling.cpp and initalized within the volumetric_drilling.h files.
*/
//==============================================================================

#define GL_SILENCE_DEPRECATION
#ifndef VIDEO_RECORDING_H
#define VIDEO_RECORDING_H
#include <afFramework.h>
#include <stdio.h>

using namespace std;
using namespace ambf;

namespace boost{
    namespace program_options{
        class variables_map;
    }
}

namespace p_opt = boost::program_options;


class VideoRecordingController: public afSimulatorPlugin{
public:
    VideoRecordingController();
    int init(afWorldPtr a_afWorld, const string& startingDir);
    int start_recording();
    virtual void keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods) override;
    virtual void mouseBtnsUpdate(GLFWwindow* a_window, int a_button, int a_action, int a_modes) override;
    virtual void mousePosUpdate(GLFWwindow* a_window, double x_pos, double y_pos) override;
    virtual void mouseScrollUpdate(GLFWwindow* a_window, double x_pos, double y_pos) override;
    void update();
    virtual void physicsUpdate(double dt) override;
    virtual void reset() override;
    bool close();


protected:
    afCameraPtr m_camera;               // Pointer to the camera
    cFrameBufferPtr m_frameBuffer;      // Frame buffer for the video
    int m_width;                        // Width of the video
    int m_height;                       // Height of the video
    FILE* m_ffmpeg;                     // File pointer for ffmpeg
    int* m_buffer;                      // Buffer for the video frames
    cImagePtr m_image;                  // Image pointer for storing video frames
    string m_video_filename;            // Video filename to save the recording
    string m_saveDirectory;             // Directory to save the video
};

#endif