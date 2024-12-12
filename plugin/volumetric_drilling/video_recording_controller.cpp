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

#include "video_recording_controller.h"
#include <boost/program_options.hpp>
#include <filesystem>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <ctime>

using namespace std;
namespace fs = std::filesystem;

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace p_opt = boost::program_options;

string g_current_filepath;

VideoRecordingController::VideoRecordingController()
{

}

//------------------------------------------------------------------------------
// Function to create a new subdirectory inside "Simulator_Recordings" directory
//------------------------------------------------------------------------------
string createNewDirectory(const string& baseDir, const string& startingDir) {
    // Create the overarching "Simulator_Recordings" directory if it does not exist
    string baseDirectory = startingDir + "/" + baseDir;
    if (!fs::exists(baseDirectory)) {
        if (!fs::create_directory(baseDirectory)) {
            cerr << "Failed to create base directory: " << baseDirectory << endl;
            throw runtime_error("Base directory creation failed");
        }
    }
    // Get current time for a unique directory name
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
    string newDir = baseDirectory + "/" + ss.str();
    // Create the directory inside the base directory
    if (!fs::create_directories(newDir)) {
        cerr << "Failed to create directory: " << newDir << endl;
        throw runtime_error("Directory creation failed");
    }
    return newDir;
}

int VideoRecordingController::init(const afWorldPtr a_afWorld, const string& startingDir) {
    cerr << "INFO! Initializing video recorder plugin variables...\n";

    m_worldPtr = a_afWorld;
    m_camera = m_worldPtr->getCameras()[0];

    // Copy resolution from the camera
    m_width = m_camera->m_width;
    m_height = m_camera->m_height;

    // Setup frame buffer
    m_frameBuffer = cFrameBuffer::create();
    m_frameBuffer->setup(m_camera->getInternalCamera(), m_width, m_height, true, true);

    m_image = cImage::create();

    // Set the directory to save recordings
    m_saveDirectory = createNewDirectory("Simulator_Recordings", startingDir);
    cerr << "Recording save directory: " << m_saveDirectory << endl;

    cerr << "INFO! Variables initialized successfully.\n";
    return 1;
}

int VideoRecordingController::start_recording() {
    if (!m_camera || m_saveDirectory.empty()) {
        cerr << "ERROR! Camera or save directory is not initialized. Call init first.\n";
        return -1;
    }

    // Generate video file path
    time_t now = time(0);
    string size_str = to_string(m_width) + "x" + to_string(m_height);
    m_video_filename = m_saveDirectory + "/" + m_camera->getName() + "_" + to_string(int(m_worldPtr->getSystemTime())) + ".mp4";

    // ffmpeg settings
    string cmd = "ffmpeg";
    cmd += " -r 60";
    cmd += " -f rawvideo";
    cmd += " -pix_fmt rgba";
    cmd += " -s " + size_str;
    cmd += " -i -";
    cmd += " -threads 0";
    cmd += " -preset fast";
    cmd += " -y";
    cmd += " -pix_fmt yuv420p";
    cmd += " -crf 21"; // Major parameter to tweak video compression and output size
    cmd += " -vf vflip";
    cmd += " " + m_video_filename;

    try {
        m_ffmpeg = popen(cmd.c_str(), "w");
        cerr << "INFO! Recording started: " << m_video_filename << "\n";
    } catch (exception& e) {
        cerr << "ERROR! Failed to start recording: " << e.what() << "\n";
        return -1;
    }

    return 1;
}

void VideoRecordingController::keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods)
{

}

void VideoRecordingController::mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes)
{

}

void VideoRecordingController::mousePosUpdate(GLFWwindow *a_window, double x_pos, double y_pos) {}

void VideoRecordingController::mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos)
{

}

void VideoRecordingController::update()
{
    try{
        m_frameBuffer->renderView();
        m_frameBuffer->copyImageBuffer(m_image);
        fwrite(m_image->getData(), m_image->getBytesPerPixel() * m_width * m_height, 1, m_ffmpeg);
    }
    catch (exception e){
        cerr << e.what() << endl;
    }

    static bool save_first_frame_img = true;
    if(save_first_frame_img){
        m_image->saveToFile(m_camera->getName() + ".png");
        save_first_frame_img = false;
    }
}

void VideoRecordingController::physicsUpdate(double dt)
{

}

void VideoRecordingController::reset()
{

}

bool VideoRecordingController::close()
{
    if (m_ffmpeg){
        pclose(m_ffmpeg);
        cerr << "INFO! CLOSING FFMPEG FILE \n";
    }
    return true;
}


