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

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar

    \author    <pkunjam1@jhu.edu>
    \author    Punit Kunjam
*/
//==============================================================================

#include "projection_override.h"
#include <yaml-cpp/yaml.h>

using namespace std;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

string g_current_filepath;

afCameraProjectionOverride::afCameraProjectionOverride()
{
    m_customProjectionMatrix.m_flagTransform = true;
}

int afCameraProjectionOverride::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{
    m_camera = (afCameraPtr)a_afObjectPtr;
    m_customProjectionMatrix = m_camera->getInternalCamera()->m_projectionMatrix;
    YAML::Node specificationDataNode;
    cerr << "INFO! RUNNING PROJECTION OVERRIDE PLUGIN " << endl;
//    cerr << "INFO! SPECIFICATION DATA " << a_objectAttribs->getSpecificationData().m_rawData << endl;
    specificationDataNode = YAML::Load(a_objectAttribs->getSpecificationData().m_rawData);

    YAML::Node projectionMatrixNode = specificationDataNode["projection matrix"];

    if (projectionMatrixNode.IsDefined()){
        try{
            vector<vector<double>> mat = projectionMatrixNode.as<vector<vector<double>>>();
            const int rows = 4, cols = 4;
            for (int r = 0 ; r < rows ; r++){
                cerr << "[";
                for (int c = 0 ; c < cols ; c++){
                    cerr << mat[r][c] << " ";
                    m_customProjectionMatrix(r, c) = mat[r][c];
                }
                cerr << "]" << endl;
            }

            m_camera->getInternalCamera()->m_useCustomProjectionMatrix = true;
            m_camera->getInternalCamera()->m_projectionMatrix = m_customProjectionMatrix;

            cerr << "INFO! SETTING CUSTOM PROJECT MATRIX FOR " << m_camera->getName() << endl;
        }
        catch(YAML::Exception& e){
            cerr << "ERROR! Exception " << e.what() << endl;
        }

    }
    cerr << m_camera->getInternalCamera()->m_projectionMatrix.str(4) << endl;

    return 0; // Return -1 as we don't want to do anything else in this plugin
}

void afCameraProjectionOverride::graphicsUpdate(){
    cerr << m_camera->getInternalCamera()->m_projectionMatrix.str(4) << endl;
}

void afCameraProjectionOverride::physicsUpdate(double dt){

}

void afCameraProjectionOverride::reset(){

}

bool afCameraProjectionOverride::close(){
    return true;
}
