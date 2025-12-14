////////////////////////////////////////////////////////////////////////////////
// FILE:          DemoHub.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   A demo implementation of the HubBase class. It is responsible
//                for managing other demo devices, as well as making global variables
//                such as available to other devices.
//
// AUTHOR:        Nenad Amodaj, nenad@amodaj.com, 06/08/2005
//                
//                Karl Hoover (stuff such as programmable CCD size  & the various image processors)
//                Arther Edelstein ( equipment error simulation)
//
// COPYRIGHT:     University of California, San Francisco, 2006-2015
//                100X Imaging Inc, 2008
//
// LICENSE:       This file is distributed under the BSD license.
//                License text is included with the source distribution.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "DeviceBase.h"
#include "ImgBuffer.h"
#include "DeviceThreads.h"
#include <string>
#include <map>
#include <algorithm>
#include <stdint.h>
#include <future>

///////////////////////////////////////////////////////////////////////////////
// Global variables
///////////////////////////////////////////////////////////////////////////////

// External names used used by the rest of the system
// to load particular device from the "DemoCamera.dll" library
extern const char* g_CameraDeviceName;
extern const char* g_WheelDeviceName;
extern const char* g_StateDeviceName;
extern const char* g_LightPathDeviceName;
extern const char* g_ObjectiveDeviceName;
extern const char* g_StageDeviceName;
extern const char* g_XYStageDeviceName;
extern const char* g_AutoFocusDeviceName;
extern const char* g_ShutterDeviceName;
extern const char* g_DADeviceName;
extern const char* g_DA2DeviceName;
extern const char* g_GalvoDeviceName;
extern const char* g_MagnifierDeviceName;
extern const char* g_PressurePumpDeviceName;
extern const char* g_VolumetricPumpDeviceName;
extern const char* g_HubDeviceName;
extern const char* g_ImageTransposeName;
extern const char* g_ImageFlipXName;
extern const char* g_ImageFlipYName;
extern const char* g_ImageMedianFilterName;


// Global variables
extern double g_IntensityFactor;


//////////////////////////////////////////////////////////////////////////////
// Error codes
//////////////////////////////////////////////////////////////////////////////

#define ERR_UNKNOWN_MODE         102
#define ERR_UNKNOWN_POSITION     103
#define ERR_IN_SEQUENCE          104
#define ERR_SEQUENCE_INACTIVE    105
#define ERR_STAGE_MOVING         106
#define HUB_NOT_AVAILABLE        107

const char* NoHubError = "Parent Hub not defined.";


class DemoHub : public HubBase<DemoHub>
{
public:
   DemoHub() :
      initialized_(false),
      busy_(false)
   {}
   ~DemoHub() {}

   // MM::Device API
   void GetName(char* pName) const; 
   bool Busy() { return busy_;} ;
   int Initialize();
   int Shutdown() {return DEVICE_OK;};

   // MM::HUB API
   int DetectInstalledDevices();

private:
   // Data members
   std::vector<std::string> peripherals_;
   bool initialized_;
   bool busy_;
};
