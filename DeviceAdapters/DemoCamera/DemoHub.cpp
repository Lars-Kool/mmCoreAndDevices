////////////////////////////////////////////////////////////////////////////////
// FILE:          DemoHub.cpp
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

#include "ModuleInterface.h"
#include "DemoHub.h"
#include "DemoAutoFocus.h"
#include "DemoCamera.h"
#include "DemoDA.h"
#include "DemoFilterWheel.h"
#include "DemoGalvo.h"
#include "DemoLightPath.h"
#include "DemoMagnifier.h"
#include "DemoObjectiveTurret.h"
#include "DemoPressurePump.h"
#include "DemoShutter.h"
#include "DemoStage.h"
#include "DemoStateDevice.h"
#include "DemoVolumetricPump.h"
#include "DemoXYStage.h"
#include "ImageFlipX.h"
#include "ImageFlipY.h"
#include "ImageMedianFilter.h"
#include "ImageTranspose.h"

///////////////////////////////////////////////////////////////////////////////
// Global variables
///////////////////////////////////////////////////////////////////////////////

// External names used used by the rest of the system
// to load particular device from the "DemoCamera.dll" library
const char* g_CameraDeviceName = "DCam";
const char* g_WheelDeviceName = "DWheel";
const char* g_StateDeviceName = "DStateDevice";
const char* g_LightPathDeviceName = "DLightPath";
const char* g_ObjectiveDeviceName = "DObjective";
const char* g_StageDeviceName = "DStage";
const char* g_XYStageDeviceName = "DXYStage";
const char* g_AutoFocusDeviceName = "DAutoFocus";
const char* g_ShutterDeviceName = "DShutter";
const char* g_DADeviceName = "D-DA";
const char* g_DA2DeviceName = "D-DA2";
const char* g_GalvoDeviceName = "DGalvo";
const char* g_MagnifierDeviceName = "DOptovar";
const char* g_PressurePumpDeviceName = "DPressurePump";
const char* g_VolumetricPumpDeviceName = "DVolumetricPump";
const char* g_HubDeviceName = "DHub";
const char* g_ImageTransposeName = "TransposeProcessor";
const char* g_ImageFlipXName = "ImageFlipX";
const char* g_ImageFlipYName = "ImageFlipY";
const char* g_ImageMedianFilterName = "ImageMedianFilter";

// Global variables
double g_IntensityFactor = 0;


///////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////

MODULE_API void InitializeModuleData()
{
   RegisterDevice(g_CameraDeviceName, MM::CameraDevice, "Demo camera");
   RegisterDevice(g_WheelDeviceName, MM::StateDevice, "Demo filter wheel");
   RegisterDevice(g_StateDeviceName, MM::StateDevice, "Demo State Device");
   RegisterDevice(g_ObjectiveDeviceName, MM::StateDevice, "Demo objective turret");
   RegisterDevice(g_StageDeviceName, MM::StageDevice, "Demo stage");
   RegisterDevice(g_XYStageDeviceName, MM::XYStageDevice, "Demo XY stage");
   RegisterDevice(g_LightPathDeviceName, MM::StateDevice, "Demo light path");
   RegisterDevice(g_AutoFocusDeviceName, MM::AutoFocusDevice, "Demo auto focus");
   RegisterDevice(g_ShutterDeviceName, MM::ShutterDevice, "Demo shutter");
   RegisterDevice(g_DADeviceName, MM::SignalIODevice, "Demo DA");
   RegisterDevice(g_DA2DeviceName, MM::SignalIODevice, "Demo DA-2");
   RegisterDevice(g_MagnifierDeviceName, MM::MagnifierDevice, "Demo Optovar");
   RegisterDevice(g_GalvoDeviceName, MM::GalvoDevice, "Demo Galvo");
   RegisterDevice(g_PressurePumpDeviceName, MM::PressurePumpDevice, "Demo Pressure Pump");
   RegisterDevice(g_VolumetricPumpDeviceName, MM::VolumetricPumpDevice, "Demo Volumetric Pump");
   RegisterDevice(g_ImageTransposeName, MM::ImageProcessorDevice, "TransposeProcessor");
   RegisterDevice(g_ImageFlipXName, MM::ImageProcessorDevice, "ImageFlipX");
   RegisterDevice(g_ImageFlipYName, MM::ImageProcessorDevice, "ImageFlipY");
   RegisterDevice(g_ImageMedianFilterName, MM::ImageProcessorDevice, "MedianFilter");
   RegisterDevice(g_HubDeviceName, MM::HubDevice, "DHub");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
   if (deviceName == 0)
      return 0;

   // decide which device class to create based on the deviceName parameter
   if (strcmp(deviceName, g_CameraDeviceName) == 0)
   {
      // create camera
      return new CDemoCamera();
   }
   else if (strcmp(deviceName, g_WheelDeviceName) == 0)
   {
      // create filter wheel
      return new CDemoFilterWheel();
   }
   else if (strcmp(deviceName, g_ObjectiveDeviceName) == 0)
   {
      // create objective turret
      return new CDemoObjectiveTurret();
   }
   else if (strcmp(deviceName, g_StateDeviceName) == 0)
   {
      // create state device
      return new CDemoStateDevice();
   }
   else if (strcmp(deviceName, g_StageDeviceName) == 0)
   {
      // create stage
      return new CDemoStage();
   }
   else if (strcmp(deviceName, g_XYStageDeviceName) == 0)
   {
      // create stage
      return new CDemoXYStage();
   }
   else if (strcmp(deviceName, g_LightPathDeviceName) == 0)
   {
      // create light path
      return new CDemoLightPath();
   }
   else if (strcmp(deviceName, g_ShutterDeviceName) == 0)
   {
      // create shutter
      return new DemoShutter();
   }
   else if (strcmp(deviceName, g_DADeviceName) == 0)
   {
      // create DA
      return new DemoDA(0);
   }
   else if (strcmp(deviceName, g_DA2DeviceName) == 0)
   {
      // create DA
      return new DemoDA(1);
   }
   else if (strcmp(deviceName, g_AutoFocusDeviceName) == 0)
   {
      // create autoFocus
      return new DemoAutoFocus();
   }
   else if (strcmp(deviceName, g_MagnifierDeviceName) == 0)
   {
      // create Optovar 
      return new DemoMagnifier();
   }
   else if (strcmp(deviceName, g_GalvoDeviceName) == 0)
   {
      // create Galvo 
      return new DemoGalvo();
   }
   else if(strcmp(deviceName, "TransposeProcessor") == 0)
   {
      return new ImageTranspose();
   }
   else if(strcmp(deviceName, "ImageFlipX") == 0)
   {
      return new ImageFlipX();
   }
   else if(strcmp(deviceName, "ImageFlipY") == 0)
   {
      return new ImageFlipY();
   }
   else if(strcmp(deviceName, "MedianFilter") == 0)
   {
      return new ImageMedianFilter();
   }
   else if (strcmp(deviceName, g_PressurePumpDeviceName) == 0)
   {
      return new DemoPressurePump();
   }
   else if (strcmp(deviceName, g_VolumetricPumpDeviceName) == 0)
   {
      return new DemoVolumetricPump();
   }
   else if (strcmp(deviceName, g_HubDeviceName) == 0)
   {
	  return new DemoHub();
   }

   // ...supplied name not recognized
   return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
   delete pDevice;
}


///////////////////////////////////////////////////////////////////////////////
// DemoHub API
///////////////////////////////////////////////////////////////////////////////

// Copies the device name to the provided buffer.
void DemoHub::GetName(char* pName) const
{
   CDeviceUtils::CopyLimitedString(pName, g_HubDeviceName);
}

// Initializes the device and its properties.
// Since it does have any properties to initialize,
// nor any resources to allocate, we simply return DEVICE_OK.
int DemoHub::Initialize()
{
  	initialized_ = true;
 
	return DEVICE_OK;
}

// Detects and instantiates all available child peripheral devices.
int DemoHub::DetectInstalledDevices()
{  
   ClearInstalledDevices();

   // make sure this method is called before we look for available devices
   InitializeModuleData();

   char hubName[MM::MaxStrLength];
   GetName(hubName); // this device name
   for (unsigned i = 0; i < GetNumberOfDevices(); i++)
   { 
      char deviceName[MM::MaxStrLength];
      bool success = GetDeviceName(i, deviceName, MM::MaxStrLength);
      if (success && (strcmp(hubName, deviceName) != 0))
      {
         MM::Device* pDev = CreateDevice(deviceName);
         AddInstalledDevice(pDev);
      }
   }
   return DEVICE_OK; 
}


