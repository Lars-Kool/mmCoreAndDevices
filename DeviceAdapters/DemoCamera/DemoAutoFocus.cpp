////////////////////////////////////////////////////////////////////////////////
// FILE:          DemoAutoFocus.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   A demo implementation of the CAutoFocusBase class. It
//					   simulates an autofocus device, and enables testing of
//                the rest of the system without the need to connect to the
//                actual hardware.
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

#include "DemoHub.h"
#include "DemoAutoFocus.h"

extern const char* g_AutoFocusDeviceName;

////////////////////////////////////////////////////////////////////////////////
// MM::Device API
////////////////////////////////////////////////////////////////////////////////

void DemoAutoFocus::GetName(char* name) const
{
   CDeviceUtils::CopyLimitedString(name, g_AutoFocusDeviceName);
}

int DemoAutoFocus::Initialize()
{
   DemoHub* pHub = static_cast<DemoHub*>(GetParentHub());
   if (pHub)
   {
      char hubLabel[MM::MaxStrLength];
      pHub->GetLabel(hubLabel);
      SetParentID(hubLabel); // for backward comp.
   }
   else
   {
      LogMessage(NoHubError);
   }

   if (initialized_) { return DEVICE_OK; }

   // Name
   int ret = CreateStringProperty(MM::g_Keyword_Name, g_AutoFocusDeviceName, true);
   if (DEVICE_OK != ret)
      return ret;

   // Description
   ret = CreateStringProperty(MM::g_Keyword_Description, "Demo auto-focus adapter", true);
   if (DEVICE_OK != ret)
      return ret;

   running_ = false;   

   ret = UpdateStatus();
   if (DEVICE_OK != ret) { return ret; }

   initialized_ = true;
   return DEVICE_OK;
}

int DemoAutoFocus::Shutdown()
{
   initialized_ = false;
   return DEVICE_OK;
}
