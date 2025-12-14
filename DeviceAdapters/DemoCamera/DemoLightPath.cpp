////////////////////////////////////////////////////////////////////////////////
// FILE:          DemoLightPath.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   A demo implementation of the CStateDeviceBase class to
//                simulate a microscope light path switch. It enables testing of
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
#include "DemoLightPath.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor/Destructor
////////////////////////////////////////////////////////////////////////////////

CDemoLightPath::CDemoLightPath()
{
   InitializeDefaultErrorMessages();
   // parent ID display
   CreateHubIDProperty();
}

CDemoLightPath::~CDemoLightPath()
{
   Shutdown();
}

////////////////////////////////////////////////////////////////////////////////
// MM::Device API
////////////////////////////////////////////////////////////////////////////////

// Copies the device name to the provided buffer.
void CDemoLightPath::GetName(char* Name) const
{
   CDeviceUtils::CopyLimitedString(Name, g_LightPathDeviceName);
}

// Initialize the light path and its properties.
int CDemoLightPath::Initialize()
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
   int ret = CreateStringProperty(MM::g_Keyword_Name, g_LightPathDeviceName, true);
   if (DEVICE_OK != ret) { return ret; }

   // Description
   ret = CreateStringProperty(MM::g_Keyword_Description, "Demo light-path driver", true);
   if (DEVICE_OK != ret) { return ret; }

   // create default positions and labels
   const int bufSize = 1024;
   char buf[bufSize];
   for (long i = 0; i < numPos_; i++)
   {
      snprintf(buf, bufSize, "State-%ld", i);
      SetPositionLabel(i, buf);
   }

   // State
   CPropertyAction* pAct = new CPropertyAction (this, &CDemoLightPath::OnState);
   ret = CreateIntegerProperty(MM::g_Keyword_State, 0, false, pAct);
   if (DEVICE_OK != ret) { return ret; }

   // Label
   pAct = new CPropertyAction (this, &CStateBase::OnLabel);
   ret = CreateStringProperty(MM::g_Keyword_Label, "", false, pAct);
   if (DEVICE_OK != ret) { return ret; }

   ret = UpdateStatus();
   if (DEVICE_OK != ret) { return ret; }

   initialized_ = true;
   return DEVICE_OK;
}

// Shutdown the light path and release any resources.
// Since there are no resources in this demo device, we simply
// set the initialized_ flag to false.
int CDemoLightPath::Shutdown()
{
   if (initialized_)
   {
      initialized_ = false;
   }
   return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// Action handlers
///////////////////////////////////////////////////////////////////////////////

// Handles the State property.
int CDemoLightPath::OnState(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      // nothing to do, let the caller to use cached property
      pProp->Set(position_);
   }
   else if (eAct == MM::AfterSet)
   {
      long pos;
      pProp->Get(pos);
      if (pos >= numPos_ || pos < 0)
      {
         pProp->Set(position_); // revert
         return ERR_UNKNOWN_POSITION;
      }
      position_ = pos;
   }

   return DEVICE_OK;
}

