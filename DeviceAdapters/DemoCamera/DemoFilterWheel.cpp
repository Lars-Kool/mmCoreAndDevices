////////////////////////////////////////////////////////////////////////////////
// FILE:          DemoFilterWheel.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   A demo implementation of the CStateDeviceBase class,
//                simulating a filter wheel device, and enabling testing of
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
#include "DemoFilterWheel.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor/destructor
////////////////////////////////////////////////////////////////////////////////

CDemoFilterWheel::CDemoFilterWheel()
{
   InitializeDefaultErrorMessages();
   SetErrorText(ERR_UNKNOWN_POSITION, "Requested position not available in this device");
   EnableDelay(); // signals that the delay setting will be used
   // parent ID display
   CreateHubIDProperty();
}

CDemoFilterWheel::~CDemoFilterWheel()
{
   Shutdown();
}

////////////////////////////////////////////////////////////////////////////////
// MM::Device API
////////////////////////////////////////////////////////////////////////////////

// Copy the device name to the supplied buffer
void CDemoFilterWheel::GetName(char* Name) const
{
   CDeviceUtils::CopyLimitedString(Name, g_WheelDeviceName);
}

// Tells the system whether the device is busy executing a command.
// This is done by checking the time interval since the last
// change and comparing it to the user-set delay time.
bool CDemoFilterWheel::Busy()
{
   MM::MMTime interval = GetCurrentMMTime() - changedTime_;
   MM::MMTime delay(GetDelayMs() * 1000.0);
   return (interval < delay);
}

// Initialize the device and its properties
int CDemoFilterWheel::Initialize()
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
   int ret = CreateStringProperty(MM::g_Keyword_Name, g_WheelDeviceName, true);
   if (DEVICE_OK != ret)
      return ret;

   // Description
   ret = CreateStringProperty(MM::g_Keyword_Description, "Demo filter wheel driver", true);
   if (DEVICE_OK != ret)
      return ret;

   // Set timer for the Busy signal, or we'll get a time-out the first time we check the state of the shutter, for good measure, go back 'delay' time into the past
   changedTime_ = GetCurrentMMTime();   

   // Gate Closed Position
   ret = CreateIntegerProperty(MM::g_Keyword_Closed_Position, 0, false);
   if (DEVICE_OK != ret) { return ret; }

   // create default positions and labels
   const int bufSize = 1024;
   char buf[bufSize];
   for (long i = 0; i < numPos_; i++)
   {
      snprintf(buf, bufSize, "State-%ld", i);
      SetPositionLabel(i, buf);
      snprintf(buf, bufSize, "%ld", i);
      AddAllowedValue(MM::g_Keyword_Closed_Position, buf);
   }

   // State
   CPropertyAction* pAct = new CPropertyAction (this, &CDemoFilterWheel::OnState);
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

// Shutdown is called upon unloading the device and should
// release any resources acquired during Initialize(), which
// in this case there are none.
int CDemoFilterWheel::Shutdown()
{
   if (initialized_)
   {
      initialized_ = false;
   }
   return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// MM Action handlers
///////////////////////////////////////////////////////////////////////////////

// Handles changes to the "State" property
int CDemoFilterWheel::OnState(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      // nothing to do, let the caller to use cached property
      pProp->Set(position_);
   }
   else if (eAct == MM::AfterSet)
   {
      // Set timer for the Busy signal
      changedTime_ = GetCurrentMMTime();

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
