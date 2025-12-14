////////////////////////////////////////////////////////////////////////////////
// FILE:          DemoStateDevice.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   A demo implementation of a state device. It simulates a generic
// 				   state device (a device with a finite number of discrete states),
//                and enables testing of the rest of the system without the need to
//                connect to the actual hardware.
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

#include "DemoHub.h"
#include "DemoStateDevice.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor/Destructor
////////////////////////////////////////////////////////////////////////////////

CDemoStateDevice::CDemoStateDevice()
{
   InitializeDefaultErrorMessages();
   SetErrorText(ERR_UNKNOWN_POSITION, "Requested position not available in this device");
   EnableDelay(); // signals that the dealy setting will be used

   // Number of positions
   CPropertyAction* pAct = new CPropertyAction (this, &CDemoStateDevice::OnNumberOfStates);
   CreateIntegerProperty("Number of positions", 0, false, pAct, true);

   // parent ID display
   CreateHubIDProperty();
}

CDemoStateDevice::~CDemoStateDevice()
{
   Shutdown();
}

////////////////////////////////////////////////////////////////////////////////
// MM::Device API
////////////////////////////////////////////////////////////////////////////////

void CDemoStateDevice::GetName(char* Name) const
{
   CDeviceUtils::CopyLimitedString(Name, g_StateDeviceName);
}

int CDemoStateDevice::Initialize()
{
   DemoHub* pHub = static_cast<DemoHub*>(GetParentHub());
   if (pHub)
   {
      char hubLabel[MM::MaxStrLength];
      pHub->GetLabel(hubLabel);
      SetParentID(hubLabel); // for backward comp.
   }
   else {
      LogMessage(NoHubError);
   }

   if (initialized_) {
      return DEVICE_OK;
   }

   // set property list
   // -----------------

   // Name
   int ret = CreateStringProperty(MM::g_Keyword_Name, g_StateDeviceName, true);
   if (DEVICE_OK != ret) { return ret; }

   // Description
   ret = CreateStringProperty(MM::g_Keyword_Description, "Demo state device driver", true);
   if (DEVICE_OK != ret) { return ret; }

   // Set timer for the Busy signal, or we'll get a time-out the first time we check
   // the state of the shutter, for good measure, go back 'delay' time into the past.
   changedTime_ = GetCurrentMMTime();

   // Gate Closed Position
   ret = CreateIntegerProperty(MM::g_Keyword_Closed_Position, 0, false);
   if (DEVICE_OK != ret) { return ret; }

   // create default positions and labels
   const int bufSize = 1024;
   char buf[bufSize];
   for (long i=0; i<numPos_; i++)
   {
      snprintf(buf, bufSize, "State-%ld", i);
      SetPositionLabel(i, buf);
      snprintf(buf, bufSize, "%ld", i);
      AddAllowedValue(MM::g_Keyword_Closed_Position, buf);
   }

   // State
   CPropertyAction* pAct = new CPropertyAction (this, &CDemoStateDevice::OnState);
   ret = CreateIntegerProperty(MM::g_Keyword_State, 0, false, pAct);
   if (DEVICE_OK != ret) { return ret; }

   // Label
   pAct = new CPropertyAction (this, &CStateBase::OnLabel);
   ret = CreateStringProperty(MM::g_Keyword_Label, "", false, pAct);
   if (DEVICE_OK != ret) { return ret; }

   // Sequence
   pAct = new CPropertyAction(this, &CDemoStateDevice::OnSequence);
   ret = CreateProperty("Sequence", "Off", MM::String, false, pAct);
   if (DEVICE_OK != ret) { return ret; }
   AddAllowedValue("Sequence", "On");
   AddAllowedValue("Sequence", "Off");

   ret = UpdateStatus();
   if (DEVICE_OK != ret) { return ret; }

   initialized_ = true;
   return DEVICE_OK;
}

bool CDemoStateDevice::Busy()
{
    MM::MMTime interval = GetCurrentMMTime() - changedTime_;
    MM::MMTime delay(GetDelayMs() * 1000.0);
    return (interval < delay) ? true : false;
}

int CDemoStateDevice::Shutdown()
{
   if (initialized_) { initialized_ = false; }
   return DEVICE_OK;
}

////////////////////////////////////////////////////////////////////////////////
// MM::State API
////////////////////////////////////////////////////////////////////////////////

int CDemoStateDevice::SetGateOpen(bool open)
{
    if (gateOpen_ != open)
    {
        gateOpen_ = open;
    }
    return DEVICE_OK;
}

int CDemoStateDevice::GetGateOpen(bool& open)
{
    open = gateOpen_;
    return DEVICE_OK;
}

////////////////////////////////////////////////////////////////////////////////
// MM Action handlers
////////////////////////////////////////////////////////////////////////////////

int CDemoStateDevice::OnState(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(position_);
      // nothing to do, let the caller to use cached property
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

      if (gateOpen_) {
          if ((pos == position_ && !isClosed_)) {
              return DEVICE_OK;
          }
          isClosed_ = false;
      }
      else if (!isClosed_) {
          isClosed_ = true;
      }

      position_ = pos;
      return DEVICE_OK;
   }
   else if (eAct == MM::IsSequenceable)
   {
       if (sequenceOn_)
           pProp->SetSequenceable(numPatterns_);
       else
           pProp->SetSequenceable(0);
       return DEVICE_OK;
   }
   return DEVICE_OK;
}

int CDemoStateDevice::OnNumberOfStates(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(numPos_);
   }
   else if (eAct == MM::AfterSet)
   {
      if (!initialized_)
         pProp->Get(numPos_);
   }

   return DEVICE_OK;
}

int CDemoStateDevice::OnSequence(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        if (sequenceOn_)
            pProp->Set("On");
        else
            pProp->Set("Off");
    }
    else if (eAct == MM::AfterSet)
    {
        std::string state;
        pProp->Get(state);
		  sequenceOn_ = (state == "On") ? true : false;
    }
    return DEVICE_OK;
}




