///////////////////////////////////////////////////////////////////////////////
// FILE:          DemoHub.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   The example implementation of the demo camera.
//                Simulates generic digital camera and associated automated
//                microscope devices and enables testing of the rest of the
//                system without the need to connect to the actual hardware. 
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

#pragma once

#include "DemoHub.h"
#include "DemoObjectiveTurret.h"

extern const char* g_ObjectiveDeviceName;

///////////////////////////////////////////////////////////////////////////////
// CDemoObjectiveTurret implementation
///////////////////////////////////////////////////////////////////////////////

CDemoObjectiveTurret::CDemoObjectiveTurret()
{
   SetErrorText(ERR_IN_SEQUENCE, "Error occurred while executing sequence");
   SetErrorText(ERR_SEQUENCE_INACTIVE, "Sequence triggered, but sequence is not running");
   InitializeDefaultErrorMessages();
   // parent ID display
   CreateHubIDProperty();
}

CDemoObjectiveTurret::~CDemoObjectiveTurret()
{
   Shutdown();
}

void CDemoObjectiveTurret::GetName(char* Name) const
{
   CDeviceUtils::CopyLimitedString(Name, g_ObjectiveDeviceName);
}


int CDemoObjectiveTurret::Initialize()
{
   DemoHub* pHub = static_cast<DemoHub*>(GetParentHub());
   if (pHub)
   {
      char hubLabel[MM::MaxStrLength];
      pHub->GetLabel(hubLabel);
      SetParentID(hubLabel); // for backward comp.
   }
   else
      LogMessage(NoHubError);

   if (initialized_)
      return DEVICE_OK;

   // set property list
   // -----------------

   // Name
   int ret = CreateStringProperty(MM::g_Keyword_Name, g_ObjectiveDeviceName, true);
   if (DEVICE_OK != ret)
      return ret;

   // Description
   ret = CreateStringProperty(MM::g_Keyword_Description, "Demo objective turret driver", true);
   if (DEVICE_OK != ret)
      return ret;

   // create default positions and labels
   const int bufSize = 1024;
   char buf[bufSize];
   for (long i=0; i<numPos_; i++)
   {
      snprintf(buf, bufSize, "Objective-%c",'A'+ (char)i);
      SetPositionLabel(i, buf);
   }

   // State
   // -----
   CPropertyAction* pAct = new CPropertyAction (this, &CDemoObjectiveTurret::OnState);
   ret = CreateIntegerProperty(MM::g_Keyword_State, 0, false, pAct);
   if (ret != DEVICE_OK)
      return ret;

   // Label
   // -----
   pAct = new CPropertyAction (this, &CStateBase::OnLabel);
   ret = CreateStringProperty(MM::g_Keyword_Label, "", false, pAct);
   if (ret != DEVICE_OK)
      return ret;

   // Triggers to test sequence capabilities
   pAct = new CPropertyAction (this, &CDemoObjectiveTurret::OnTrigger);
   ret = CreateStringProperty("Trigger", "-", false, pAct);
   AddAllowedValue("Trigger", "-");
   AddAllowedValue("Trigger", "+");

   ret = UpdateStatus();
   if (ret != DEVICE_OK)
      return ret;

   initialized_ = true;

   return DEVICE_OK;
}

int CDemoObjectiveTurret::Shutdown()
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

int CDemoObjectiveTurret::OnState(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      // nothing to do, let the caller to use cached property
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
      std::ostringstream os;
      os << position_;
      OnPropertyChanged("State", os.str().c_str());
      char label[MM::MaxStrLength];
      GetPositionLabel(position_, label);
      OnPropertyChanged("Label", label);
   }
   else if (eAct == MM::IsSequenceable) 
   {
      pProp->SetSequenceable(sequenceMaxSize_);
   }
   else if (eAct == MM::AfterLoadSequence)
   {
      sequence_ = pProp->GetSequence();
      // DeviceBase.h checks that the vector is smaller than sequenceMaxSize_
   }
   else if (eAct == MM::StartSequence)
   {
      if (sequence_.size() > 0) {
         sequenceIndex_ = 0;
         sequenceRunning_ = true;
      }
   }
   else if (eAct  == MM::StopSequence)
   {
      sequenceRunning_ = false;
   }

   return DEVICE_OK;
}

int CDemoObjectiveTurret::OnTrigger(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set("-");
   } else if (eAct == MM::AfterSet) {
      if (!sequenceRunning_)
         return ERR_SEQUENCE_INACTIVE;
      std::string tr;
      pProp->Get(tr);
      if (tr == "+") {
         if (sequenceIndex_ < sequence_.size()) {
            std::string state = sequence_[sequenceIndex_];
            int ret = SetProperty("State", state.c_str());
            if (ret != DEVICE_OK)
               return ERR_IN_SEQUENCE;
            sequenceIndex_++;
            if (sequenceIndex_ >= sequence_.size()) {
               sequenceIndex_ = 0;
            }
         } else
         {
            return ERR_IN_SEQUENCE;
         }
      }
   }
   return DEVICE_OK;
}

