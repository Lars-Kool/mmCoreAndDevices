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
#include "DemoXYStage.h"

const char* g_XYStageDeviceName;


CDemoXYStage::CDemoXYStage()
{
   InitializeDefaultErrorMessages();

   // parent ID display
   CreateHubIDProperty();
}

CDemoXYStage::~CDemoXYStage()
{
   Shutdown();
}

void CDemoXYStage::GetName(char* Name) const
{
   CDeviceUtils::CopyLimitedString(Name, g_XYStageDeviceName);
}

int CDemoXYStage::Initialize()
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
   int ret = CreateStringProperty(MM::g_Keyword_Name, g_XYStageDeviceName, true);
   if (DEVICE_OK != ret)
      return ret;

   // Description
   ret = CreateStringProperty(MM::g_Keyword_Description, "Demo XY stage driver", true);
   if (DEVICE_OK != ret)
      return ret;

   CPropertyAction* pAct = new CPropertyAction(this, &CDemoXYStage::OnVelocity);
   ret = CreateFloatProperty("Velocity", 10.0, false, pAct, false);
   if (ret != DEVICE_OK)
      return ret;

   ret = UpdateStatus();
   if (ret != DEVICE_OK)
      return ret;

   initialized_ = true;

   return DEVICE_OK;
}

int CDemoXYStage::Shutdown()
{
   if (initialized_)
   {
      initialized_ = false;
   }
   return DEVICE_OK;
}

bool CDemoXYStage::Busy()
{
   if (timeOutTimer_ == 0)
      return false;
   if (timeOutTimer_->expired(GetCurrentMMTime()))
   {
      // delete(timeOutTimer_);
      return false;
   }
   return true;
}

int CDemoXYStage::SetPositionSteps(long x, long y)
{
   MM::MMTime currentTime = GetCurrentMMTime();
   double newTargetX = x * stepSize_um_;
   double newTargetY = y * stepSize_um_;

   // If a move is in progress, compute the intermediate position and cancel the old move.
   if (timeOutTimer_ != nullptr && !timeOutTimer_->expired(currentTime))
   {
      double currentPosX, currentPosY;
      ComputeIntermediatePosition(currentTime, currentPosX, currentPosY);
      startPosX_um_ = currentPosX;
      startPosY_um_ = currentPosY;
      delete timeOutTimer_;
      timeOutTimer_ = nullptr;
   }
   else
   {
      // No move in progress; start from the last settled position.
      startPosX_um_ = posX_um_;
      startPosY_um_ = posY_um_;
   }

   // Set the new target.
   targetPosX_um_ = newTargetX;
   targetPosY_um_ = newTargetY;

   // Calculate the distance and determine the move duration (in ms)
   double difX = targetPosX_um_ - startPosX_um_;
   double difY = targetPosY_um_ - startPosY_um_;
   double distance = sqrt((difX * difX) + (difY * difY));
   moveDuration_ms_ = (long)(distance / velocity_);
   if (moveDuration_ms_ < 1)
      moveDuration_ms_ = 1;  // enforce a minimum duration

   moveStartTime_ = currentTime;
   timeOutTimer_ = new MM::TimeoutMs(currentTime, moveDuration_ms_);

   // Optionally, notify listeners of the starting position (as an acknowledgement)
   int ret = OnXYStagePositionChanged(startPosX_um_, startPosY_um_);
   if (ret != DEVICE_OK)
      return ret;

   return DEVICE_OK;
}

int CDemoXYStage::GetPositionSteps(long& x, long& y)
{
   MM::MMTime currentTime = GetCurrentMMTime();
   if (timeOutTimer_ != nullptr && !timeOutTimer_->expired(currentTime))
   {
      double currentPosX, currentPosY;
      ComputeIntermediatePosition(currentTime, currentPosX, currentPosY);
      x = (long)(currentPosX / stepSize_um_);
      y = (long)(currentPosY / stepSize_um_);
   }
   else
   {
      // Movement complete; ensure final position is set.
      if (timeOutTimer_ != nullptr)
      {
         posX_um_ = targetPosX_um_;
         posY_um_ = targetPosY_um_;
         delete timeOutTimer_;
         timeOutTimer_ = nullptr;
      }
      x = (long)(posX_um_ / stepSize_um_);
      y = (long)(posY_um_ / stepSize_um_);
   }
   return DEVICE_OK;
}

int CDemoXYStage::SetRelativePositionSteps(long x, long y)
{
   long xSteps, ySteps;
   GetPositionSteps(xSteps, ySteps);

   return this->SetPositionSteps(xSteps+x, ySteps+y);
}

// currentTime: the current time
// currentPosX, currentPosY: output parameters for the computed position in microns
void CDemoXYStage::ComputeIntermediatePosition(
   const MM::MMTime& currentTime, double& currentPosX, double& currentPosY)
{
   double elapsed_ms = (currentTime - moveStartTime_).getMsec();
   double fraction = elapsed_ms / moveDuration_ms_;
   if (fraction > 1.0)
      fraction = 1.0;
   currentPosX = startPosX_um_ + fraction * (targetPosX_um_ - startPosX_um_);
   currentPosY = startPosY_um_ + fraction * (targetPosY_um_ - startPosY_um_);
}

void CDemoXYStage::CommitCurrentIntermediatePosition_(const MM::MMTime& now)
{
   if (timeOutTimer_ && !timeOutTimer_->expired(now))
   {
      // freeze where we *are* now
      ComputeIntermediatePosition(now, posX_um_, posY_um_);
      (void)OnXYStagePositionChanged(posX_um_, posY_um_);
   }
   // Drop the timer so Busy() instantly goes idle
   delete timeOutTimer_;
   timeOutTimer_ = nullptr;
}

int CDemoXYStage::Stop()
{
   MM::MMTime now = GetCurrentMMTime();
   CommitCurrentIntermediatePosition_(now);
   return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// Action handlers
///////////////////////////////////////////////////////////////////////////////

int CDemoXYStage::OnVelocity(MM::PropertyBase* pProp, MM::ActionType eAct){
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(velocity_);
   }
   else if (eAct == MM::AfterSet)
   {
      double newVelocity;
      pProp->Get(newVelocity);
      // Enforce a minimum positive velocity
      if (newVelocity <= 0.0)
         newVelocity = 0.1;
      velocity_ = newVelocity;
   }
   return DEVICE_OK;
}
