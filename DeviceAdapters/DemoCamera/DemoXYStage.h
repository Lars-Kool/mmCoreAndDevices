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

#include "DeviceBase.h"
#include "DemoHub.h"

class CDemoXYStage : public CXYStageBase<CDemoXYStage>
{
public:
   CDemoXYStage();
   ~CDemoXYStage();

   bool Busy();
   void GetName(char* pszName) const;

   int Initialize();
   int Shutdown();
     
   // XYStage API
   /* Note that only the Set/Get PositionStep functions are implemented in the adapter
    * It is best not to override the Set/Get PositionUm functions in DeviceBase.h, since
    * those implement corrections based on whether or not X and Y directionality should be 
    * mirrored and based on a user defined origin
    */

   // This must be correct or the conversions between steps and Um will go wrong
   virtual double GetStepSize() {return stepSize_um_;}
   virtual int SetPositionSteps(long x, long y);
   virtual int GetPositionSteps(long& x, long& y);
   virtual int SetRelativePositionSteps(long x, long y);
   virtual int Home() { return DEVICE_OK; }
   virtual int Stop();

   /* This sets the 0,0 position of the adapter to the current position.  
    * If possible, the stage controller itself should also be set to 0,0
    * Note that this differs form the function SetAdapterOrigin(), which 
    * sets the coordinate system used by the adapter
    * to values different from the system used by the stage controller
    */
   virtual int SetOrigin() { return DEVICE_OK; }

   virtual int GetLimitsUm(double& xMin, double& xMax, double& yMin, double& yMax)
   {
      xMin = lowerLimit_; xMax = upperLimit_;
      yMin = lowerLimit_; yMax = upperLimit_;
      return DEVICE_OK;
   }

   virtual int GetStepLimits(long& /*xMin*/, long& /*xMax*/, long& /*yMin*/, long& /*yMax*/)
   { return DEVICE_UNSUPPORTED_COMMAND; }
   double GetStepSizeXUm() { return stepSize_um_; }
   double GetStepSizeYUm() { return stepSize_um_; }
   int Move(double /*vx*/, double /*vy*/) {return DEVICE_OK;}

   int IsXYStageSequenceable(bool& isSequenceable) const {isSequenceable = false; return DEVICE_OK;}


   // action interface
   // ----------------
   int OnPosition(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnVelocity(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   double stepSize_um_ = 0.015;
   double posX_um_ = 0.0;
   double posY_um_ = 0.0;
   double startPosX_um_ = 0.0, startPosY_um_ = 0.0;
   double targetPosX_um_ = 0.0, targetPosY_um_ = 0.0;
   MM::MMTime moveStartTime_;     // from GetCurrentMMTime()
   long moveDuration_ms_ = 100;   // duration of current move in milliseconds
   MM::TimeoutMs* timeOutTimer_ = nullptr;
   double velocity_ = 10.0;  // in mm/s ( = um/ms)
   bool initialized_ = false;
   double lowerLimit_ = 0.0;
   double upperLimit_ = 20000.0;

   void CommitCurrentIntermediatePosition_(const MM::MMTime& now);
   void ComputeIntermediatePosition(const MM::MMTime& currentTime,
      double& currentPosX,
      double& currentPosY);
};

