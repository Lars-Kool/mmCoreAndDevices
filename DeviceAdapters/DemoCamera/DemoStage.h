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

class CDemoStage : public CStageBase<CDemoStage>
{
public:
   CDemoStage();
   ~CDemoStage();

   bool Busy() {return busy_;}
   void GetName(char* pszName) const;

   int Initialize();
   int Shutdown();
     
   // Stage API
   int SetPositionUm(double pos);
   int GetPositionUm(double& pos) {pos = pos_um_; LogMessage("Reporting position", true); return DEVICE_OK;}
   double GetStepSize() {return stepSize_um_;}
   int SetPositionSteps(long steps) 
   {
      pos_um_ = steps * stepSize_um_; 
      return  OnStagePositionChanged(pos_um_);
   }
   int GetPositionSteps(long& steps)
   {
      steps = (long)(pos_um_ / stepSize_um_);
      return DEVICE_OK;
   }
   int SetOrigin() { return DEVICE_OK; }
   int GetLimits(double& lower, double& upper)
   {
      lower = lowerLimit_;
      upper = upperLimit_;
      return DEVICE_OK;
   }
   int Move(double /*v*/) {return DEVICE_OK;}

   bool IsContinuousFocusDrive() const {return false;}

   // action interface
   // ----------------
   int OnPosition(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSequence(MM::PropertyBase* pProp, MM::ActionType eAct);

   // Sequence functions
   int IsStageSequenceable(bool& isSequenceable) const;
   int GetStageSequenceMaxLength(long& nrEvents) const;
   int StartStageSequence();
   int StopStageSequence();
   int ClearStageSequence();
   int AddToStageSequence(double /* position */);
   int SendStageSequence();

private:
   void SetIntensityFactor(double pos);
   double stepSize_um_ = 0.025;
   double pos_um_ = 0.0;
   bool busy_ = false;
   bool initialized_ = false;
   double lowerLimit_ = -300.0;
   double upperLimit_ = 300.0;
   bool sequenceable_ = false;
};

