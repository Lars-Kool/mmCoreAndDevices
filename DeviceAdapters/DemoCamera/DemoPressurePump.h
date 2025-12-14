////////////////////////////////////////////////////////////////////////////////
// FILE:          DemoPressurePump.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   A demo implementation of the CPressurePumpBase class. It
//                simulates a pressure pump device, and enables testing of
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

#pragma once

#include "DeviceBase.h"
#include "DemoHub.h"

extern const char* g_PropImposedPressure;

class DemoPressurePump : public CPressurePumpBase<DemoPressurePump>
{
public:
   DemoPressurePump() : 
      initialized_ (false),
      currentPressure_ (0.),
      busy_(false) 
   {};

   ~DemoPressurePump() {
      if (initialized_) {
         Shutdown();
	  }
   };

   // Device API
   void GetName(char* name) const;
   bool Busy() { return busy_; }
   int Initialize();
   int Shutdown();

   // PressurePump API
   int Stop();
   int Calibrate();
   bool RequiresCalibration() { return false; };
   int SetPressureKPa(double pressureKPa);
   int GetPressureKPa(double& pressureKPa);

   // Action handlers
   int OnImposedPressure(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnMeasuredPressure(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   bool busy_;
   bool initialized_;
   double currentPressure_;
};

