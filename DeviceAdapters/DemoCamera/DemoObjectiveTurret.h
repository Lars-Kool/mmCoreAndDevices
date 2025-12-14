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
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include "DeviceBase.h"
#include "DemoHub.h"

//////////////////////////////////////////////////////////////////////////////
// CDemoObjectiveTurret class
// Simulation of the objective changer (state device)
//////////////////////////////////////////////////////////////////////////////

class CDemoObjectiveTurret : public CStateDeviceBase<CDemoObjectiveTurret>
{
public:
   CDemoObjectiveTurret();
   ~CDemoObjectiveTurret();
  
   // MMDevice API
   // ------------
   int Initialize();
   int Shutdown();
  
   void GetName(char* pszName) const ;
   bool Busy() {return busy_;}
   unsigned long GetNumberOfPositions()const {return numPos_;}

   // action interface
   // ----------------
   int OnState(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnTrigger(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   long numPos_ = 6;
   bool busy_ = false;
   bool initialized_ = false;
   bool sequenceRunning_ = false;
   unsigned long sequenceMaxSize_ = 10;
   unsigned long sequenceIndex_ = 0;
   std::vector<std::string> sequence_;
   long position_ = 0;
};

