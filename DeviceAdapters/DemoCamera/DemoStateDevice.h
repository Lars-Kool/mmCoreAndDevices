////////////////////////////////////////////////////////////////////////////////
// FILE:          DemoStateDevice.h
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

#include "DeviceBase.h"
#include "DemoHub.h"

class CDemoStateDevice : public CStateDeviceBase<CDemoStateDevice>
{
public:
   CDemoStateDevice();
   ~CDemoStateDevice();
  
	// MM::Device API
   void GetName(char* pszName) const;
   bool Busy();
   int Initialize();
   int Shutdown();

   // MM::State API
   unsigned long GetNumberOfPositions()const { return numPos_; };
   int SetGateOpen(bool open);
   int GetGateOpen(bool& open);

   // MM Action handlers
   int OnState(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnNumberOfStates(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSequence(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   uint16_t numPatterns_ = 50;
   long numPos_ = 10;
   bool initialized_ = false;
   MM::MMTime changedTime_;
   bool sequenceOn_ = false;
   bool gateOpen_ = true;
   bool isClosed_ = true;
   long position_ = 0;
};

