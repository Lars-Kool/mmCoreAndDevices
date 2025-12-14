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

#pragma once

#include "DemoHub.h"

class CDemoFilterWheel : public CStateDeviceBase<CDemoFilterWheel>
{
public:
   CDemoFilterWheel();
   ~CDemoFilterWheel();
  
   // MM::Device API
   void GetName(char* pszName) const;
   bool Busy();
   int Initialize();
   int Shutdown();
  
   // MM::State API
   unsigned long GetNumberOfPositions()const { return numPos_; }
   // The rest of the MM::State API is implemented in the base class

   // Action handlers
   int OnState(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   long numPos_ = 10;
   bool initialized_ = false;
   MM::MMTime changedTime_;
   long position_ = 0;
};

