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

class DemoShutter : public CShutterBase<DemoShutter>
{
public:
   DemoShutter()
   {
      EnableDelay(); // signals that the dealy setting will be used
      
      // parent ID display
      CreateHubIDProperty();
   }
   ~DemoShutter() {}

   int Initialize();
   int Shutdown() {initialized_ = false; return DEVICE_OK;}

   void GetName (char* pszName) const;
   bool Busy();

   // Shutter API
   int SetOpen (bool open = true)
   {
      state_ = open;
      changedTime_ = GetCurrentMMTime();
      GetCoreCallback()->OnShutterOpenChanged(this, open);
      return DEVICE_OK;
   }
   int GetOpen(bool& open)
   {
      open = state_;
      return DEVICE_OK;
   }
   int Fire(double /*deltaT*/)
   { return DEVICE_UNSUPPORTED_COMMAND; }

   // action interface
   int OnState(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   bool state_ = false;
   bool initialized_ = false;
   MM::MMTime changedTime_;
};

