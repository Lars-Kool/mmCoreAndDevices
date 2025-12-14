////////////////////////////////////////////////////////////////////////////////
// FILE:          DemoAutoFocus.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   A demo implementation of the CAutoFocusBase class. It
//					   simulates an autofocus device, and enables testing of
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

class DemoAutoFocus : public CAutoFocusBase<DemoAutoFocus>
{
public:
   DemoAutoFocus()
   {
      CreateHubIDProperty();
   }

   // MM::Device API
   void GetName(char* pszName) const;
   bool Busy() { return busy_; }
   int Initialize();
   int Shutdown();

   // MM::AutoFocus API
   virtual int SetContinuousFocusing(bool state)
   {
      running_ = state;
      return DEVICE_OK;
   }
   virtual int GetContinuousFocusing(bool& state)
   {
      state = running_;
      return DEVICE_OK;
   }
   virtual bool IsContinuousFocusLocked() { return running_; }
   virtual int FullFocus() { return DEVICE_OK; }
   virtual int IncrementalFocus() { return DEVICE_OK; }
   virtual int GetLastFocusScore(double& score)
   {
      score = 0.0;
      return DEVICE_OK;
   }
   virtual int GetCurrentFocusScore(double& score)
   {
      score = 1.0;
      return DEVICE_OK;
   }
   virtual int GetOffset(double& offset)
   {
      offset = offset_;
      return DEVICE_OK;
   }
   virtual int SetOffset(double offset)
   {
      offset_ = offset;
      return DEVICE_OK;
   }

private:
   // Data members
   bool running_ = false;
   bool busy_ = false;
   bool initialized_ = false;
   double offset_ = 0.0;
};

