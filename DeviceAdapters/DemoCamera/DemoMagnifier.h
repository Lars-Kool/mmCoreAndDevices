////////////////////////////////////////////////////////////////////////////////
// FILE:          DemoMagnifier.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   A demo implementation of the CMagnifierBase class,
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

class DemoMagnifier : public CMagnifierBase<DemoMagnifier>
{
public:
   DemoMagnifier();
   ~DemoMagnifier () {};

   // MM::Device API
   void GetName(char* name) const;
   bool Busy() { return false; }
   int Initialize();
   int Shutdown() { return DEVICE_OK; }

   // MM::Magnifier API
   double GetMagnification();

   // MM Action handlers
   int OnPosition(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnZoom(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnHighMag(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnVariable(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   // Utility functions
   std::string highMagString();

   // Data members
   int position_ = 0;
   double zoomPosition_ = 1.0;
   double highMag_ = 1.6;
   bool variable_ = false;
};

