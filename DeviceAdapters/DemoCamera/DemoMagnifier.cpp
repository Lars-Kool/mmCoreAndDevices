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

#include "DemoHub.h"
#include "DemoMagnifier.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor/Destructor
////////////////////////////////////////////////////////////////////////////////

DemoMagnifier::DemoMagnifier ()
{
   CPropertyAction* pAct = new CPropertyAction (this, &DemoMagnifier::OnHighMag);
   CreateFloatProperty("High Position Magnification", 1.6, false, pAct, true);

   pAct = new CPropertyAction (this, &DemoMagnifier::OnVariable);
   std::string propName = "Freely variable or fixed magnification";
   CreateStringProperty(propName.c_str(), "Fixed", false, pAct, true);
   AddAllowedValue(propName.c_str(), "Fixed");
   AddAllowedValue(propName.c_str(), "Variable");

   // parent ID display
   CreateHubIDProperty();
};

////////////////////////////////////////////////////////////////////////////////
// MM::Device API
////////////////////////////////////////////////////////////////////////////////

void DemoMagnifier::GetName(char* name) const
{
   CDeviceUtils::CopyLimitedString(name, g_MagnifierDeviceName);
}

int DemoMagnifier::Initialize()
{
   DemoHub* pHub = static_cast<DemoHub*>(GetParentHub());
   if (pHub)
   {
      char hubLabel[MM::MaxStrLength];
      pHub->GetLabel(hubLabel);
      SetParentID(hubLabel); // for backward comp.
   }
   else
   {
      LogMessage(NoHubError);
   }

   if (variable_)
   {
      CPropertyAction* pAct = new CPropertyAction (this, &DemoMagnifier::OnZoom);
      int ret = CreateFloatProperty("Zoom", zoomPosition_, false, pAct);
      if (DEVICE_OK != ret) { return ret; }
      SetPropertyLimits("Zoom", 0.1, highMag_);
   }
   else
   {
      CPropertyAction* pAct = new CPropertyAction (this, &DemoMagnifier::OnPosition);
      int ret = CreateStringProperty("Position", "1x", false, pAct);
      if (DEVICE_OK != ret) { return ret; }

      position_ = 0;

      AddAllowedValue("Position", "1x"); 
      AddAllowedValue("Position", highMagString().c_str()); 
   }

   int ret = UpdateStatus();
   if (DEVICE_OK != ret)
      return ret;

   return DEVICE_OK;
}

////////////////////////////////////////////////////////////////////////////////
// MM::Magnifier API
////////////////////////////////////////////////////////////////////////////////

double DemoMagnifier::GetMagnification() {
   if (variable_) { return zoomPosition_; }
   return (position_ == 0) ? 1.0 : highMag_;
}

////////////////////////////////////////////////////////////////////////////////
// MM Action Handlers
////////////////////////////////////////////////////////////////////////////////

int DemoMagnifier::OnPosition(MM::PropertyBase* pProp, MM::ActionType eAct) 
{
   if (eAct == MM::BeforeGet)
   {
      // nothing to do, let the caller use cached property
   }
   else if (eAct == MM::AfterSet)
   {
      std::string pos;
      pProp->Get(pos);
      position_ = (pos == "1x") ? 0 : 1;
      OnMagnifierChanged();
   }

   return DEVICE_OK;
}

int DemoMagnifier::OnZoom(MM::PropertyBase* pProp, MM::ActionType eAct) 
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(zoomPosition_);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(zoomPosition_);
      OnMagnifierChanged();
   }
   return DEVICE_OK;
}

int DemoMagnifier::OnHighMag(MM::PropertyBase* pProp, MM::ActionType eAct) 
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(highMag_);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(highMag_);
      ClearAllowedValues("Position");
      AddAllowedValue("Position", "1x"); 
      AddAllowedValue("Position", highMagString().c_str()); 
   }

   return DEVICE_OK;
}

int DemoMagnifier::OnVariable(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      std::string response = (variable_) ? "Variable" : "Fixed";
      pProp->Set(response.c_str());
   }
   else if (eAct == MM::AfterSet)
   {
      std::string response;
      pProp->Get(response);
      variable_ = (response == "Fixed") ? false : true;
   }
   return DEVICE_OK;
}

////////////////////////////////////////////////////////////////////////////////
// Utility functions
////////////////////////////////////////////////////////////////////////////////

std::string DemoMagnifier::highMagString() {
   std::ostringstream os;
   os << highMag_ << "x";
   return os.str();
}
