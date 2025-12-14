////////////////////////////////////////////////////////////////////////////////
// FILE:          DemoPressurePump.cpp
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

#include "DemoHub.h"
#include "DemoPressurePump.h"

const char* g_PressurePumpDeviceName = "Imposed Pressure";
const char* g_PropMeasuredPressure = "Measured Pressure";

////////////////////////////////////////////////////////////////////////////////
// MM::Device API
////////////////////////////////////////////////////////////////////////////////

// Copies the device name to the provided buffer.
void DemoPressurePump::GetName(char* name) const
{
	CDeviceUtils::CopyLimitedString(name, g_PressurePumpDeviceName);
}

// Initialize the pump and its properties.
int DemoPressurePump::Initialize()
{
   CPropertyAction* pAct = new CPropertyAction(this, &DemoPressurePump::OnImposedPressure);
   int ret  = CreateFloatProperty(g_PropImposedPressure, 0.0, false, pAct);
   if (DEVICE_OK != ret) { return ret; }
   SetPropertyLimits(g_PropImposedPressure, 0.0, 100.0);

   pAct = new CPropertyAction(this, &DemoPressurePump::OnMeasuredPressure);
   ret = CreateFloatProperty(g_PropMeasuredPressure, 0.0, true, pAct);
   if (DEVICE_OK != ret) { return ret; }

   initialized_ = true;
   return DEVICE_OK;
}

// Shuts down (unloads) the device. Since this is a demo device, there are no
// resources to release.
int DemoPressurePump::Shutdown()
{
    initialized_ = false;
    return DEVICE_OK;
}

////////////////////////////////////////////////////////////////////////////////
// MM::PressurePump API
////////////////////////////////////////////////////////////////////////////////

// Stops the pump. The implementation should halt any dispensing/withdrawal,
// hence the pressure is set to zero.
int DemoPressurePump::Stop()
{
	currentPressure_ = 0.0;
    OnPropertyChanged(g_PropImposedPressure, "0.0");
    return DEVICE_OK;
}

// Calibrates the pressure controller. Since this is a demo device, there is
// nothing to calibrate. Hence, we simply rturn DEVICE_OK.
int DemoPressurePump::Calibrate()
{
    return DEVICE_OK;
}

// Sets the pressure of the pressure controller. The provided value will
// be in kPa. The implementation should convert the unit from kPa to the
// desired unit by the device.
int DemoPressurePump::SetPressureKPa(double pressureKPa) {
   // TODO: more realistic, build it up slowly;)
   currentPressure_ = pressureKPa;
   OnPropertyChanged(g_PropImposedPressure,
       CDeviceUtils::ConvertToString(currentPressure_));
   return DEVICE_OK;
};


// Gets the pressure of the pressure controller. The returned value
// has to be in kPa. The implementation, therefore, should convert the
// value provided by the pressure controller to kPa.
int DemoPressurePump::GetPressureKPa(double& pressureKPa) {
   pressureKPa = currentPressure_;
   return DEVICE_OK;
};

////////////////////////////////////////////////////////////////////////////////
// Action handlers
////////////////////////////////////////////////////////////////////////////////

// Handles changes to the imposed pressure property.
// This is the main property that the user will interact with to set the
// desired pressure.
int DemoPressurePump::OnImposedPressure(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(currentPressure_);
   }
   else if (eAct == MM::AfterSet)
   {
	   double tempPressure;
      pProp->Get(tempPressure);
		SetPressureKPa(tempPressure);
   }
   return DEVICE_OK;
}

// Handles requests for the measured pressure property.
// This is the property that reports the actual pressure in the system.
int DemoPressurePump::OnMeasuredPressure(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      // For demo purposes, we just return the imposed pressure as measured
		// pressure. An actual implementation would query the hardware here.
      pProp->Set(currentPressure_);
   }
   return DEVICE_OK;
}
