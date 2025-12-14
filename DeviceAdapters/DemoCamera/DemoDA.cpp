////////////////////////////////////////////////////////////////////////////////
// FILE:          DemoDA.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   A demo implementation of the CSignalIOBase class. It
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
#include "DemoDA.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor/Destructor
////////////////////////////////////////////////////////////////////////////////

DemoDA::DemoDA (uint8_t n) : n_(n)
{
   SetErrorText(ERR_SEQUENCE_INACTIVE, "Sequence triggered, but sequence is not running");

   // parent ID display
   CreateHubIDProperty();
}

////////////////////////////////////////////////////////////////////////////////
// MM::Device API
////////////////////////////////////////////////////////////////////////////////

// Copies the device name to the provided buffer.
void DemoDA::GetName(char* name) const
{
   if (n_ == 0)
   {
      CDeviceUtils::CopyLimitedString(name, g_DADeviceName);
   }
   else if (n_ == 1)
   {
      CDeviceUtils::CopyLimitedString(name, g_DA2DeviceName);
   }
   else // bad!
   {
      CDeviceUtils::CopyLimitedString(name, "ERROR");
   }
}

// Initialize the DA and its properties.
int DemoDA::Initialize()
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

   // Triggers to test sequence capabilities
   CPropertyAction* pAct = new CPropertyAction (this, &DemoDA::OnTrigger);
   CreateStringProperty("Trigger", "-", false, pAct);
   AddAllowedValue("Trigger", "-");
   AddAllowedValue("Trigger", "+");

   pAct = new CPropertyAction(this, &DemoDA::OnVoltage);
   CreateFloatProperty("Voltage", 0, false, pAct);
   SetPropertyLimits("Voltage", 0.0, 10.0);

   pAct = new CPropertyAction(this, &DemoDA::OnRealVoltage);
   CreateFloatProperty("Real Voltage", 0, true, pAct);

   return DEVICE_OK;
}

////////////////////////////////////////////////////////////////////////////////
// MM::SignalIO API
////////////////////////////////////////////////////////////////////////////////

int DemoDA::SetGateOpen(bool open) 
{
   open_ = open; 
   gatedVolts_ = (open_) ? volt_ : 0;
   return DEVICE_OK;
}

int DemoDA::GetGateOpen(bool& open) 
{
   open = open_; 
   return DEVICE_OK;
}

// Sets the output voltage
int DemoDA::SetSignal(double volts)
{
   volt_ = volts; 
   if (open_)
      gatedVolts_ = volts;
   std::stringstream s;
   s << "Voltage set to " << volts << " with the gate "
      << (open_) ? "open." : "closed.";
   LogMessage(s.str(), false);
   return DEVICE_OK;
}

// Gets the current output voltage
int DemoDA::GetSignal(double& volts) 
{
   volts = volt_; 
   return DEVICE_OK;
}

// Gets the voltage limits of the ADC/DAC device
int DemoDA::GetLimits(double& minVolts, double& maxVolts)
{
   minVolts=0.0; 
   maxVolts= 10.0;
   return DEVICE_OK;
}

// Lets the UI know whether or not this DA device accepts sequences
int DemoDA::IsDASequenceable(bool& isSequenceable) const
{
   isSequenceable = true;
   return DEVICE_OK;
}

// Gets the maximum length of a sequence that can be stored
int DemoDA::GetDASequenceMaxLength(long& nrEvents) const 
{
   nrEvents = 256;
   return DEVICE_OK;
}

// Starts sequence
int DemoDA::StartDASequence()
{
   (const_cast<DemoDA*>(this))->SetSequenceStateOn();
   return DEVICE_OK;
}

// Stops sequence
int DemoDA::StopDASequence()
{
   (const_cast<DemoDA*> (this))->SetSequenceStateOff();
   return DEVICE_OK;
}

// Sends the sequence composed using AddToDASequence() to the device
int DemoDA::SendDASequence() 
{
   (const_cast<DemoDA*> (this))->SetSentSequence();
   return DEVICE_OK;
}

// Clears the sequence to be set
// This does not clear the sequence already sent to the device using
// SetSentSequence()!
int DemoDA::ClearDASequence()
{
   nascentSequence_.clear();
   return DEVICE_OK;
}

// Add a voltage to the sequence
// To actually set the sequence to the device, SetSentSequence() must be called
int DemoDA::AddToDASequence(double voltage)
{
   nascentSequence_.push_back(voltage);
   return DEVICE_OK;
}

////////////////////////////////////////////////////////////////////////////////
// MM Action handlers
////////////////////////////////////////////////////////////////////////////////

// Property handler for Trigger property
// This is the main way the user will trigger the next voltage in the sequence
int DemoDA::OnTrigger(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set("-");
   }
   else if (eAct == MM::AfterSet) {
      if (!sequenceRunning_) { return ERR_SEQUENCE_INACTIVE; }
      std::string tr;

      pProp->Get(tr);
      if (tr == "+")
      {
         if (sequenceIndex_ < sentSequence_.size())
         {
            double voltage = sentSequence_[sequenceIndex_];
            int ret = SetSignal(voltage);
            if (DEVICE_OK != ret) { return ERR_IN_SEQUENCE; }

            sequenceIndex_++;
            if (sequenceIndex_ >= sentSequence_.size())
            {
               sequenceIndex_ = 0;
            }
         }
         else
         {
            return ERR_IN_SEQUENCE;
         }
      }
   }
   return DEVICE_OK;
}

// Property handler for Voltage property
// This is the main way the user will set the output voltage
int DemoDA::OnVoltage(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   int ret = DEVICE_OK;
   if (eAct == MM::BeforeGet)
   {
      double volts = 0.0;
      ret = GetSignal(volts);
      pProp->Set(volts);
   }
   else if (eAct == MM::AfterSet)
   {
      double volts = 0.0;
      pProp->Get(volts);
      ret = SetSignal(volts);
   }
   return ret;
}

// Read-only property showing the actual voltage output (considering gate state)
int DemoDA::OnRealVoltage(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(gatedVolts_);
   }
   return DEVICE_OK;
}

////////////////////////////////////////////////////////////////////////////////
// Utility methods
////////////////////////////////////////////////////////////////////////////////

void DemoDA::SetSentSequence()
{
   sentSequence_ = nascentSequence_;
   nascentSequence_.clear();
}

