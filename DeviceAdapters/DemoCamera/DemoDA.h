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

#pragma once

#include "DeviceBase.h"
#include "DemoHub.h"

class DemoDA : public CSignalIOBase<DemoDA>
{
public:
   DemoDA(uint8_t n);
   ~DemoDA() {};

   // MM::Device API
   void GetName(char* name) const;
   bool Busy() { return false; }
   int Initialize();
   int Shutdown() { return DEVICE_OK; }

   // MM::SignalIO API
   int SetGateOpen(bool open);
   int GetGateOpen(bool& open);
   int SetSignal(double volts);
   int GetSignal(double& volts);
   int GetLimits(double& minVolts, double& maxVolts);

   // Sequence functions
   int IsDASequenceable(bool& isSequenceable) const;
   int GetDASequenceMaxLength(long& nrEvents) const;
   int StartDASequence();
   int StopDASequence();
   int SendDASequence();
   int ClearDASequence();
   int AddToDASequence(double voltage);

   // MM Action handlers
   int OnTrigger(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnVoltage(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnRealVoltage(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   // Utility methods
   void SetSequenceStateOn() { sequenceRunning_ = true; }
   void SetSequenceStateOff() { sequenceRunning_ = false; sequenceIndex_ = 0; }
   void SetSentSequence();

   // Data members
   uint8_t n_{};
   double volt_ = 0;
   double gatedVolts_ = 0;
   bool open_ = true;
   bool sequenceRunning_ = false;
   unsigned long sequenceIndex_ = 0;
   std::vector<double> sentSequence_;
   std::vector<double> nascentSequence_;
};

