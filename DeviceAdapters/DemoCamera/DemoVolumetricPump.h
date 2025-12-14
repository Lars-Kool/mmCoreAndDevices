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

class DemoVolumetricPump : public CVolumetricPumpBase<DemoVolumetricPump>
{
public:
   DemoVolumetricPump() :
      busy_ (false),
      initialized_ (false),
      currentVolumeUl_(0.0),
      maxVolumeUl_ (200.0),
      flowRateUlpS_ (10.0)
   {};
   ~DemoVolumetricPump() {
      if (initialized_)
         Shutdown();
   };

   void GetName(char* name) const {strcpy(name, g_VolumetricPumpDeviceName);}

   bool Busy() {return busy_;}

   int Initialize() {
      initialized_ = true;
      return DEVICE_OK;
   }

   int Shutdown() {
      initialized_ = false;
      return DEVICE_OK;
   }


    /**
     * Homes the pump. If no homing is supported, just return
     * DEVICE_UNSUPPORTED_COMMAND.
     *
     * Optional function of VolumetricPump API
     */
   int Home() { return DEVICE_UNSUPPORTED_COMMAND; };

    /**
     * Stops the pump. The implementation should halt any dispensing/withdrawal,
     * and make the pump available again (make Busy() return false).
     *
     * Required function of VolumetricPump API
     */
   int Stop() {
      // TODO: implement!
      return DEVICE_OK;
   }

    /**
     * Flag to check whether the pump requires homing before being operational
     *
     * Required function of VolumetricPump API
     */
    bool RequiresHoming() { return false; };

    /**
     * Sets the direction of the pump. Certain pump
     * (e.g. peristaltic and DC pumps) don't have an apriori forward-reverse direction,
     * as it depends on how it is connected. This function allows you to switch
     * forward and reverse.
     *
     * If the pump is uni-directional, this function does not need to be
     * implemented (return DEVICE_UNSUPPORTED_COMMAND).
     *
     * Optional function of VolumetricPump API
     */
    int InvertDirection(bool /* inverted */) { return DEVICE_UNSUPPORTED_COMMAND; };

    /**
     * Sets the direction of the pump. Certain pump
     * (e.g. peristaltic and DC pumps) don't have an apriori forward-reverse direction,
     * as it depends on how it is connected. This function allows you to switch
     * forward and reverse.
     *
     * When the pump is uni-directional, this function should always assign
     * false to `inverted`
     *
     * Required function of VolumetricPump API
     */
    int IsDirectionInverted(bool& inverted) {
       inverted = false;
       return DEVICE_OK;
    };

    /**
     * Sets the current volume of the pump in microliters (uL).
     *
     * Required function of VolumetricPump API
     */
    int SetVolumeUl(double volUl) {
       currentVolumeUl_ = volUl;
       return DEVICE_OK;
    };

    /**
     * Gets the current volume of the pump in microliters (uL).
     *
     * Required function of VolumetricPump API
     */
    int GetVolumeUl(double& volUl) {
       volUl = currentVolumeUl_;
       return DEVICE_OK;
    };

    /**
     * Sets the maximum volume of the pump in microliters (uL).
     *
     * Required function of VolumetricPump API
     */
    int SetMaxVolumeUl(double volUl) {
       maxVolumeUl_ = volUl;
       return DEVICE_OK;
    };

    /**
     * Gets the maximum volume of the pump in microliters (uL).
     *
     * Required function of VolumetricPump API
     */
    int GetMaxVolumeUl(double& volUl) {
       volUl = maxVolumeUl_;
       return DEVICE_OK;
    };

    /**
     * Sets the flowrate in microliter (uL) per second. The implementation
     * should convert the provided flowrate to whichever unit the pump desires
     * (steps/s, mL/h, V).
     *
     * Required function of VolumetricPump API
     */
    int SetFlowrateUlPerSecond(double flowrate) {
       flowRateUlpS_ = flowrate;
       return DEVICE_OK;
    };

    /**
     * Gets the flowrate in microliter (uL) per second.
     *
     * Required function of VolumetricPump API
     */
    int GetFlowrateUlPerSecond(double& flowrate) {
       flowrate = flowRateUlpS_;
       return DEVICE_OK;
    };

    /**
     * Dispenses/withdraws until the minimum or maximum volume has been
     * reached, or the pumping is manually stopped
     *
     * Required function of VolumetricPump API
     */
    int Start() {
       // TODO run a thread, time things, and set variables accordingly.  Will need a mutex on our variables
       return DEVICE_OK;
    };

    /**
     * Dispenses/withdraws for the provided time, with the flowrate provided
     * by GetFlowrate_uLperMin
     * Dispensing for an undetermined amount of time can be done with DBL_MAX
     * During the dispensing/withdrawal, Busy() should return "true".
     *
     * Required function of VolumetricPump API
     */
    int DispenseDurationSeconds(double /*durSec */) {
       // TODO run a thread, time things, and set variables accordingly.  Will need a mutex on our variables
       return DEVICE_OK;
    };

    /**
     * Dispenses/withdraws the provided volume.
     *
     * The implementation should cause positive volumes to be dispensed, whereas
     * negative volumes should be withdrawn. The implementation should prevent
     * the volume to go negative (i.e. stop the pump once the syringe is empty),
     * or to go over the maximum volume (i.e. stop the pump once it is full).
     * This automatically allows for dispensing/withdrawal for an undetermined
     * amount of time by providing DBL_MAX for dispense, and DBL_MIN for
     * withdraw.
     *
     * During the dispensing/withdrawal, Busy() should return "true".
     *
     * Required function of VolumetricPump API
     */
    int DispenseVolumeUl(double /* volUl */) {
       // TODO run a thread, time things, and set variables accordingly.  Will need a mutex on our variables
       return DEVICE_OK;
    };

    private:
       bool busy_;
       bool initialized_;
       double currentVolumeUl_;
       double maxVolumeUl_;
       double flowRateUlpS_;
};

