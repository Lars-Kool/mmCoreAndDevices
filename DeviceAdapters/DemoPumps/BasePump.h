///////////////////////////////////////////////////////////////////////////////
// FILE:          BasePump.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Base class for various pumps (syringe pump, peristalic
//                pump etc)
//                
// AUTHOR:        Lars Kool, Institut Pierre-Gilles de Gennes
//
// YEAR:          2023
//                
// VERSION:       0.1
//
// LICENSE:       This file is distributed under the BSD license.
//                License text is included with the source distribution.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE   LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
//
//LAST UPDATE:    12.02.2024 LK

#ifndef _BASE_PUMP_H_
#define _BASE_PUMP_H_

#include "DeviceBase.h"
#include "DeviceThreads.h"
#include <string>
#include <map>
#include <algorithm>
#include <stdint.h>
#include <future>

using namespace std;

/**
* Some global names of properties
**/
const char* g_Keyword_Fluid_device = "Fluid controller";
const char* g_Pressure_controller = "Pressure controller";
const char* g_Pump = "Pump";

template <class T>
class CPumpHub : public HubBase<T>
{
public:
	CPumpHub() {}

	virtual ~CPumpHub() {}

	/**
	* Required by MMDevice API, return whether the device is busy executing a command
	*/
	virtual bool Busy() = 0 {}

	/**
	* Required by MMDevice API, initializes the device.
	*/
	virtual int Initialize() { return DEVICE_OK; }

	/**
	* Required by MMDevice API, is called upon shutdown of the device and/or MM.
	*/
	virtual int Shutdown() { return DEVICE_OK; }

	/**
	* Required by MMHub API. This allows daisy-chained pumps to be initialized
	* all at once.
	*/
	virtual int DetectInstalledDevices() = 0;
};

template <class T>
class CPump : public CDeviceBase<MM::Generic, T>
{
public:
	CPump() {}

	virtual ~CPump() {}

	/**
	* Required by MMDevice API, return whether the device is busy executing a command
	*/
	virtual bool Busy() = 0;

	/**
	* Initializes the device.
	* 
	* Required by MMDevice API
	*/
	virtual int Initialize() = 0;

	/**
	* Shutdown is called upon shutdown of the device and/or MM. It should
	* reverse Initialize. Multiple calls to Shutdown() should not throw an
	* exception.
	* 
	* Required by MMDevice API
	*/
	virtual int Shutdown() = 0;

	/**
	* Gets the COM port. There is no SetPort, as the port should be set in
	* Initialize(), and should be read-only afterwards. If the device doesn't
	* use serial commands (but instead a C++ API e.g.), there is no need to
	* implement this.
	* 
	* If the device shares the COM port with other pumps (daisy-chaining), we
	* recommend that the only the Hub has the COM port as a property, and all
	* sub-pumps only have it as a private class variable, to reduce the total
	* number of properties.
	* 
	* Required by MMPump API.
	*/
	virtual int GetPort(string& port) = 0;

	/**
	* Homes the pump. This function should move the pump to its
	* "currentVolume = 0", and then set currentVolume to 0.
	* 
	* Required by MMPump API. 
	*/
	virtual int Home() = 0;

	/**
	* Stops the pump. The implementation should halt any dispensing/withdrawal,
	* and make the pump available again (make Busy() return false).
	* 
	* Required by MMPump API. 
	*/
	virtual int Stop() = 0;

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
	* Required by MMPump API.
	*/
	virtual int Dispense_uL(double V) = 0;

	/**
	* Dispenses/withdraws for the provided time.
	* Dispensing for an undetermined amount of time can be done with DBL_MAX
	* During the dispensing/withdrawal, Busy() should return "true".
	*
	* Required by MMPump API.
	*/
	virtual int Dispense_seconds(double T) = 0;

	/**
	* Gets the maximum volume of the pump in microliters (uL).
	* 
	* Required by MMPump API.
	*/
	virtual int GetMaxVolume(double& V) = 0;

	/**
	* Sets the maximum volume of the pump in microliters (uL).
	* 
	* Required by MMPump API.
	*/
	virtual int SetMaxVolume(double V) = 0;

	/**
	* Gets the current volume of the pump in microliters (uL).
	* 
	* Required by MMPump API.
	*/
	virtual int GetCurrentVolume(double& V) = 0;

	/**
	* Sets the current volume of the pump in microliters (uL).
	* 
	* Required by MMPump API. 
	*/
	virtual int SetCurrentVolume(double V) = 0;

	/**
	* Sets the direction of the pump. Certain pump
	* (e.g. peristaltic and DC pumps) don't have an apriori forward-reverse direction,
	* as it depends on how it is connected. This function allows you to switch
	* forward and reverse.
	*
	* The implementation of this function should allow two values, [1] and [-1],
	* and should ignore all other values, where [1] indicates that the direction
	* is left as-is, and [-1] indicates that the direction should be reversed.
	* When the pump is uni-directional, the function should always assign [1] to
	* [direction]
	* 
	* Required by MMPump API.
	*/
	virtual int GetDirection(long& direction) = 0;

	/**
	* Sets the direction of the pump. Certain pump
	* (e.g. peristaltic and DC pumps) don't have an apriori forward-reverse direction,
	* as it depends on how it is connected. This function allows you to switch
	* forward and reverse.
	*
	* The implementation of this function should allow two values, 1 and -1,
	* and should ignore all other values, where 1 indicates that the direction
	* is left as-is, and -1 indicates that the direction should be reversed. If
	* the pump is uni-directional, this function does not need to be
	* implemented.
	* 
	* Required by MMPump API.
	*/
	virtual int SetDirection(long direction) = 0;

	/**
	* Gets the inner diameter of the syringe in mm.
	* 
	* Required by MMPump API. 
	*/
	virtual int GetDiameter(double& diam) = 0;

	/**
	* Sets the internal diameter of the syringe in mm.
	* The syringe diameter can be used as conversion factor between volume and
	* a number of steps. Commercial syringe pumps often do this conversion
	* internally. DIY syringe pumps will most likely need a second calibration
	* factor, the linear displacement per step (stepSize).
	* The volume would then be: V = pi * (diam/2)^2 * stepSize * nSteps
	* 
	* Required by MMPump API. 
	*/
	virtual int SetDiameter(double diam) = 0;

	/**
	* Gets the linear displacement of the plunger per step of the stepper motor
	* in mm/step. If your device handles the conversion from volume to nSteps
	* internally, just don't implement this function.
	* 
	* Optional method of MMPump API. 
	*/
	virtual int GetStepSize(double& stepSize) = 0;
	
	/**
	* Sets the linear displacement of the plunger per step of the stepper motor
	* in mm/step. Internally, this value can be used to convert Volume to
	* number of steps (and viceversa) according to:
	* V = pi * (diam/2)^2 * stepSize * nSteps
	* If your device handles the conversion from volume to nSteps internally,
	* just don't implement this function.
	* 
	* Optional method of MMPump API.
	*/
	virtual int SetStepSize(double stepSize) = 0;

	/**
	* Gets the flowrate in microliter (uL) per minute.
	* 
	* Required by MMPump API.
	*/
	virtual int GetFlowRateUlPerMinute(double& flowrate) = 0;

	/**
	* Sets the flowrate in microliter (uL) per minute. The implementation
	* should convert the provided flowrate to whichever unit the pump desires
	* (steps/s, mL/h, V).
	* 
	* Required by MMPump API.
	*/
	virtual int SetFlowRateUlPerMinute(double flowrate) = 0;
};
#endif //_BASE_PUMP_H_