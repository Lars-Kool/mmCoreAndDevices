///////////////////////////////////////////////////////////////////////////////
// FILE:          DemoSyringePump.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Demo for syringe pump devices
//                
// AUTHOR:        Lars Kool, Institut Pierre-Gilles de Gennes
//
// YEAR:          2024
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
//LAST UPDATE:    23.02.2024 LK

#ifndef _WPI_PUMP_H_
#define _WPI_PUMP_H_

#include "DeviceBase.h"
#include "DeviceThreads.h"
#include "ModuleInterface.h"
#include <string>
#include <map>
#include <algorithm>
#include <stdint.h>
#include <future>

using namespace std;

enum flowrate_units {
    mL_min,
    mL_hr,
    uL_min,
    uL_hr
};

///////////////////////////////////////////////////////////////////////////////
// WPIPumpHub class
// Hub for WPI pumps
///////////////////////////////////////////////////////////////////////////////

class WPIPumpHub : public HubBase<WPIPumpHub>
{
    //friend class DemoPump;
public:
    WPIPumpHub();
    ~WPIPumpHub();

    // Device API
    int Initialize();
    int Shutdown();
    void GetName(char* pName) const;
    bool Busy();

    // HUB API
    int DetectInstalledDevices();

    // Hub action handlers
    //int OnPort(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnBaudrate(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnNPumps(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnPort(MM::PropertyBase* pProp, MM::ActionType eAct);

    // Hub public utitility methods
    int GetPort(string& port);
    int GetBaudrate(int& baudrate);

    // Hub class variables

private:
    // Hub private utility methods
    int ConnectToPort();
    int Ping(int idx);

    // Hub class variables
    bool initialized_;
    bool busy_;
    string port_ = "";
    int baudrate_;
    int nPumps_;
};

//////////////////////////////////////////////////////////////////////////////
// DemoPump class
// A demo pump
//////////////////////////////////////////////////////////////////////////////
class WPIPump : public CPumpBase<WPIPump>
{
    friend class PumpThread;

public:
    WPIPump(int idx);
    ~WPIPump();

    // MMDevice API
    int Initialize();
    int Shutdown();
    bool Busy();
    void GetName(char* name) const;

    // MMPump API
    int GetPort(string& port);
    int Home();
    int Stop();
    int GetMaxVolumeUl(double& volUl);
    int SetMaxVolumeUl(double volUl);
    int GetVolumeUl(double& volUl);
    int SetVolumeUl(double volUl);
    int IsDirectionInverted(bool& invert);
    int InvertDirection(bool invert);
    int GetDiameter(double& V);
    int SetDiameter(double V);
    int GetFlowrateUlPerSecond(double& flowrate);
    int SetFlowrateUlPerSecond(double flowrate);
    int Dispense();
    int DispenseVolume(double volUl);
    int DispenseDuration(double seconds);

    int RunOnThread(double dt);
    int UpdateVolume(double dt);

    // Action Handlers
    int OnMaxVolume(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnCurrentVolume(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnDiameter(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnDirection(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnFlowrate(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnRun(MM::PropertyBase* pProp, MM::ActionType eAct);

    // Utility methods
    int CheckConnection();
    string Ping();
    int Send(string cmd);
    int ReceiveOneLine(string& ans);
    int Purge();
    int AdjustUnits(double flowrate);
    double ConvertFlowrate(double flowrate, int flowrate_unit);
    string GetUnitString();
    bool IsPumping();

private:
    // Communication class variables
    bool initialized_;
    bool busy_;
    string port_;
    int baudrate_;
    int id_;
    string name_;

    // Pump state class variables
    double minVolumeUl_ = 0;
    double maxVolumeUl_;
    double volumeUl_;
    double diameter_;
    double stepSize_;
    double flowrateUlperSecond_;
    long direction_;
    long flowrate_unit_;
    long run_;
    

    // Pump thread related
    PumpThread* thd_;
    double duration_;
    double startVolume_;

    MMThreadLock currentVolumeLock_;
    MMThreadLock durationLock_;
};

class PumpThread : public MMDeviceThreadBase
{
public:
    PumpThread(WPIPump* pPump);
    ~PumpThread();

    void Start(double duration);
    void Stop();
    bool IsStopped();

private:
    WPIPump* pump_;
    MMThreadLock stopLock_;
    bool stop_ = true;
    double duration_ = 0;
    double dt_ = 0;
    MM::MMTime startTime_;

    int svc(void) throw();
};

#endif //_WPI_PUMP_H_