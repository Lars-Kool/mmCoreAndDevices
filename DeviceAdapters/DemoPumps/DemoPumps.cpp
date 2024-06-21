///////////////////////////////////////////////////////////////////////////////
// FILE:          DemoSyringePump.cpp
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

#include "DeviceBase.h"
#include "DeviceThreads.h"
#include "ModuleInterface.h"
#include <string>
#include <map>
#include <algorithm>
#include <stdint.h>
#include <future>
#include "DemoPumps.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//  Global constants
///////////////////////////////////////////////////////////////////////////////

const char* g_DemoPumpHubName = "DemoPumpHub";
const char* g_DemoPumpName = "DemoPump";
const char* NoHubError = "Parent Hub not defined.";


///////////////////////////////////////////////////////////////////////////////
//  MMDevice API
///////////////////////////////////////////////////////////////////////////////


MODULE_API void InitializeModuleData()
{
    RegisterDevice(g_DemoPumpHubName, MM::HubDevice, "Hub for demo pumps.");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
    if (!deviceName)
    {
        return 0; // Trying to create nothing, return nothing
    }
    else if (strcmp(deviceName, g_DemoPumpHubName) == 0)
    {
        return new DemoPumpHub(); // Create Hub
    }
    // 8 is the length of the g_DemoPumpName name
    else if (strcmp(((string)deviceName).substr(0, 8).c_str(), g_DemoPumpName) == 0)
    {
        return new DemoPump(stoi(((string)deviceName).substr(8))); // Create pump
    }
    return 0; // If an unexpected name is provided, return nothing
}

MODULE_API void DeleteDevice(MM::Device* device)
{
    delete device;
}


///////////////////////////////////////////////////////////////////////////////
// DemoPumpHub class
// Hub for DemoPump devices
///////////////////////////////////////////////////////////////////////////////

DemoPumpHub::DemoPumpHub() :
    initialized_(false),
    busy_(false),
    nPumps_(0),
    port_("Undefined"),
    baudrate_(0)
{
    // Assign COM-port
    CPropertyAction* pAct;
    pAct = new CPropertyAction(this, &DemoPumpHub::OnPort);
    CreateStringProperty("Portal", port_.c_str(), false, pAct, true);

    // Assign number of pumps
    pAct = new CPropertyAction(this, &DemoPumpHub::OnNPumps);
    CreateIntegerProperty("Number of pumps", nPumps_, false, pAct, true);
}

DemoPumpHub::~DemoPumpHub() {
	Shutdown();
}

///////////////////////////////////////////////////////////////////////////////
// DemoPumpHub class
// MMDevice API
///////////////////////////////////////////////////////////////////////////////

int DemoPumpHub::Initialize()
{
    // Name
    int ret = CreateStringProperty(MM::g_Keyword_Name, g_DemoPumpHubName, true);
    if (DEVICE_OK != ret)
        return ret;

    // Description
    ret = CreateStringProperty(MM::g_Keyword_Description, "Hub for Demo pumps", true);
    if (DEVICE_OK != ret)
        return ret;

    // Establish connection
    ret = ConnectToPort();
    if (ret != DEVICE_OK) { return ret; }

    initialized_ = true;
    return DEVICE_OK;
}

int DemoPumpHub::Shutdown() {
    if (!initialized_) {
        return DEVICE_OK;
    }

    // Shut down all of the sub-pumps.
    // The pumps should handle their own safe shutdown.
    for (unsigned int i = 0; i < GetNumberOfInstalledDevices(); i++) {
        MM::Device* temp = GetInstalledDevice(i);
        if (temp->GetType() != MM::HubDevice) {
            temp->Shutdown();
        }
    }
    return DEVICE_OK;
}

bool DemoPumpHub::Busy() {
    return busy_;
}

void DemoPumpHub::GetName(char* name) const
{
    // Return the name used to refer to this device adapter
    CDeviceUtils::CopyLimitedString(name, g_DemoPumpHubName);
}

///////////////////////////////////////////////////////////////////////////////
// DemoPumpHub class
// MMHub API
///////////////////////////////////////////////////////////////////////////////

/**
* In this function the iterator "i" is also used as the id of the device, i.e.
* this index can be included in the serial command (or API) to address that
* specific pump. The implementation should be adapter to match the serial/API
* requirements of your specific pump.
*/
int DemoPumpHub::DetectInstalledDevices()
{
    ClearInstalledDevices();

    // make sure this method is called before we look for available devices
    InitializeModuleData();

    char hubName[MM::MaxStrLength];
    GetName(hubName); // Name of the hub
    for (int i = 0; i < nPumps_; i++)
    {
        string deviceName = (string)g_DemoPumpName + to_string(i);
        //RegisterDevice(deviceName.c_str(), MM::PumpDevice, "Demo Pump");
        MM::Device* pDev = CreateDevice(deviceName.c_str());
        AddInstalledDevice(pDev);

        char temp[MM::MaxStrLength];
        pDev->GetName(temp);
        LogMessage("Created pump: " + (string)temp);
    }
    return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// DemoPumpHub class
// Action handlers
///////////////////////////////////////////////////////////////////////////////

int DemoPumpHub::OnPort(MM::PropertyBase* pProp, MM::ActionType eAct) {
    switch (eAct) {
    case MM::BeforeGet:
        pProp->Set(port_.c_str());
        break;
    case MM::AfterSet:
        pProp->Get(port_);
        break;
    }
    return DEVICE_OK;
}

int DemoPumpHub::OnBaudrate(MM::PropertyBase* pProp, MM::ActionType eAct) {
    string temp = "";

    switch (eAct) {
    case MM::BeforeGet:
        temp = to_string(baudrate_);
        pProp->Set(temp.c_str());
        break;
    case MM::AfterSet:
        pProp->Get(temp);
        baudrate_ = stoi(temp);
        break;
    }
    return DEVICE_OK;
}

int DemoPumpHub::OnNPumps(MM::PropertyBase* pProp, MM::ActionType eAct) {
    string temp = "";

    switch (eAct) {
    case MM::BeforeGet:
        temp = to_string(nPumps_);
        pProp->Set(temp.c_str());
        break;
    case MM::AfterSet:
        pProp->Get(temp);
        nPumps_ = stoi(temp);
        break;
    }
    return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// DemoPumpHub class
// Utility methods
///////////////////////////////////////////////////////////////////////////////

/**
* Here you should establish the connection to the COM port, and check whether
* the provided COM-port is correct, e.g. by pinging the port with an expected
* return. If the connection is successful, return DEVICE_OK. Else,
* return DEVICE_SERIAL_COMMAND_FAILED.
* 
* Logging a message on success or failure is encouraged.
*/
int DemoPumpHub::ConnectToPort() {
    LogMessage("Hub connected successfully to port: " + port_ + " with baudrate: " + to_string(baudrate_));
    return DEVICE_OK;
}

int DemoPumpHub::GetPort(string& port) {
    port = port_;
    return DEVICE_OK;
}

int DemoPumpHub::GetBaudrate(int& baudrate) {
    baudrate = baudrate_;
    return DEVICE_OK;
}



///////////////////////////////////////////////////////////////////////////////
// DemoPump class
// A demo pump
///////////////////////////////////////////////////////////////////////////////

DemoPump::DemoPump(int idx) :
    initialized_(false),
    busy_(false),
    port_("Undefined"),
    baudrate_(0),
    minVolumeUl_(0),
    maxVolumeUl_(1),
    volumeUl_(0),
    diameter_(0),
    flowrateUlperSecond_(0),
    direction_(1)
{
    // Set pump id and name
    id_ = idx;
    name_ = ((string)g_DemoPumpName) + to_string(id_);
    thd_ = new PumpThread(this);

    // parent ID display
    //CreateHubIDProperty();
}

DemoPump::~DemoPump() {
    Shutdown();
};

///////////////////////////////////////////////////////////////////////////////
// DemoPump class
// MMDevice API
///////////////////////////////////////////////////////////////////////////////

int DemoPump::Initialize() {
    if (initialized_)
        return DEVICE_OK;

    // Link to parent Hub
    DemoPumpHub* pHub = static_cast<DemoPumpHub*>(GetParentHub());
    if (pHub)
    {
        char hubLabel[MM::MaxStrLength];
        pHub->GetLabel(hubLabel);
        SetParentID(hubLabel); // for backward comp.
    }
    else
        LogMessage(NoHubError);

    // Name
    int ret = CreateStringProperty(MM::g_Keyword_Name, name_.c_str(), true);
    if (DEVICE_OK != ret)
        return ret;

    // Description
    ret = CreateStringProperty(MM::g_Keyword_Description, "Demo syringe pump", true);
    if (DEVICE_OK != ret)
        return ret;

    // Set COM Port
    pHub->GetPort(port_);

    // Set communication parameters (obtained from Hub)
    CPropertyAction* pAct;

    // Set minVolume
    pAct = new CPropertyAction(this, &DemoPump::OnMinVolume);
    ret = CreateFloatProperty("Min Volume uL", minVolumeUl_, false, pAct);

    // Set maxVolume
    pAct = new CPropertyAction(this, &DemoPump::OnMaxVolume);
    ret = CreateFloatProperty("Max Volume uL", maxVolumeUl_, false, pAct);

    // Set currentVolume
    pAct = new CPropertyAction(this, &DemoPump::OnCurrentVolume);
    ret = CreateFloatProperty("Current Volume uL", volumeUl_, false, pAct);

    // Set diameter
    pAct = new CPropertyAction(this, &DemoPump::OnDiameter);
    ret = CreateFloatProperty("Diameter mm", diameter_, false, pAct);

    // Set stepSize
    pAct = new CPropertyAction(this, &DemoPump::OnStepSize);
    ret = CreateFloatProperty("Step size mm", stepSize_, false, pAct);

    // Set direction
    vector<string> allowedDirections = { "1", "-1" };
    pAct = new CPropertyAction(this, &DemoPump::OnDirection);
    ret = CreateIntegerProperty("Direction", direction_, false, pAct);
    SetAllowedValues("Direction", allowedDirections);

    // Set flowrate
    pAct = new CPropertyAction(this, &DemoPump::OnFlowrate);
    ret = CreateFloatProperty("Flow rate uL/min", flowrateUlperSecond_, false, pAct);

    // Ping specific pump to check whether connection is established
    ret = CheckConnection();

    initialized_ = true;
    return DEVICE_OK;
}

int DemoPump::Shutdown() {
    if (!initialized_) {
        return DEVICE_OK;
    }

    if (Busy()) {
        Stop();
    }

    initialized_ = false;
    return DEVICE_OK;
}

bool DemoPump::Busy() {
    busy_ = !thd_->IsStopped();
    return busy_;
}



void DemoPump::GetName(char* name) const {
    // Return the name used to refer to this device adapter
    CDeviceUtils::CopyLimitedString(name, name_.c_str());
}

///////////////////////////////////////////////////////////////////////////////
// DemoPump class
// MMPump API
///////////////////////////////////////////////////////////////////////////////

int DemoPump::GetPort(string& port) {
    port = port_;
    return DEVICE_OK;
}

int DemoPump::Home() {
    if (Busy()) {
        return DEVICE_PUMP_IS_RUNNING;
    }

    MMThreadGuard g(this->currentVolumeLock_);
    volumeUl_ = 0;
    return DEVICE_OK;
}

int DemoPump::Stop() {
    // If pump is not running, no need to stop it.
    if (!Busy()) {
        return DEVICE_OK;
    }

    thd_->Stop();
    busy_ = thd_->IsStopped();
    return DEVICE_OK;
}

int DemoPump::GetMaxVolumeUl(double& volUl) {
    volUl = maxVolumeUl_;
    return DEVICE_OK;
}

int DemoPump::SetMaxVolumeUl(double volUl) {
    maxVolumeUl_ = volUl;
    SetPropertyLimits("Current Volume uL", minVolumeUl_, maxVolumeUl_);
    return DEVICE_OK;
}

int DemoPump::GetVolumeUl(double& volUl) {
    MMThreadGuard g(this->currentVolumeLock_);
    volUl = volumeUl_;
    return DEVICE_OK;
}

int DemoPump::SetVolumeUl(double volUl) {
    if (Busy())
        return DEVICE_PUMP_IS_RUNNING;

    MMThreadGuard g(this->currentVolumeLock_);
    volumeUl_ = volUl;
    return DEVICE_OK;
}

int DemoPump::IsDirectionInverted(bool& invert) {
    invert = direction_;
    return DEVICE_OK;
}

int DemoPump::InvertDirection(bool invert) {
    if (Busy())
        return DEVICE_PUMP_IS_RUNNING;
    direction_ = invert;
    return DEVICE_OK;
}

int DemoPump::GetDiameter(double& diam) {
    diam = diameter_;
    return DEVICE_OK;
}

int DemoPump::SetDiameter(double diam) {
    if (Busy())
        return DEVICE_PUMP_IS_RUNNING;
    diameter_ = diam;
    return DEVICE_OK;
}

int DemoPump::GetStepSize(double& stepSize) {
    stepSize = stepSize_;
    return DEVICE_OK;
}

int DemoPump::SetStepSize(double stepSize) {
    if (Busy())
        return DEVICE_PUMP_IS_RUNNING;
    stepSize_ = stepSize;
    return DEVICE_OK;
}

int DemoPump::GetFlowrateUlPerSecond(double& flowrate) {
    flowrate = flowrateUlperSecond_;
    return DEVICE_OK;
}

int DemoPump::SetFlowrateUlPerSecond(double flowrate) {
    if (Busy())
        return DEVICE_PUMP_IS_RUNNING;
    flowrateUlperSecond_ = flowrate;
    return DEVICE_OK;
}

int DemoPump::Dispense() {
    if (Busy())
        return DEVICE_PUMP_IS_RUNNING;

    double seconds = 0;
    if (flowrateUlperSecond_ >= 0) {
        seconds = volumeUl_ / flowrateUlperSecond_;
    }
    else {
        seconds = (maxVolumeUl_ - volumeUl_) / abs(flowrateUlperSecond_);
    }
    DispenseDuration(seconds);
    return DEVICE_OK;
}

int DemoPump::DispenseVolume(double volUl) {
    if (Busy())
        return DEVICE_PUMP_IS_RUNNING;

    // Calculate duration based on volume and flowrate
    double seconds = (volUl / flowrateUlperSecond_);
    if (seconds < 0) {
        LogMessage("Negative dispense/withdraw duration. Check the sign of the flowrate and volume");
        return DEVICE_ERR;
    }
    DispenseDuration(seconds);
    return DEVICE_OK;
}

int DemoPump::DispenseDuration(double seconds) {
    if (Busy())
        return DEVICE_PUMP_IS_RUNNING;

    MMThreadGuard g(this->currentVolumeLock_);

    startVolume_ = volumeUl_;
    duration_ = seconds; // Convert seconds to milliseconds, since clock is in ms.
    if (duration_ < 0) {
        LogMessage("Negative dispense/withdraw duration. Check the sign of the flowrate and volume");
        return DEVICE_ERR;
    }
    busy_ = true;
    thd_->Start(duration_);
    return DEVICE_OK;
}

/**
* Actual updating in separate function to release the threadlocks asap.
*/
int DemoPump::UpdateVolume(double dt) {
    MMThreadGuard g1(this->currentVolumeLock_);

    // 1000 to convert dt (which is in ms) to second
    volumeUl_ = startVolume_ - flowrateUlperSecond_ * dt;
    return DEVICE_OK;
}

int DemoPump::RunOnThread(double dt) {
    // The updating executes very fast, potentially straining the CPU. Hence
    // we limit the refresh rate to 100/s. This value is chosen relatively
    // arbitrarily, so it could be updated in the future.
    UpdateVolume(dt);
    CDeviceUtils::SleepMs(10);
    return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// DemoPump class
// Action Handlers
///////////////////////////////////////////////////////////////////////////////

int DemoPump::OnMinVolume(MM::PropertyBase* pProp, MM::ActionType eAct) {
    switch (eAct) {
    case MM::BeforeGet:
        pProp->Set(minVolumeUl_);
        break;
    case MM::AfterSet:
        if (Busy()) {
            return DEVICE_PUMP_IS_RUNNING;
        }
        string temp;
        pProp->Get(temp);
        minVolumeUl_ = stod(temp);
        SetPropertyLimits("Current Volume uL", minVolumeUl_, maxVolumeUl_);
        break;
    }
    return DEVICE_OK;
}

int DemoPump::OnMaxVolume(MM::PropertyBase* pProp, MM::ActionType eAct) {
    switch (eAct) {
    case MM::BeforeGet:
        pProp->Set(maxVolumeUl_);
        break;
    case MM::AfterSet:
        if (Busy()) {
            return DEVICE_PUMP_IS_RUNNING;
        }
        string temp;
        pProp->Get(temp);
        maxVolumeUl_ = stod(temp);
        SetPropertyLimits("Current Volume uL", minVolumeUl_, maxVolumeUl_);
        break;
    }
    return DEVICE_OK;
}

int DemoPump::OnCurrentVolume(MM::PropertyBase* pProp, MM::ActionType eAct) {
    MMThreadGuard g(this->currentVolumeLock_);
    switch (eAct) {
    case MM::BeforeGet:
        pProp->Set(volumeUl_);
        break;
    case MM::AfterSet:
        if (Busy()) {
            return DEVICE_PUMP_IS_RUNNING;
        }
        string temp;
        pProp->Get(temp);
        volumeUl_ = stod(temp);
        break;
    }
    return DEVICE_OK;
}

int DemoPump::OnDiameter(MM::PropertyBase* pProp, MM::ActionType eAct) {
    switch (eAct) {
    case MM::BeforeGet:
        pProp->Set(diameter_);
        break;
    case MM::AfterSet:
        if (Busy()) {
            return DEVICE_PUMP_IS_RUNNING;
        }
        string temp;
        pProp->Get(temp);
        diameter_ = stod(temp);
        break;
    }
    return DEVICE_OK;
}

int DemoPump::OnStepSize(MM::PropertyBase* pProp, MM::ActionType eAct) {
    switch (eAct) {
    case MM::BeforeGet:
        pProp->Set(stepSize_);
        break;
    case MM::AfterSet:
        if (Busy()) {
            return DEVICE_PUMP_IS_RUNNING;
        }
        string temp;
        pProp->Get(temp);
        stepSize_ = stod(temp);
        break;
    }
    return DEVICE_OK;
}

int DemoPump::OnDirection(MM::PropertyBase* pProp, MM::ActionType eAct) {
    switch (eAct) {
    case MM::BeforeGet:
        if (direction_) {
            pProp->Set((long)-1);
        }
        break;
    case MM::AfterSet:
        if (Busy()) {
            return DEVICE_PUMP_IS_RUNNING;
        }
        string temp;
        pProp->Get(temp);
        direction_ = temp == "-1";
        break;
    }
    return DEVICE_OK;
}

int DemoPump::OnFlowrate(MM::PropertyBase* pProp, MM::ActionType eAct) {
    switch (eAct) {
    case MM::BeforeGet:
        pProp->Set(flowrateUlperSecond_);
        break;
    case MM::AfterSet:
        if (Busy()) {
            return DEVICE_PUMP_IS_RUNNING;
        }
        string temp;
        pProp->Get(temp);
        flowrateUlperSecond_ = stod(temp);
        break;
    }
    return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// DemoPumpHub class
// Utility methods
///////////////////////////////////////////////////////////////////////////////

/**
* The hub should already have established the connection. Here you just check
* if the device adapter can communicate with the pump. Note this is just a
* dummy method.
*
* Logging a message on success or failure is encouraged.
*/
int DemoPump::CheckConnection() {
    // Send command with known response
    string response = Ping();
    // Check if response matches the expected value
    if (response == "Success") {
        LogMessage("Ping of pump: " + to_string(id_) + " succesful.");
        return DEVICE_OK;
    }
    else {
        LogMessage("Ping of pump: " + to_string(id_) + " failed.");
        return DEVICE_ERR;
    }
}

/**
* Send command with known reponse. Useful to check connection. Note this is
* just a dummy method. The return of the implementation doesn't matter, the
* only requirement is that it should be known in advance, and change depending
* on whether the pump is successfully connected or not.
* 
* Options could include "get version" (either of device or firmware),
* "is running".
*/
string DemoPump::Ping() {
    return "Success";
}

///////////////////////////////////////////////////////////////////////////////
// PumpThread class
// Thread to run pump (prevents blocking of main thread).
///////////////////////////////////////////////////////////////////////////////

PumpThread::PumpThread(DemoPump* pPump) {
    pump_ = pPump;
};

PumpThread::~PumpThread() {};

///////////////////////////////////////////////////////////////////////////////
// PumpThread class
// MMPump API
///////////////////////////////////////////////////////////////////////////////

void PumpThread::Start(double duration) {
    MMThreadGuard g(this->stopLock_);
    duration_ = duration;
    startTime_ = pump_->GetCurrentMMTime();
    stop_ = false;
    pump_->LogMessage("Thread is started");
    activate();
};

void PumpThread::Stop() {
    MMThreadGuard g(this->stopLock_);
    stop_ = true;
};

bool PumpThread::IsStopped() {
    MMThreadGuard g(this->stopLock_);
    return stop_;
}

///////////////////////////////////////////////////////////////////////////////
// PumpThread class
// MMDeviceThreadBase API
///////////////////////////////////////////////////////////////////////////////

int PumpThread::svc(void) throw() {
    int ret = DEVICE_ERR;
    try
    {
        do
        {
            MM::MMTime currentTime = pump_->GetCurrentMMTime();
            dt_ = (currentTime - startTime_).getMsec()/1000; // Convert ms to seconds
            ret = pump_->RunOnThread(dt_);
        } while (DEVICE_OK == ret && !IsStopped() && dt_ < duration_);
        if (IsStopped())
            pump_->LogMessage("Pump stopped by the user.\n");
        if (dt_ >= duration_) {
            pump_->LogMessage("Dispense/withdrawal finished.\n"); 
        }
    }
    catch (...) {
        pump_->LogMessage(g_Msg_EXCEPTION_IN_THREAD);
    }
    stop_ = true;
    return ret;
}