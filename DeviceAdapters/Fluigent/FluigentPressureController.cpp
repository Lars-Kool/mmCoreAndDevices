///////////////////////////////////////////////////////////////////////////////
// FILE:          FluigentPressureController.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Device adapter for Fluigent pressure controllers
//                
// AUTHOR:        Lars Kool, Institut Pierre-Gilles de Gennes
//
// YEAR:          2023
//                
// VERSION:       1.0
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
//LAST UPDATE:    18.10.2023 LK

#include "DeviceBase.h"
#include "DeviceThreads.h"
#include "ModuleInterface.h"
#include <string>
#include <map>
#include <algorithm>
#include <stdint.h>
#include <future>

#include "fgt_SDK.h"
#include "FluigentPressureController.h"

const char* g_FluigentChannelName = "FluigentChannel";
const char* g_FluigentHubName = "FluigentHub";
const char* g_Calibrate = "Calibrate";
const char* g_Imposed = "Imposed Pressure";
const char* g_Measured = "Measured Pressure";

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//  MMDevice API
///////////////////////////////////////////////////////////////////////////////


MODULE_API void InitializeModuleData()
{
    RegisterDevice(g_FluigentHubName, MM::HubDevice, "Hub for Fluigent pressure controllers");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
    if (!deviceName)
    { 
        return 0; // Trying to create nothing, return nothing
    }
    if (strcmp(deviceName, g_FluigentHubName) == 0)
    {
        return new FluigentHub(); // Create Hub
    }
    if (strncmp(deviceName, g_FluigentChannelName, strlen(g_FluigentChannelName)) == 0)
    {
        int idx = stoi(((string)deviceName).substr(strlen(g_FluigentChannelName)));
        return new FluigentChannel(idx);
    }
    return 0; // If an unexpected name is provided, return nothing
}

MODULE_API void DeleteDevice(MM::Device* device)
{
    delete device;
}

///////////////////////////////////////////////////////////////////////////////
// FluigentHub class
// Hub for Fluigent Pressure Controller devices
///////////////////////////////////////////////////////////////////////////////

FluigentHub::FluigentHub() :
    initialized_(false),
    busy_(false),
    nDevices_(0),
    nChannels_(0),
    errorCode_(0),
    calibrate_("None")
{}

FluigentHub::~FluigentHub()
{
    if (initialized_)
        fgt_close();
}

///////////////////////////////////////////////////////////////////////////////
// FluigentHub class
// MMDevice API
///////////////////////////////////////////////////////////////////////////////

void FluigentHub::GetName(char* name) const
{
    // Return the name used to refer to this device adapter
    string deviceName = g_FluigentHubName;
    CDeviceUtils::CopyLimitedString(name, g_FluigentHubName);
}

int FluigentHub::Initialize()
{
    // Name
    int ret = CreateStringProperty(MM::g_Keyword_Name, g_FluigentHubName, true);
    if (DEVICE_OK != ret)
        return ret;

    // Description
    ret = CreateStringProperty(MM::g_Keyword_Description, "Hub for Fluigent pressure controllers", true);
    if (DEVICE_OK != ret)
        return ret;

    // Detect number of pressure controllers (MFCS, MFCS-EZ, LINE-UP)
    unsigned char nDevicesDetected;
    nDevicesDetected = fgt_detect(SNs_, instrumentTypes_);
    for (int i = 0; i < nDevicesDetected; i++)
    {
        switch (instrumentTypes_[i])
        {
        case 1: // MFCS
            nDevices_++;
            break;
        case 2: // MFCS-EZ
            nDevices_++;
            break;
        case 4: // LineUP
            nDevices_++;
            break;
        default: // Remove other devices from list
            SNs_[i] = 0;
        }
    }
    LogMessage("Number of devices detected: " + to_string(nDevicesDetected));

    // Initialize pressure controllers
    errorCode_ = fgt_initEx(SNs_);
    if (errorCode_ != 0) { return DEVICE_ERR; }

    // Set system-wide pressure unit to the unit chosen during startup (default is kPa)
    fgt_set_sessionPressureUnit((char*)"kPa");

    // Detect total number of pressure channels
    unsigned char nChannelsTemp;
    errorCode_ = fgt_get_pressureChannelCount(&nChannelsTemp);
    nChannels_ = (int)nChannelsTemp;
    LogMessage("Number of channels detected: " + to_string(nChannels_));

    // Get channel information
    fgt_get_pressureChannelsInfo(channelInfo_);

    // Create calibration property
    vector<string> allowedNames = { "All", "None" };
    for (size_t i = 0; i < nChannels_; i++) {
        if (channelInfo_[i].InstrType != fgt_INSTRUMENT_TYPE::None) {
            allowedNames.push_back(to_string(i));
        }
    }
    CPropertyAction* pAct = new CPropertyAction(this, &FluigentHub::OnCalibrate);
    ret = CreateStringProperty("Calibrate", "None", false, pAct);
    SetAllowedValues("Calibrate", allowedNames);

    initialized_ = true;
    return DEVICE_OK;
}

int FluigentHub::Shutdown()
{
    // Close the communication with Fluigent devices
    fgt_close();
    initialized_ = false;
    return DEVICE_OK;
}

int FluigentHub::DetectInstalledDevices()
{
    // Automatically add all discovered pumps
    ClearInstalledDevices();
    for (int i = 0; i < nChannels_; i++) {
        MM::Pump* pPump = new FluigentChannel(i);
        if (pPump)
            AddInstalledDevice(pPump);
    }
    return DEVICE_OK;
}

int FluigentHub::OnCalibrate(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int ret = DEVICE_ERR;
    switch (eAct)
    {
    case MM::AfterSet:
    {
        pProp->Get(calibrate_);
        if (calibrate_ == "All") {
            // Calibrate all channels, one by one
            for (int i = 0; i < nChannels_; i++) {
                fgt_calibratePressure(channelInfo_[i].indexID);
            }
        }
        else if (calibrate_ != "None") {
            // Calibrate specific channel
            fgt_calibratePressure(channelInfo_[atoi(calibrate_.c_str())].indexID);
        }
        calibrate_ = "None";
        ret = DEVICE_OK;
    }break;
    case MM::BeforeGet:
    {
        // Refresh the displayed value
        ret = DEVICE_OK;
        pProp->Set(calibrate_.c_str());
    }break;
    }
    return ret;
    return DEVICE_OK;
}

int FluigentHub::GetNChannels(int& nChannels) {
    nChannels = nChannels_;
    return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// FluigentChannel class
// Fluigent Pressure Controller Channel
///////////////////////////////////////////////////////////////////////////////

FluigentChannel::FluigentChannel(int idx) :
    initialized_(false),
    busy_(false),
    errorCode_(0)
{
    idx_ = idx;
};

///////////////////////////////////////////////////////////////////////////////
// FluigentChannel class
// MMDevice API
///////////////////////////////////////////////////////////////////////////////

void FluigentChannel::GetName(char* name) const
{
    // Return the name used to refer to this device adapter
    CDeviceUtils::CopyLimitedString(name, (g_FluigentChannelName + to_string(idx_)).c_str());
}

int FluigentChannel::Initialize()
{
    // Name
    int ret = CreateStringProperty(MM::g_Keyword_Name, (g_FluigentChannelName + to_string(idx_)).c_str(), true);
    if (DEVICE_OK != ret)
        return ret;

    // Description
    ret = CreateStringProperty(MM::g_Keyword_Description, "Fluigent Pressure Controller Channel", true);
    if (DEVICE_OK != ret)
        return ret;

    // Link with Hub
    CreateHubIDProperty();
    FluigentHub* pHub = static_cast<FluigentHub*>(GetParentHub());
    if (pHub)
    {
        char hubLabel[MM::MaxStrLength];
        pHub->GetLabel(hubLabel);
        SetParentID(hubLabel); // for backward comp.
    }
    else
        LogMessage("No Hub found!");

    // Get channelinfo for this specific channel
    fgt_CHANNEL_INFO tempInfo[256];
    fgt_get_pressureChannelsInfo(tempInfo);
    channelInfo_ = tempInfo[idx_];

    // Serial Number
    ret = CreateIntegerProperty("Serial Number", channelInfo_.indexID, true);
    if (DEVICE_OK != ret)
        return ret;

    // Imposed pressure
    GetPressureLimits(Pmin_, Pmax_);
    CPropertyAction* pAct = new CPropertyAction(this, &FluigentChannel::OnImposedPressure);
    ret = CreateFloatProperty("Imposed Pressure", 0, false, pAct);
    SetPropertyLimits("Imposed Pressure", Pmin_, Pmax_);

    // Measured pressure
    pAct = new CPropertyAction(this, &FluigentChannel::OnMeasuredPressure);
    ret = CreateFloatProperty("Measured Pressure", 0, true, pAct);
    return ret;
}

///////////////////////////////////////////////////////////////////////////////
// FluigentChannel class
// Action handlers
///////////////////////////////////////////////////////////////////////////////

int FluigentChannel::OnImposedPressure(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int ret = DEVICE_ERR;
    switch (eAct)
    {
    case MM::AfterSet:
    {
        // Sets a new pressure, and immediately updates the meausured pressure
        double Ptemp;
        pProp->Get(Ptemp);
        Pimp_ = (float)Ptemp;
        SetPressure(Pimp_);
        GetPressure(Pmeas_);
        ret = DEVICE_OK;
    }break;
    case MM::BeforeGet:
    {
        // Simply get the imposed pressure value
        pProp->Set(Pimp_);
        ret = DEVICE_OK;
    }break;
    }
    return ret;
}

int FluigentChannel::OnMeasuredPressure(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int ret = DEVICE_OK;
    switch (eAct)
    {
        // This function is read-only
    case MM::BeforeGet:
    {
        // Get the measured pressure, and refresh the value
        ret = GetPressure(Pmeas_);
        pProp->Set(Pmeas_);
    }break;
    }
    if (ret == 0)
        return DEVICE_OK;
    else
        return DEVICE_ERR;

}

///////////////////////////////////////////////////////////////////////////////
// FluigentChannel class
// MMPump API
///////////////////////////////////////////////////////////////////////////////

int FluigentChannel::GetPressure(float& P) {
    int ret = fgt_get_pressure(channelInfo_.indexID, &P);
    return ret;
}

int FluigentChannel::SetPressure(float P)
{
    int ret = fgt_get_pressure(channelInfo_.indexID, &P);
    return ret;
}

int FluigentChannel::Calibrate()
{
    fgt_calibratePressure(channelInfo_.indexID);
    int ret = DEVICE_OK;
    return ret;
}

///////////////////////////////////////////////////////////////////////////////
// FluigentChannel class
// Utility methods
///////////////////////////////////////////////////////////////////////////////

int FluigentChannel::GetPressureLimits(float& Pmin, float& Pmax)
{
    int ret = DEVICE_OK;
    fgt_get_pressureRange(channelInfo_.indexID, &Pmin, &Pmax);
    return ret;
}