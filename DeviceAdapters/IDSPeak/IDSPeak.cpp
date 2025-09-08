///////////////////////////////////////////////////////////////////////////////
// FILE:          IDSPeak.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Driver for IDS peak series of USB cameras
//
//                Based on IDS peak SDK and Micro-manager DemoCamera example
//                tested with SDK version 2.5
//                Requires Micro-manager Device API 71 or higher!
//                
// AUTHOR:        Lars Kool, Institut Pierre-Gilles de Gennes
//
// YEAR:          2023
//                
// VERSION:       1.1.1
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
//
//LAST UPDATE:    03.12.2024 LK

#include "IDSPeak.h"
#include <cstdio>
#include <string>
#include <math.h>
#include "ModuleInterface.h"
#include <sstream>
#include <algorithm>
#include <iostream>
#include <future>
#include <unordered_map>

#include <peak/peak.hpp>
#include <peak_ipl/peak_ipl.hpp>
#include <peak/converters/peak_buffer_converter_ipl.hpp>

using AccessNode = peak::core::nodes::NodeAccessStatus;
using CommandNode = peak::core::nodes::CommandNode;
using EnumNode = peak::core::nodes::EnumerationNode;
using FloatNode = peak::core::nodes::FloatNode;
using IntNode = peak::core::nodes::IntegerNode;

double g_IntensityFactor_ = 1.0;
const char* g_PixelType = "PixelType";
const char* g_PixelType_8bit = "8bit";
const char* g_PixelType_16bit = "16bit";
const char* g_PixelType_32bitRGBA = "32bit RGBA";

// External names used used by the rest of the system
// to load particular device from the "IDSPeak.dll" library
const char* g_IDSPeakCameraName = "IDSCam";
const char* g_IDSPeakHubName = "IDS Peak Hub";
const char* g_keyword_Peak_PixelFormat = "IDS Pixel Format";

template <typename T>
T boundValue(T val, T minVal, T maxVal) {
    return min(minVal, max(maxVal, val));
}

///////////////////////////////////////////////////////////////////////////////
//  MMDevice API
///////////////////////////////////////////////////////////////////////////////

MODULE_API void InitializeModuleData()
{
    RegisterDevice(g_IDSPeakHubName, MM::HubDevice, "Hub for IDS cameras");
    RegisterDevice(g_IDSPeakCameraName, MM::CameraDevice, "Device adapter for IDS peak cameras");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
    std::cout << deviceName << std::endl;
    if (!deviceName)
    {
        return 0; // Trying to create nothing, return nothing
    }
    if (strcmp(deviceName, g_IDSPeakHubName) == 0)
    {
        return new IDSPeakHub(); // Create Hub
    }
    if (strncmp(deviceName, g_IDSPeakCameraName, strlen(g_IDSPeakCameraName)) == 0)
    {
        std::string name_s = deviceName;
        std::string substr = name_s.substr(strlen(g_IDSPeakCameraName), strlen(deviceName));
        int deviceIdx = std::stoi(substr);
        return new CIDSPeak(deviceIdx); // Create channel
    }
    return 0; // If an unexpected name is provided, return nothing
}

MODULE_API void DeleteDevice(MM::Device* device)
{
    delete device;
}

//////////////////////////////////////////////////////////////////////////////
// IDSPeakHub class
// Hub for IDS Peak cameras
//////////////////////////////////////////////////////////////////////////////

IDSPeakHub::IDSPeakHub() :
    initialized_(false),
    busy_(false)
{}

IDSPeakHub::~IDSPeakHub() {
    Shutdown();
}

/**
* Obtains device name.
* Required by the MM::Device API.
*/
void IDSPeakHub::GetName(char* name) const
{
    // Return the name used to refer to this device adapter
    std::string deviceName = g_IDSPeakHubName;
    CDeviceUtils::CopyLimitedString(name, deviceName.c_str());
}

int IDSPeakHub::Initialize()
{
    if (initialized_)
        return DEVICE_OK;

    // Name
    int ret = CreateStringProperty(MM::g_Keyword_Name, g_IDSPeakHubName, true);
    if (DEVICE_OK != ret)
        return ret;

    // Descriptio
    ret = CreateStringProperty(MM::g_Keyword_Description, "Hub for IDS Peak cameras", true);
    if (DEVICE_OK != ret)
        return ret;

    peak::Library::Initialize();
    auto& deviceManager = peak::DeviceManager::Instance();
    deviceManager.Update();
    nCameras_ = (int)deviceManager.Devices().size();

    initialized_ = true;
    return DEVICE_OK;
}

/**
* Shuts down (unloads) the device.
* Required by the MM::Device API.
* Ideally this method will completely unload the device and release all resources.
* Shutdown() may be called multiple times in a row.
* After Shutdown() we should be allowed to call Initialize() again to load the device
* without causing problems.
*/
int IDSPeakHub::Shutdown()
{
    peak::Library::Close();
    initialized_ = false;
    return DEVICE_OK;
}

int IDSPeakHub::DetectInstalledDevices()
{
    ClearInstalledDevices();
    for (int i = 0; i < nCameras_; i++)
    {
        MM::Device* device = new CIDSPeak(i);
        if (device)
        {
            AddInstalledDevice(device);
        }
    }
    return DEVICE_OK;
}

bool IDSPeakHub::Busy()
{
    return false;
}

///////////////////////////////////////////////////////////////////////////////
// CIDSPeak implementation
// ~~~~~~~~~~~~~~~~~~~~~~~~~~

/**
* CIDSPeak constructor.
* Setup default all variables and create device properties required to exist
* before intialization. In this case, no such properties were required. All
* properties will be created in the Initialize() method.
*
* As a general guideline Micro-Manager devices do not access hardware in the
* the constructor. We should do as little as possible in the constructor and
* perform most of the initialization in the Initialize() method.
*/
CIDSPeak::CIDSPeak(int idx) :
    CCameraBase<CIDSPeak>(),
    initialized_(false)
{
    // call the base class method to set-up default error codes/messages
    InitializeDefaultErrorMessages();
    readoutStartTime_ = GetCurrentMMTime();
    thd_ = new MySequenceThread(this);
    deviceIdx_ = idx;

    // parent ID display
    CreateHubIDProperty();
}

/**
* CIDSPeak destructor.
* If this device used as intended within the Micro-Manager system,
* Shutdown() will be always called before the destructor. But in any case
* we need to make sure that all resources are properly released even if
* Shutdown() was not called.
*/
CIDSPeak::~CIDSPeak()
{
    Shutdown();
    StopSequenceAcquisition();
    delete thd_;
}

/**
* Obtains device name.
* Required by the MM::Device API.
*/
void CIDSPeak::GetName(char* name) const
{
    // Return the name used to referr to this device adapte
    std::string deviceName = g_IDSPeakCameraName;
    deviceName.append(std::to_string(deviceIdx_));
    CDeviceUtils::CopyLimitedString(name, deviceName.c_str());
}

/**
* Intializes the hardware.
* Required by the MM::Device API.
* Typically we access and initialize hardware at this point.
* Device properties are typically created here as well, except
* the ones we need to use for defining initialization parameters.
* Such pre-initialization properties are created in the constructor.
* (This device does not have any pre-initialization properties)
*/
int CIDSPeak::Initialize()
{
    if (initialized_)
        return DEVICE_OK;

    // Get handle to Hub
    IDSPeakHub* pHub = static_cast<IDSPeakHub*>(GetParentHub());
    if (pHub) {
        char hubLabel[MM::MaxStrLength];
        pHub->GetLabel(hubLabel);
        SetParentID(hubLabel);
    }
    else {
        LogMessage("No hub was found.");
    }

    // Open camera
    try {
        auto& deviceManager = peak::DeviceManager::Instance(); // Get device manager
        descriptor = deviceManager.Devices().at(deviceIdx_); // set device
        device = descriptor->OpenDevice(peak::core::DeviceAccessType::Control); // Open camera
        nodeMapRemoteDevice = device->RemoteDevice()->NodeMaps().at(0);
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not open camera.");
        LogMessage(e.what());
        return DEVICE_ERR;
    }

    // set property list
    // -----------------

    // Name
    int nRet = CreateStringProperty(MM::g_Keyword_Name, g_IDSPeakCameraName, true);
    if (nRet != DEVICE_OK) { return nRet; }

    // Description
    nRet = CreateStringProperty(MM::g_Keyword_Description, "IDS Peak Camera Adapter", true);
    if (nRet != DEVICE_OK) { return nRet; }

    // CameraName
    try {
        modelName_ = descriptor->ModelName();
        serialNumber_ = descriptor->SerialNumber();
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not get model name or serial number.");
        LogMessage(e.what());
        return DEVICE_ERR;
    }
    nRet = CreateStringProperty(MM::g_Keyword_CameraName, modelName_.c_str(), true);
    if (nRet != DEVICE_OK) { return nRet; }

    // SerialNumber
    nRet = CreateStringProperty("Serial Number", descriptor->SerialNumber().c_str(), true);
    if (nRet != DEVICE_OK) { return nRet; }

    // Exposure time, divide by 1000 to convert us to ms.
    try {
        auto node = nodeMapRemoteDevice->FindNode<FloatNode>("ExposureTime");
        exposureCur_ = node->Value() / 1000;
        exposureMin_ = node->Minimum() / 1000;
        exposureMax_ = node->Maximum() / 1000;
        exposureInc_ = node->Increment() / 1000;
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not read exposure time.");
        LogMessage(e.what());
        return DEVICE_ERR;
    }
    nRet = CreateFloatProperty(MM::g_Keyword_Exposure, exposureCur_, false);
    if (nRet != DEVICE_OK) { return nRet; }
    nRet = SetPropertyLimits(MM::g_Keyword_Exposure, exposureMin_, exposureMax_);
    if (nRet != DEVICE_OK) { return nRet; }

    // Frame rate
    try {
        auto node = nodeMapRemoteDevice->FindNode<FloatNode>("AcquisitionFrameRate");
        frameRateCur_ = node->Value();
        frameRateMin_ = node->Minimum();
        frameRateMax_ = node->Maximum();
        if (node->HasConstantIncrement()) {
            frameRateInc_ = node->Increment();
        }
        else {
            frameRateInc_ = 0.001;
        }
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not read acquisition framerate");
        LogMessage(e.what());
        return DEVICE_ERR;
    }
    CPropertyAction* pAct = new CPropertyAction(this, &CIDSPeak::OnFrameRate);
    nRet = CreateFloatProperty("MDA framerate", 1, false, pAct);
    if (!nRet == DEVICE_OK) { return nRet; }
    nRet = SetPropertyLimits("MDA framerate", frameRateMin_, frameRateMax_);
    if (!nRet == DEVICE_OK) { return nRet; }

    // Get current acquisition Mode
    acquisitionMode_ = nodeMapRemoteDevice->FindNode<EnumNode>("AcquisitionMode")
        ->CurrentEntry()->SymbolicValue();

    // Auto white balance
    if (enableAutoWhitebalance_) {
        std::vector<std::string> whitebalanceValues;
        try {
            auto node = nodeMapRemoteDevice->FindNode<EnumNode>("GainAuto");
            auto entries = node->Entries();
            for (const auto& entry : entries) {
                if (entry->AccessStatus() == peak::core::nodes::NodeAccessStatus::ReadWrite) {
                    whitebalanceValues.push_back(entry->SymbolicValue());
                }
            }
            whitebalanceCurr_ = node->CurrentEntry()->SymbolicValue();
        }
        catch (std::exception& e) {
            LogMessage("IDS exception: error occurred during auto whitebalance request.");
            LogMessage(e.what());
            return DEVICE_ERR;
        }

        pAct = new CPropertyAction(this, &CIDSPeak::OnAutoWhitebalance);
        nRet = CreateStringProperty("Auto white balance", whitebalanceCurr_.c_str(), false, pAct);
        if (nRet != DEVICE_OK) { return nRet; }
        nRet = SetAllowedValues("Auto white balance", whitebalanceValues);
        if (nRet != DEVICE_OK) { return nRet; }
    }

    // Analog gain
    if (enableAnalogGain_) {
        nRet = CreateGain("AnalogAll", gainAnalogMaster_, &CIDSPeak::OnAnalogMaster);
        if (nRet != DEVICE_OK) { return nRet; }
        nRet = CreateGain("AnalogRed", gainAnalogRed_, &CIDSPeak::OnAnalogRed);
        if (nRet != DEVICE_OK) { return nRet; }
        nRet = CreateGain("AnalogGreen", gainAnalogGreen_, &CIDSPeak::OnAnalogGreen);
        if (nRet != DEVICE_OK) { return nRet; }
        nRet = CreateGain("AnalogBlue", gainAnalogBlue_, &CIDSPeak::OnAnalogBlue);
        if (nRet != DEVICE_OK) { return nRet; }
    }

    // Digital gain
    if (enableDigitalGain_) {
        nRet = CreateGain("DigitalAll", gainDigitalMaster_, &CIDSPeak::OnDigitalMaster);
        if (nRet != DEVICE_OK) { return nRet; }
        nRet = CreateGain("DigitalRed", gainDigitalRed_, &CIDSPeak::OnDigitalRed);
        if (nRet != DEVICE_OK) { return nRet; }
        nRet = CreateGain("DigitalGreen", gainDigitalGreen_, &CIDSPeak::OnDigitalGreen);
        if (nRet != DEVICE_OK) { return nRet; }
        nRet = CreateGain("DigitalBlue", gainDigitalBlue_, &CIDSPeak::OnDigitalBlue);
        if (nRet != DEVICE_OK) { return nRet; }
    }

    // camera temperature ReadOnly, and request camera temperature
    if (enableTemperature_) {
        try {
            auto node = nodeMapRemoteDevice->FindNode<FloatNode>("DeviceTemperature");
            if (node->AccessStatus() != peak::core::nodes::NodeAccessStatus::ReadOnly ||
                node->AccessStatus() != peak::core::nodes::NodeAccessStatus::ReadWrite) {
                LogMessage("IDS warning: Temperature not available");
                return DEVICE_OK;
            }
            sensorTemp_ = node->Value();
        }
        catch (std::exception& e) {
            LogMessage("IDS exception: An error occurred while reading the temperature");
            LogMessage(e.what());
            return DEVICE_ERR;
        }
        pAct = new CPropertyAction(this, &CIDSPeak::OnSensorTemp);
        nRet = CreateFloatProperty("Sensor temperature", sensorTemp_, true, pAct);
        if (nRet != DEVICE_OK) { return nRet; }
    }

    // Sensor height
    try {
        sensorHeight_ = (long)nodeMapRemoteDevice->FindNode<IntNode>("SensorHeight")->Value();
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not get sensor height.");
        LogMessage(e.what());
        return DEVICE_ERR;
    }
    nRet = CreateIntegerProperty("Sensor height", sensorHeight_, true);
    if (nRet != DEVICE_OK) { return nRet; }

    // Sensor width
    try {
        sensorWidth_ = (long)nodeMapRemoteDevice->FindNode<IntNode>("SensorWidth")->Value();
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not get sensor width.");
        LogMessage(e.what());
        return DEVICE_ERR;
    }
    nRet = CreateIntegerProperty("Sensor width", sensorWidth_, true);
    if (nRet != DEVICE_OK) { return nRet; }

    // Binning
    std::vector<std::string> binningValues;
    try {
        std::string binningSelector = "Sensor";
        if (!nodeMapRemoteDevice->FindNode<EnumNode>("BinningSelector")->HasEntry(binningSelector)) {
            binningSelector = "Region0";
        }
        nodeMapRemoteDevice->FindNode<EnumNode>("BinningSelector")
            ->SetCurrentEntry(binningSelector);
        
        int64_t maxVal = nodeMapRemoteDevice->FindNode<IntNode>("BinningHorizontal")->Maximum();
        int64_t i = 1;
        while (i <= maxVal) {
            binningValues.push_back(std::to_string(i));
            i *= 2;
        }
        nodeMapRemoteDevice->FindNode<IntNode>("BinningVertical")->SetValue(1);
        nodeMapRemoteDevice->FindNode<IntNode>("BinningHorizontal")->SetValue(1);
        binSize_ = 1;
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: An error occurred while getting available binning options.");
        LogMessage(e.what());
        return DEVICE_ERR;
    }

	pAct = new CPropertyAction(this, &CIDSPeak::OnBinning);
	nRet = CreateIntegerProperty(MM::g_Keyword_Binning, 1, false, pAct);
	if (nRet != DEVICE_OK) { return nRet; }
	SetAllowedValues(MM::g_Keyword_Binning, binningValues);
    
    // IDS PixelFormat
    std::vector<std::string> availablePixelFormats;
    try {
        auto node = nodeMapRemoteDevice->FindNode<EnumNode>("PixelFormat");
        for (const auto& entry : node->Entries()) {
            if ((peak::core::nodes::NodeAccessStatus::NotAvailable == entry->AccessStatus()) ||
                (peak::core::nodes::NodeAccessStatus::NotImplemented == entry->AccessStatus()))
            {
                // Format not available (at the moment)
                continue;
            }
            availablePixelFormats.push_back(entry->SymbolicValue());
        }
        pixelFormat_ = node->CurrentEntry()->SymbolicValue();
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: An error occurred while reading all available PixelFormats");
        LogMessage(e.what());
        return DEVICE_ERR;
    }
    pAct = new CPropertyAction(this, &CIDSPeak::OnPixelFormat);
    nRet = CreateStringProperty("IDS PixelFormat", pixelFormat_.c_str(), false, pAct);
    if (nRet != DEVICE_OK) { return nRet; }
    SetAllowedValues("IDS PixelFormat", availablePixelFormats);

    // MM PixelType
    std::vector<std::string> availablePixelTypes = { g_PixelType_8bit, g_PixelType_16bit, g_PixelType_32bitRGBA };
    pAct = new CPropertyAction(this, &CIDSPeak::OnPixelType);
    nRet = CreateStringProperty(g_PixelType, g_PixelType_8bit, false, pAct);
    if (nRet != DEVICE_OK) { return nRet; }
    SetAllowedValues(g_PixelType, availablePixelTypes);

    // ROI properties
    try {
        roiOffsetXMin_ = (unsigned int)nodeMapRemoteDevice->FindNode<IntNode>("OffsetX")->Minimum();
        roiOffsetYMin_ = (unsigned int)nodeMapRemoteDevice->FindNode<IntNode>("OffsetY")->Minimum();
        roiWidthMin_ = (unsigned int)nodeMapRemoteDevice->FindNode<IntNode>("Width")->Minimum();
        roiHeightMin_ = (unsigned int)nodeMapRemoteDevice->FindNode<IntNode>("Height")->Minimum();

        // Set the minimum ROI. This removes any size restrictions due to previous ROI settings
        nodeMapRemoteDevice->FindNode<IntNode>("OffsetX")->SetValue(roiOffsetXMin_);
        nodeMapRemoteDevice->FindNode<IntNode>("OffsetY")->SetValue(roiOffsetYMin_);
        nodeMapRemoteDevice->FindNode<IntNode>("Width")->SetValue(roiWidthMin_);
        nodeMapRemoteDevice->FindNode<IntNode>("Height")->SetValue(roiHeightMin_);

        roiOffsetXMax_ = (unsigned int)nodeMapRemoteDevice->FindNode<IntNode>("OffsetX")->Maximum();
        roiOffsetYMax_ = (unsigned int)nodeMapRemoteDevice->FindNode<IntNode>("OffsetY")->Maximum();
        roiWidthMax_ = (unsigned int)nodeMapRemoteDevice->FindNode<IntNode>("Width")->Maximum();
        roiHeightMax_ = (unsigned int)nodeMapRemoteDevice->FindNode<IntNode>("Height")->Maximum();

        // Maximize ROI
        nodeMapRemoteDevice->FindNode<IntNode>("Width")->SetValue(roiWidthMax_);
        nodeMapRemoteDevice->FindNode<IntNode>("Height")->SetValue(roiHeightMax_);
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: An error occurred while getting the minimum and maximum ROI");
        LogMessage(e.what());
        return DEVICE_ERR;
    }
    img_ = ImgBuffer(roiWidthMax_, roiHeightMax_, 1);

    // Prepare camera buffer.
    // This should be initialized after all variables that influence buffer size
    // i.e. ROI, Binning and PixelFormat
    try {
        auto dataStreams = device->DataStreams();
        if (dataStreams.empty()) {
            LogMessage("IDS error: Could not find any data streams.");
            return DEVICE_ERR;
        }
        dataStream = device->DataStreams().at(0)->OpenDataStream();
        nodeMapDataStream = dataStream->NodeMaps().at(0);
        // Set number of buffers (10, or more if required)
        // This is slightly higher than the minimum (typically 3),
        // which might smooth data transfer
        nBuffers_ = (int)max(10, dataStream->NumBuffersAnnouncedMinRequired());
        PrepareBuffer();
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not prepare camera buffer");
        LogMessage(e.what());
        return DEVICE_ERR;
    }

    if (nRet != DEVICE_OK) { return nRet; }

    // synchronize all properties
    // --------------------------
    nRet = UpdateStatus();
    if (nRet != DEVICE_OK)
        return nRet;

    initialized_ = true;
    return DEVICE_OK;
}

/**
* Shuts down (unloads) the device.
* Required by the MM::Device API.
* Ideally this method will completely unload the device and release all resources.
* Shutdown() may be called multiple times in a row.
* After Shutdown() we should be allowed to call Initialize() again to load the device
* without causing problems.
*/
int CIDSPeak::Shutdown()
{
    if (initialized_ == false) { return DEVICE_OK; }
    peak::Library::Close();
    initialized_ = false;
    return DEVICE_OK;
}

/**
* Performs exposure and grabs a single image.
* This function should block during the actual exposure and return immediately afterwards
* (i.e., before readout).  This behavior is needed for proper synchronization with the shutter.
* Required by the MM::Camera API.
*/
int CIDSPeak::SnapImage()
{
    int nRet = DEVICE_OK;
    static int callCounter = 0;
    ++callCounter;

    MM::MMTime startTime = GetCurrentMMTime();

    // Make SnapImage responsive, even if low framerate has been set
    double frameRateTemp = frameRateCur_;
    nRet = setFrameRate(frameRateMax_);
    if (nRet != DEVICE_OK) { return nRet; }

    LogMessage("Set framerate");

    uint64_t timeout_ms = (uint64_t)(3000 / frameRateCur_);

    // Start acquisition of single image
    nRet = StartAcquisition(1);
    if (nRet != DEVICE_OK) { return nRet; }

    LogMessage("Started acquisition.");

    // Take single image and transfer to MM buffer
    nRet = AcquireAndTransferImage(timeout_ms, false);
    StopAcquisition();
    if (nRet != DEVICE_OK) {
        LogMessage("IDS warning: Could not snap image. Acquisition stopped automatically.");
    }
    else {
        LogMessage("Transfered image");
    }

    readoutStartTime_ = GetCurrentMMTime();
    // Revert framerate back to framerate before snapshot
    setFrameRate(frameRateTemp);
    return DEVICE_OK;
}


/**
* Returns pixel data.
* Required by the MM::Camera API.
* The calling program will assume the size of the buffer based on the values
* obtained from GetImageBufferSize(), which in turn should be consistent with
* values returned by GetImageWidth(), GetImageHight() and GetImageBytesPerPixel().
* The calling program also assumes that camera never changes the size of
* the pixel buffer on its own. In other words, the buffer can change only if
* appropriate properties are set (such as binning, pixel type, etc.)
*/
const unsigned char* CIDSPeak::GetImageBuffer()
{
    MMThreadGuard g(imgPixelsLock_);
    unsigned char* pB = (unsigned char*)(img_.GetPixels());
    return pB;
}

/**
* Returns image buffer X-size in pixels.
* Required by the MM::Camera API.
*/
unsigned CIDSPeak::GetImageWidth() const
{
    return img_.Width();
}

/**
* Returns image buffer Y-size in pixels.
* Required by the MM::Camera API.
*/
unsigned CIDSPeak::GetImageHeight() const
{
    return img_.Height();
}

/**
* Returns image buffer pixel depth in bytes.
* Required by the MM::Camera API.
*/
unsigned CIDSPeak::GetImageBytesPerPixel() const
{
    return img_.Depth();
}

/**
* Returns the bit depth (dynamic range) of the pixel.
* This does not affect the buffer size, it just gives the client application
* a guideline on how to interpret pixel values.
* Required by the MM::Camera API.
*/
unsigned CIDSPeak::GetBitDepth() const
{
    return 8 * img_.Depth();
}

/**
* Returns the size in bytes of the image buffer.
* Required by the MM::Camera API.
*/
long CIDSPeak::GetImageBufferSize() const
{
    return img_.Width() * img_.Height() * GetImageBytesPerPixel();
}

/**
* Sets the camera Region Of Interest.
* Required by the MM::Camera API.
* This command will change the dimensions of the image.
* Depending on the hardware capabilities the camera may not be able to configure the
* exact dimensions requested - but should try do as close as possible.
* If both xSize and ySize are set to 0, the ROI is set to the entire CCD
* @param x - top-left corner coordinate
* @param y - top-left corner coordinate
* @param xSize - width
* @param ySize - height
*/
int CIDSPeak::SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize)
{
    // if xSize and/or ySize == 0, set them to max
    xSize = (xSize == 0) ? roiWidthMax_ : xSize;
    ySize = (ySize == 0) ? roiHeightMax_ : ySize;

    // Make sure values are bound by the min/max values
    x = boundValue(x, roiOffsetXMin_, roiOffsetXMax_);
    y = boundValue(y, roiOffsetYMin_, roiOffsetYMax_);
    xSize = boundValue(xSize, roiWidthMin_, roiWidthMax_ - x);
    ySize = boundValue(ySize, roiHeightMin_, roiHeightMax_ - y);

    try {
        if (nodeMapRemoteDevice->FindNode<IntNode>("OffsetX")->AccessStatus() !=
            peak::core::nodes::NodeAccessStatus::ReadWrite) {
            return DEVICE_CAN_NOT_SET_PROPERTY;
        }
        nodeMapRemoteDevice->FindNode<IntNode>("OffsetX")->SetValue(x);
        nodeMapRemoteDevice->FindNode<IntNode>("OffsetY")->SetValue(y);
        nodeMapRemoteDevice->FindNode<IntNode>("Width")->SetValue(xSize);
        nodeMapRemoteDevice->FindNode<IntNode>("Height")->SetValue(ySize);
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: An error occurred while setting the ROI.");
        LogMessage(e.what());
        return DEVICE_CAN_NOT_SET_PROPERTY;
    }

    roiX_ = x;
    roiY_ = y;
    return DEVICE_OK;
}

/**
* Returns the actual dimensions of the current ROI.
* If multiple ROIs are set, then the returned ROI should encompass all of them.
* Required by the MM::Camera API.
*/
int CIDSPeak::GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize)
{
    x = roiX_;
    y = roiY_;

    xSize = img_.Width();
    ySize = img_.Height();

    return DEVICE_OK;
}

/**
* Resets the Region of Interest to full frame.
* Required by the MM::Camera API.
*/
int CIDSPeak::ClearROI()
{
    // Passing all zeros to SetROI sets the ROI to the full frame
    int nRet = SetROI(0, 0, 0, 0);
    return nRet;
}

/**
* Returns the current binning factor.
* Required by the MM::Camera API.
*/
int CIDSPeak::GetBinning() const
{
    char buf[MM::MaxStrLength];
    int nRet = GetProperty(MM::g_Keyword_Binning, buf);
    if (nRet != DEVICE_OK) { return 0; } // If something goes wrong, return 0 (unphysical binning)
    return atoi(buf);
}

/**
* Sets binning factor.
* Required by the MM::Camera API.
*/
int CIDSPeak::SetBinning(int binF)
{
    try {
        nodeMapRemoteDevice->FindNode<EnumNode>("BinningSelector")
            ->SetCurrentEntry("Region0");
        nodeMapRemoteDevice->FindNode<IntNode>("BinningHorizontal")
            ->SetValue(binF);
        nodeMapRemoteDevice->FindNode<IntNode>("BinningVertical")
            ->SetValue(binF);
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: An error occurred while setting binning.");
        LogMessage(e.what());
        return DEVICE_CAN_NOT_SET_PROPERTY;
    }

    binSize_ = binF;
    int nRet = SetProperty(MM::g_Keyword_Binning, CDeviceUtils::ConvertToString(binF));
    return nRet;
}

/**
* Returns the current exposure setting in milliseconds.
* Required by the MM::Camera API.
*/
double CIDSPeak::GetExposure() const
{
    char buf[MM::MaxStrLength];
    int nRet = GetProperty(MM::g_Keyword_Exposure, buf);
    if (nRet != DEVICE_OK) { return 0.; } // If something goes wrong, return 0. 
    return atof(buf);
}

/**
 * Returns the current exposure from a sequence and increases the sequence counter
 * Used for exposure sequences
 */
double CIDSPeak::GetSequenceExposure()
{
    if (exposureSequence_.size() == 0)
        return this->GetExposure();

    double exposure = exposureSequence_[sequenceIndex_];

    sequenceIndex_++;
    if (sequenceIndex_ >= exposureSequence_.size())
        sequenceIndex_ = 0;

    return exposure;
}

/**
* Sets exposure in milliseconds.
* Required by the MM::Camera API.
*/
void CIDSPeak::SetExposure(double exp)
{
    // Check for access
    auto node = nodeMapRemoteDevice->FindNode<FloatNode>("ExposureTime");
    if (node->AccessStatus() != peak::core::nodes::NodeAccessStatus::ReadWrite) {
        LogMessage("IDS error: Could not set exposure time.");
        return;
    }

	// Check if exposure time is less than the minimun exposure time
	// If so, set it to minimum exposure time.
	if (exp <= exposureMin_) {
	    LogMessage("IDS warning: Exposure time too short. Exposure time set to minimum.");
	}
	// Check if exposure time is less than the maximum exposure time
	// If so, set it to maximum exposure time.
	else if (exp >= exposureMax_) {
		LogMessage("IDS warning: Exposure time too long. Exposure time set to maximum.");
	}
	
    // Convert milliseconds to microseconds (peak cameras expect time in microseconds)
    // and make exposure set multiple of increment.
    double exposureSet = ceil(exp / exposureInc_) * exposureInc_ * 1000;
    
    // Set new exposure time and get actual value after set
    try {
        node->SetValue(exposureSet);
        exposureCur_ = node->Value() / 1000;
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not set exposure time.");
        LogMessage(e.what());
        return;
    }

	// Update framerate range
    try {
        auto nodeFrameRate = nodeMapRemoteDevice->FindNode<FloatNode>("AcquisitionFrameRate");
        frameRateMin_ = nodeFrameRate->Minimum();
        frameRateMax_ = nodeFrameRate->Maximum();
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not read acquisition framerate");
        LogMessage(e.what());
    }
	SetPropertyLimits("MDA framerate", frameRateMin_, frameRateMax_);

    SetProperty(MM::g_Keyword_Exposure, CDeviceUtils::ConvertToString(exposureCur_));
	GetCoreCallback()->OnExposureChanged(this, exp);
}



int CIDSPeak::IsExposureSequenceable(bool& isSequenceable) const
{
    isSequenceable = isSequenceable_;
    return DEVICE_OK;
}

int CIDSPeak::GetExposureSequenceMaxLength(long& nrEvents) const
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    nrEvents = sequenceMaxLength_;
    return DEVICE_OK;
}

int CIDSPeak::StartExposureSequence()
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    // may need thread lock
    sequenceRunning_ = true;
    return DEVICE_OK;
}

int CIDSPeak::StopExposureSequence()
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    // may need thread lock
    sequenceRunning_ = false;
    sequenceIndex_ = 0;
    return DEVICE_OK;
}

/**
 * Clears the list of exposures used in sequences
 */
int CIDSPeak::ClearExposureSequence()
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    exposureSequence_.clear();
    return DEVICE_OK;
}

/**
 * Adds an exposure to a list of exposures used in sequences
 */
int CIDSPeak::AddToExposureSequence(double exposureTime_ms)
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    exposureSequence_.push_back(exposureTime_ms);
    return DEVICE_OK;
}

int CIDSPeak::SendExposureSequence() const
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    return DEVICE_OK;
}

/**
 * Required by the MM::Camera API
 * Please implement this yourself and do not rely on the base class implementation
 * The Base class implementation is deprecated and will be removed shortly
 */
int CIDSPeak::StartSequenceAcquisition(double interval)
{
    return StartSequenceAcquisition(LONG_MAX, interval, false);
}


/**
* Simple implementation of Sequence Acquisition
* A sequence acquisition should run on its own thread and transport new images
* coming of the camera into the MMCore circular buffer.
*/
int CIDSPeak::StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow)
{
    if (IsCapturing()) { return DEVICE_CAMERA_BUSY_ACQUIRING; }
    int nRet = DEVICE_OK;

    // Adjust framerate to match requested interval between frames
    nRet = setFrameRate(1000 / interval_ms);
    if (nRet != DEVICE_OK) { return nRet; }

    // Set frame count
    nRet = setFrameCount(numImages);
    if (nRet != DEVICE_OK) { return nRet; }

    // Wait until shutter is ready
    nRet = GetCoreCallback()->PrepareForAcq(this);
    if (nRet != DEVICE_OK) { return nRet; }

    // Start sequence
    sequenceStartTime_ = GetCurrentMMTime();
    imageCounter_ = 0;
    thd_->Start(numImages, interval_ms);
    stopOnOverflow_ = stopOnOverflow;
    return DEVICE_OK;
}

/**
* Stop and wait for the Sequence thread finished
*/
int CIDSPeak::StopSequenceAcquisition()
{
    if (!thd_->IsStopped()) {
        thd_->Stop();
        thd_->wait();
    }
    return DEVICE_OK;
}

/*
 * Inserts Image and MetaData into MMCore circular Buffer
 */
int CIDSPeak::InsertImage()
{
    MM::MMTime timeStamp = this->GetCurrentMMTime();
    char label[MM::MaxStrLength];
    this->GetLabel(label);

    // Important:  metadata about the image are generated here:
    Metadata md;
    md.put(MM::g_Keyword_Metadata_CameraLabel, label);
    md.put(MM::g_Keyword_Elapsed_Time_ms, CDeviceUtils::ConvertToString((timeStamp - sequenceStartTime_).getMsec()));
    md.put(MM::g_Keyword_Metadata_ROI_X, CDeviceUtils::ConvertToString((long)roiX_));
    md.put(MM::g_Keyword_Metadata_ROI_Y, CDeviceUtils::ConvertToString((long)roiY_));

    char buf[MM::MaxStrLength];
    GetProperty(MM::g_Keyword_Binning, buf);
    md.put(MM::g_Keyword_Binning, buf);

    imageCounter_++;

    MMThreadGuard g(imgPixelsLock_);
    int nRet = GetCoreCallback()->InsertImage(this,
		GetImageBuffer(),
		GetImageWidth(),
		GetImageHeight(),
		GetImageBytesPerPixel(),
        md.Serialize().c_str());

    if (!stopOnOverflow_ && nRet == DEVICE_BUFFER_OVERFLOW)
    {
        // do not stop on overflow - just reset the buffer
        GetCoreCallback()->ClearImageBuffer(this);
        return GetCoreCallback()->InsertImage(this,
            GetImageBuffer(),
            GetImageWidth(),
            GetImageHeight(),
            GetImageBytesPerPixel(),
            md.Serialize().c_str());
    }
    else { return nRet; }
}

/*
 * Do actual capturing
 * Called from inside the thread
 */
int CIDSPeak::RunSequenceOnThread()
{
    MM::MMTime startTime = GetCurrentMMTime();
    uint64_t timeout_ms = (uint64_t)(3000 / frameRateCur_);
    return AcquireAndTransferImage(timeout_ms, true);;
};

bool CIDSPeak::IsCapturing() {
    return !thd_->IsStopped();
}

/*
 * called from the thread function before exit
 */
void CIDSPeak::OnThreadExiting() throw()
{
    try
    {
        LogMessage(g_Msg_SEQUENCE_ACQUISITION_THREAD_EXITING);
        GetCoreCallback() ? GetCoreCallback()->AcqFinished(this, 0) : DEVICE_OK;
    }
    catch (std::exception& e)
    {
        LogMessage(e.what());
        LogMessage(g_Msg_EXCEPTION_IN_ON_THREAD_EXITING, false);
    }
}

int CIDSPeak::StartAcquisition(long numImages) {
    try {
        if (numImages == LONG_MAX) {
            dataStream->StartAcquisition(peak::core::AcquisitionStartMode::Default, PEAK_INFINITE_NUMBER);
        }
        else {
            dataStream->StartAcquisition(peak::core::AcquisitionStartMode::Default, numImages);
        }
        nodeMapRemoteDevice->FindNode<IntNode>("TLParamsLocked")->SetValue(1); // Lock acq params
        nodeMapRemoteDevice->FindNode<CommandNode>("AcquisitionStart")->Execute();
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not start acquisition");
        LogMessage(e.what());
        return DEVICE_ERR;
    }
    return DEVICE_OK;
}

int CIDSPeak::StopAcquisition() {
    try {
        auto node = nodeMapRemoteDevice->FindNode<CommandNode>("AcquisitionStop");
        node->Execute();
        node->WaitUntilDone();
        nodeMapRemoteDevice->FindNode<IntNode>("TLParamsLocked")->SetValue(0); // Unlock acq params
        dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not stop acquisition.");
        LogMessage(e.what());
        return DEVICE_ERR;
    }
    return DEVICE_OK;
}

MySequenceThread::MySequenceThread(CIDSPeak* pCam)
    :intervalMs_(default_intervalMS)
    , numImages_(default_numImages)
    , imageCounter_(0)
    , stop_(true)
    , suspend_(false)
    , camera_(pCam)
    , startTime_(0)
    , actualDuration_(0)
    , lastFrameTime_(0)
{};

MySequenceThread::~MySequenceThread() {};

void MySequenceThread::Stop() {
    MMThreadGuard g(this->stopLock_);
    stop_ = true;
}

void MySequenceThread::Start(long numImages, double intervalMs)
{
    MMThreadGuard g1(this->stopLock_);
    MMThreadGuard g2(this->suspendLock_);
    numImages_ = numImages;
    intervalMs_ = intervalMs;
    imageCounter_ = 0;
    stop_ = false;
    suspend_ = false;
    activate();
    actualDuration_ = MM::MMTime{};
    startTime_ = camera_->GetCurrentMMTime();
    lastFrameTime_ = MM::MMTime{};
}

//void MySequenceThread::SetIntervalMs(double intervalms)
//{
//    intervalMs_ = intervalms;
//}

bool MySequenceThread::IsStopped() {
    MMThreadGuard g(this->stopLock_);
    return stop_;
}

void MySequenceThread::Suspend() {
    MMThreadGuard g(this->suspendLock_);
    suspend_ = true;
}

bool MySequenceThread::IsSuspended() {
    MMThreadGuard g(this->suspendLock_);
    return suspend_;
}

void MySequenceThread::Resume() {
    MMThreadGuard g(this->suspendLock_);
    suspend_ = false;
}

int MySequenceThread::svc(void) throw()
{
    int nRet = DEVICE_ERR;
    try {
        // Start acquisition
        nRet = camera_->StartAcquisition(numImages_);
        if (nRet != DEVICE_OK) { return nRet; }

        // do-while loop over numImages_
        do {
            nRet = camera_->RunSequenceOnThread();
        } while (nRet == DEVICE_OK && !IsStopped() && imageCounter_++ < numImages_ - 1);

        // If the acquisition is stopped manually, the acquisition has to be properly closed to
        // prevent the camera to be locked in acquisition mode.
        if (IsStopped()) {
            camera_->StopAcquisition();
            camera_->LogMessage("SeqAcquisition interrupted by the user\n");
        }
    }
    catch (std::exception& e) {
        camera_->LogMessage("IDS exception: Something went wrong during image acquisition");
        camera_->LogMessage(e.what());
        camera_->LogMessage(g_Msg_EXCEPTION_IN_THREAD, false);
    }
    stop_ = true;
    actualDuration_ = camera_->GetCurrentMMTime() - startTime_;
    camera_->OnThreadExiting();
    return nRet;
}


///////////////////////////////////////////////////////////////////////////////
// CIDSPeak Action handlers
///////////////////////////////////////////////////////////////////////////////

int CIDSPeak::OnEnableTemperature(MM::PropertyBase* pProp, MM::ActionType eAct) {
    if (eAct == MM::BeforeGet) {
        pProp->Set((enableTemperature_) ? "true" : "false");
    }
    else if (eAct == MM::AfterSet) {
        std::string temp;
        pProp->Get(temp);
        enableTemperature_ = (temp == "true");
    }
    return DEVICE_OK;
}

int CIDSPeak::OnEnableAnalogGain(MM::PropertyBase* pProp, MM::ActionType eAct) {
    if (eAct == MM::BeforeGet) {
        pProp->Set((enableAnalogGain_) ? "true" : "false");
    }
    else if (eAct == MM::AfterSet) {
        std::string temp;
        pProp->Get(temp);
        enableAnalogGain_ = (temp == "true");
    }
    return DEVICE_OK;
}

int CIDSPeak::OnEnableDigitalGain(MM::PropertyBase* pProp, MM::ActionType eAct) {
    if (eAct == MM::BeforeGet) {
        pProp->Set((enableDigitalGain_) ? "true" : "false");
    }
    else if (eAct == MM::AfterSet) {
        std::string temp;
        pProp->Get(temp);
        enableDigitalGain_ = (temp == "true");
    }
    return DEVICE_OK;
}

int CIDSPeak::OnEnableAutoWhitebalance(MM::PropertyBase* pProp, MM::ActionType eAct) {
    if (eAct == MM::BeforeGet) {
        pProp->Set((enableAutoWhitebalance_) ? "true" : "false");
    }
    else if (eAct == MM::AfterSet) {
        std::string temp;
        pProp->Get(temp);
        enableAutoWhitebalance_ = (temp == "true");
    }
    return DEVICE_OK;
}

/**
* Handles "Binning" property.
*/
int CIDSPeak::OnBinning(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int nRet = DEVICE_ERR;
    switch (eAct)
    {
    case MM::AfterSet:
    {
        // Should not change binning during acquisition
        if (IsCapturing()) { return DEVICE_CAMERA_BUSY_ACQUIRING; }

        long binFactor;
        pProp->Get(binFactor);

		// calculate ROI using the previous bin settings
		double factor = (double)binFactor / (double)binSize_;
		roiX_ = (unsigned int)(roiX_ / factor);
		roiY_ = (unsigned int)(roiY_ / factor);
		img_.Resize(
			(unsigned int)(img_.Width() / factor),
			(unsigned int)(img_.Height() / factor)
		);
		binSize_ = binFactor;
		OnPropertyChanged("Binning", std::to_string(binSize_).c_str());
		nRet = DEVICE_OK;
    }break;
    case MM::BeforeGet:
    {
        nRet = DEVICE_OK;
        pProp->Set(std::to_string(binSize_).c_str());
    }break;
    default:
        break;
    }
    return nRet;
}

int CIDSPeak::OnFrameRate(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(frameRateCur_);
    }
    else if (eAct == MM::AfterSet)
    {
        double framerateTemp;
        pProp->Get(framerateTemp);
        setFrameRate(framerateTemp);
    }
    return DEVICE_OK;
}

/**
* Handles "Auto whitebalance" property.
*/
int CIDSPeak::OnAutoWhitebalance(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int nRet = DEVICE_OK;
    if (eAct == MM::BeforeGet)
    {
        auto node = nodeMapRemoteDevice->FindNode<EnumNode>("BalanceWhiteAuto");
        if (node->AccessStatus() != peak::core::nodes::NodeAccessStatus::ReadOnly &&
            node->AccessStatus() != peak::core::nodes::NodeAccessStatus::ReadOnly) {
            LogMessage("IDS error: Auto whitebalance read mode not available currently.");
            return DEVICE_ERR;
        }
        try {
            whitebalanceCurr_ = node->CurrentEntry()->SymbolicValue();
        }
        catch (std::exception& e) {
            LogMessage("IDS exception: Could not read the auto whitebalance mode.");
            LogMessage(e.what());
        }
        pProp->Set(whitebalanceCurr_.c_str());
    }
    else if (eAct == MM::AfterSet)
    {
        if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

        std::string autoWB;
        pProp->Get(autoWB);
        
        // Already in that mode
        if (autoWB == whitebalanceCurr_) { return DEVICE_OK; }

        try {
            nodeMapRemoteDevice->FindNode<EnumNode>("BalanceWhiteAuto")
                ->SetCurrentEntry(autoWB);
            whitebalanceCurr_ = autoWB;
        }
        catch (std::exception& e) {
            LogMessage(("IDS exception: Could not set the auto whitebalance to: " + autoWB).c_str());
            LogMessage(e.what());
            return DEVICE_ERR;
        }
    }
    return nRet;
}

/**
* Handles "Gain master" property.
*/
int CIDSPeak::OnAnalogMaster(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int nRet = DEVICE_OK;
    if (eAct == MM::BeforeGet)
    {
        nRet = GetGain("AnalogAll", gainAnalogMaster_);
        if (nRet != DEVICE_OK) { return nRet; }
        pProp->Set(gainAnalogMaster_);
    }
    else if (eAct == MM::AfterSet)
    {
        if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

        double gain;
        pProp->Get(gain);

        nRet = SetGain("AnalogAll", gain);
        if (nRet != DEVICE_OK) { return nRet; }
        gainAnalogMaster_ = gain;
    }
    return nRet;
}

/**
* Handles "Gain red" property.
*/
int CIDSPeak::OnAnalogRed(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int nRet = DEVICE_OK;
    if (eAct == MM::BeforeGet)
    {
        nRet = GetGain("AnalogRed", gainAnalogRed_);
        if (nRet != DEVICE_OK) { return nRet; }
        pProp->Set(gainAnalogRed_);
    }
    else if (eAct == MM::AfterSet)
    {
        if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

        double gain;
        pProp->Get(gain);

        nRet = SetGain("AnalogRed", gain);
        if (nRet != DEVICE_OK) { return nRet; }
        gainAnalogRed_ = gain;
    }
    return nRet;
}

/**
* Handles "Gain green" property.
*/
int CIDSPeak::OnAnalogGreen(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int nRet = DEVICE_OK;
    if (eAct == MM::BeforeGet)
    {
        nRet = GetGain("AnalogGreen", gainAnalogGreen_);
        if (nRet != DEVICE_OK) { return nRet; }
        pProp->Set(gainAnalogGreen_);
    }
    else if (eAct == MM::AfterSet)
    {
        if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

        double gain;
        pProp->Get(gain);

        nRet = SetGain("AnalogGreen", gain);
        if (nRet != DEVICE_OK) { return nRet; }
        gainAnalogGreen_ = gain;
    }
    return nRet;
}

/**
* Handles "Gain blue" property.
*/
int CIDSPeak::OnAnalogBlue(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int nRet = DEVICE_OK;
    if (eAct == MM::BeforeGet)
    {
        nRet = GetGain("AnalogBlue", gainAnalogBlue_);
        if (nRet != DEVICE_OK) { return nRet; }
        pProp->Set(gainAnalogBlue_);
    }
    else if (eAct == MM::AfterSet)
    {
        if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

        double gain;
        pProp->Get(gain);

        nRet = SetGain("AnalogBlue", gain);
        if (nRet != DEVICE_OK) { return nRet; }
        gainAnalogBlue_ = gain;
    }
    return nRet;
}

/**
* Handles "Gain master" property.
*/
int CIDSPeak::OnDigitalMaster(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int nRet = DEVICE_OK;
    if (eAct == MM::BeforeGet)
    {
        nRet = GetGain("DigitalAll", gainDigitalMaster_);
        if (nRet != DEVICE_OK) { return nRet; }
        pProp->Set(gainDigitalMaster_);
    }
    else if (eAct == MM::AfterSet)
    {
        if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

        double gain;
        pProp->Get(gain);

        nRet = SetGain("DigitalAll", gain);
        if (nRet != DEVICE_OK) { return nRet; }
        gainDigitalMaster_ = gain;
    }
    return nRet;
}

/**
* Handles "Gain red" property.
*/
int CIDSPeak::OnDigitalRed(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int nRet = DEVICE_OK;
    if (eAct == MM::BeforeGet)
    {
        nRet = GetGain("DigitalRed", gainDigitalRed_);
        if (nRet != DEVICE_OK) { return nRet; }
        pProp->Set(gainDigitalRed_);
    }
    else if (eAct == MM::AfterSet)
    {
        if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

        double gain;
        pProp->Get(gain);

        nRet = SetGain("DigitalRed", gain);
        if (nRet != DEVICE_OK) { return nRet; }
        gainDigitalRed_ = gain;
    }
    return nRet;
}

/**
* Handles "Gain green" property.
*/
int CIDSPeak::OnDigitalGreen(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int nRet = DEVICE_OK;
    if (eAct == MM::BeforeGet)
    {
        nRet = GetGain("DigitalGreen", gainDigitalGreen_);
        if (nRet != DEVICE_OK) { return nRet; }
        pProp->Set(gainDigitalGreen_);
    }
    else if (eAct == MM::AfterSet)
    {
        if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

        double gain;
        pProp->Get(gain);

        nRet = SetGain("DigitalGreen", gain);
        if (nRet != DEVICE_OK) { return nRet; }
        gainDigitalGreen_ = gain;
    }
    return nRet;
}

/**
* Handles "Gain blue" property.
*/
int CIDSPeak::OnDigitalBlue(MM::PropertyBase* pProp, MM::ActionType eAct)
{

    int nRet = DEVICE_OK;
    if (eAct == MM::BeforeGet)
    {
        nRet = GetGain("DigitalBlue", gainDigitalBlue_);
        if (nRet != DEVICE_OK) { return nRet; }
        pProp->Set(gainDigitalBlue_);
    }
    else if (eAct == MM::AfterSet)
    {
        if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

        double gain;
        pProp->Get(gain);

        nRet = SetGain("DigitalBlue", gain);
        if (nRet != DEVICE_OK) { return nRet; }
        gainDigitalBlue_ = gain;
    }
    return nRet;
}

/**
* Handles "PixelType" property.
*/
int CIDSPeak::OnPixelType(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    switch (eAct)
    {
    case MM::AfterSet:
    {
        if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

        std::string pixelType;
        pProp->Get(pixelType);

        LogMessage(pixelType);

        if (pixelType == g_PixelType_8bit) {
            nComponents_ = 1;
            bitDepth_ = 8;
        }
        else if (pixelType == g_PixelType_16bit) {
            nComponents_ = 1;
            bitDepth_ = 16;
        }
        else if (pixelType == g_PixelType_32bitRGBA) {
            nComponents_ = 4;
            bitDepth_ = 8;
        }
        else {
            return DEVICE_CAN_NOT_SET_PROPERTY;
        }
        pixelType_ = pixelType;

        // Resize buffer to accomodate the new image
        img_.Resize(img_.Width(), img_.Height(), nComponents_ * (bitDepth_ / 8));
    }
    break;
    case MM::BeforeGet:
    {
        if (nComponents_ == 1 && bitDepth_ == 8) {
            pProp->Set(g_PixelType_8bit);
        }
        else if (nComponents_ == 1 && bitDepth_ == 16) {
            pProp->Set(g_PixelType_16bit);
        }
        else if (nComponents_ == 4 && bitDepth_ == 8) {
            pProp->Set(g_PixelType_32bitRGBA);
        }
        else {
            return DEVICE_CAN_NOT_SET_PROPERTY;
        }
    } break;
    default:
        break;
    }
    return DEVICE_OK;
}

/**
* Handles "PixelFormat" property.
*/
int CIDSPeak::OnPixelFormat(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int nRet = DEVICE_OK;
    switch (eAct) {
    case MM::AfterSet: {
        if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

        // Get requested pixelFormat
        std::string temp;
        pProp->Get(temp);

        // Check if pixelFormat didn't change
        if (temp == pixelFormat_) { return DEVICE_OK; }

        try {
            // Check for write access
            auto node = nodeMapRemoteDevice->FindNode<EnumNode>("PixelFormat");
            if (node->AccessStatus() != peak::core::nodes::NodeAccessStatus::ReadWrite) {
                LogMessage("IDS error: Currently no write access to PixelFormat.");
                return DEVICE_CAN_NOT_SET_PROPERTY;
            }

            // Write pixelFormat
            node->SetCurrentEntry(temp);
            pixelFormat_ = temp;
        }
        catch (std::exception& e) {
            LogMessage("IDS exception: Could not set PixelFormat.");
            LogMessage(e.what());
            return DEVICE_CAN_NOT_SET_PROPERTY;
        }
    }
    case MM::BeforeGet:
        pProp->Set(pixelFormat_.c_str());
        break;
    }
    return nRet;
}

int CIDSPeak::OnSensorTemp(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    // This is a readonly function
    if (eAct == MM::BeforeGet)
    {
        try {
            auto node = nodeMapRemoteDevice->FindNode<FloatNode>("DeviceTemperature");
            if (node->AccessStatus() != peak::core::nodes::NodeAccessStatus::ReadOnly && 
                node->AccessStatus() != peak::core::nodes::NodeAccessStatus::ReadWrite) {
                LogMessage("IDS error: Could not read temperature.");
                return DEVICE_ERR;
            }
            sensorTemp_ = node->Value();
        }
        catch (std::exception& e) {
            LogMessage("IDS exception: Could not read temperature.");
            LogMessage(e.what());
            return DEVICE_ERR;
        }
        pProp->Set(sensorTemp_);
    }
    return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// Private CIDSPeak methods
///////////////////////////////////////////////////////////////////////////////

/**
* Sync internal image buffer size to the chosen property values.
*/
int CIDSPeak::ResizeImageBuffer()
{
    char buf[MM::MaxStrLength];
    int nRet = GetProperty(MM::g_Keyword_Binning, buf);
    if (nRet != DEVICE_OK)
        return nRet;
    binSize_ = atol(buf);

    img_.Resize(sensorWidth_ / binSize_, sensorHeight_ / binSize_, nComponents_ * (bitDepth_ / 8));
    return DEVICE_OK;
}

void CIDSPeak::GenerateEmptyImage(ImgBuffer& img)
{
    if (img.Height() == 0 || img.Width() == 0 || img.Depth() == 0)
        return;
    MMThreadGuard g(imgPixelsLock_);
    unsigned char* pBuf = const_cast<unsigned char*>(img.GetPixels());
    memset(pBuf, 0, img.Height() * img.Width() * img.Depth());
}


int CIDSPeak::CreateGain(std::string gainType, double& gain,
    int(CIDSPeak::* fpt)(MM::PropertyBase* pProp, MM::ActionType eAct))
{
    double min = 0, max = 0;
    try {
        nodeMapRemoteDevice->FindNode<EnumNode>("GainSelector")
            ->SetCurrentEntry(gainType);
        auto node = nodeMapRemoteDevice->FindNode<FloatNode>("Gain");
        if (node->AccessStatus() != peak::core::nodes::NodeAccessStatus::ReadWrite) {
            LogMessage(("IDS warning: " + gainType + " not available").c_str());
            return DEVICE_OK;
        }
        gain = node->Value();
        min = node->Minimum();
        max = node->Maximum();
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Error occurred during creation of: " + gainType);
        LogMessage(e.what());
        return DEVICE_ERR;
    }

    CPropertyAction* pAct = new CPropertyAction(this, fpt);
    int nRet = CreateFloatProperty(("Gain " + gainType).c_str(), gain, false, pAct);
    if (nRet != DEVICE_OK) { return nRet; }
    SetPropertyLimits(("Gain " + gainType).c_str(), min, max);
    if (nRet != DEVICE_OK) { return nRet; }
    return DEVICE_OK;
}

int CIDSPeak::GetGain(const std::string& gainType, double& gain) {
    try {
        nodeMapRemoteDevice->FindNode<EnumNode>("GainSelector")->SetCurrentEntry(gainType);
        gain = nodeMapRemoteDevice->FindNode<FloatNode>("Gain")->Value();
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not get " + gainType);
        LogMessage(e.what());
        return DEVICE_ERR;
    }
    return DEVICE_OK;
}

int CIDSPeak::SetGain(const std::string& gainType, double gain) {
    if (IsCapturing()) { return DEVICE_CAMERA_BUSY_ACQUIRING; }

    try {
        nodeMapRemoteDevice->FindNode<EnumNode>("GainSelector")->SetCurrentEntry(gainType);
        nodeMapRemoteDevice->FindNode<FloatNode>("Gain")->SetValue(gain);
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not set " + gainType);
        LogMessage(e.what());
        return DEVICE_CAN_NOT_SET_PROPERTY;
    }
    return DEVICE_OK;
}

int CIDSPeak::setFrameRate(double frameRate) {
    auto node = nodeMapRemoteDevice->FindNode<FloatNode>("AcquisitionFrameRate");
    // Check if we have access
    if (node->AccessStatus() != peak::core::nodes::NodeAccessStatus::ReadWrite) { return DEVICE_CAN_NOT_SET_PROPERTY; }

    // Make sure frameRate is multiple of increment
    frameRate = std::floor(frameRate / frameRateInc_) * frameRateInc_;
    try {
        node->SetValue(frameRate);
        frameRateCur_ = node->Value();
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not set frame rate.");
        LogMessage(e.what());
        return DEVICE_CAN_NOT_SET_PROPERTY;
    }
    return DEVICE_OK;
}

int CIDSPeak::setFrameCount(long count) {
    auto node = nodeMapRemoteDevice->FindNode<EnumNode>("AcquisitionMode");
    if (node->AccessStatus() != peak::core::nodes::NodeAccessStatus::ReadWrite) {
        return DEVICE_CAN_NOT_SET_PROPERTY;
    }

    try {
        if (count == 1) {
            node->SetCurrentEntry("SingleFrame");
        }
        else if (count == LONG_MAX) {
            node->SetCurrentEntry("Continuous");
        }
        else {
            node->SetCurrentEntry("MultiFrame");
            nodeMapRemoteDevice->FindNode<IntNode>("AcquisitionFrameCount")->SetValue(count);
        }
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: An error occurred while setting acquisition mode.");
        LogMessage(e.what());
        return DEVICE_CAN_NOT_SET_PROPERTY;
    }
    return DEVICE_OK;
}

int CIDSPeak::ClearBuffer() {
    if (!dataStream) {
        LogMessage("IDS error: No data stream to clear buffer. This error is unrecoverable.");
        return DEVICE_ERR;
    }

    try {
        // Flush queue (input and output)
        dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll);
        // Revoke all old buffers
        for (const auto& buffer : dataStream->AnnouncedBuffers()) {
            dataStream->RevokeBuffer(buffer);
        }
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not clear buffer.");
        LogMessage(e.what());
        return DEVICE_ERR;
    }
    return DEVICE_OK;
}

int CIDSPeak::PrepareBuffer() {
    int nRet = ClearBuffer();
    if (nRet != DEVICE_OK) { return nRet; }

    try {
        payloadSize_ = nodeMapRemoteDevice->FindNode<IntNode>("PayloadSize")->Value();

        // Allocate buffers
        for (size_t count = 0; count < nBuffers_; count++) {
            auto buffer = dataStream->AllocAndAnnounceBuffer(static_cast<size_t>(payloadSize_), nullptr);
            dataStream->QueueBuffer(buffer);
        }
    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not allocate buffers.");
        LogMessage(e.what());
        return DEVICE_ERR;
    }
    return DEVICE_OK;
}

int CIDSPeak::AcquireAndTransferImage(uint64_t timeout_ms, bool insertImage) {
    try {
		MMThreadGuard g(imgPixelsLock_);
		unsigned char* pBuf = (unsigned char*) const_cast<unsigned char*>(img_.GetPixels());

        // Get image from camera buffer
        const auto buffer = dataStream->WaitForFinishedBuffer(timeout_ms);

        // Convert buffer to expected format
        if (pixelFormat_ != destinationFormat_.Name()) {
            const auto image = peak::BufferTo<peak::ipl::Image>(buffer).ConvertTo(
                destinationFormat_, peak::ipl::ConversionMode::Fast);
			// Transfer buffer to ImgBuffer
			memcpy(pBuf, image.Data(), payloadSize_);
        }
        else {
            const auto image = peak::BufferTo<peak::ipl::Image>(buffer);
            // Transfer buffer to ImgBuffer
            memcpy(pBuf, image.Data(), payloadSize_);
        }

        // Requeue (release) buffer
        dataStream->QueueBuffer(buffer);

    }
    catch (std::exception& e) {
        LogMessage("IDS exception: Could not acquire image or transfer it to the Micro-Manager buffer.");
        LogMessage(e.what());
        return DEVICE_ERR;
    }

	// Insert image in circular buffer
	if (insertImage) { InsertImage(); }

    return DEVICE_OK;
}
