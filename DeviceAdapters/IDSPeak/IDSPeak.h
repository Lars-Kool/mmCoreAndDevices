///////////////////////////////////////////////////////////////////////////////
// FILE:          IDSPeak.h
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

#ifndef _IDSPeak_H_
#define _IDSPeak_H_

#include "DeviceBase.h"
#include "ImgBuffer.h"
#include "DeviceThreads.h"
#include <string>
#include <map>
#include <algorithm>
#include <stdint.h>
#include <future>

#include <peak/peak.hpp>
#include <peak_ipl/peak_ipl.hpp>

#define EXPOSURE_MAX 1000000

////////////////////////////////////////
// Error codes
////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// IDSPeakHub class
// Hub for IDS Peak cameras
//////////////////////////////////////////////////////////////////////////////

class IDSPeakHub : public HubBase<IDSPeakHub>
{
public:
    IDSPeakHub();
    ~IDSPeakHub();

    // Device API
    int Initialize();
    int Shutdown();
    void GetName(char* pName) const;
    bool Busy();

    // HUB api
    int DetectInstalledDevices();

    // Hub parameters

    // Hub action handlers

    // Hub members

private:
    bool initialized_;
    bool busy_;
    int nCameras_;
};

//////////////////////////////////////////////////////////////////////////////
// CIDSPeak class
//////////////////////////////////////////////////////////////////////////////

class MySequenceThread;

class CIDSPeak : public CCameraBase<CIDSPeak>
{
public:
    CIDSPeak(int deviceIdx);
    ~CIDSPeak();

    // MMDevice API
    // ------------
    int Initialize();
    int Shutdown();
    void GetName(char* name) const;

    // MMCamera API
    int SnapImage();
    const unsigned char* GetImageBuffer();
    unsigned GetImageWidth() const;
    unsigned GetImageHeight() const;
    unsigned GetImageBytesPerPixel() const;
    unsigned GetBitDepth() const;
    long GetImageBufferSize() const;
    int SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize);
    int GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize);
    int ClearROI();
    int GetBinning() const;
	int SetBinning(int binF);
    double GetExposure() const;
    double GetSequenceExposure();
    void SetExposure(double exp);
    int IsExposureSequenceable(bool& isSequenceable) const;
    int GetExposureSequenceMaxLength(long& nrEvents) const;
    int StartExposureSequence();
    int StopExposureSequence();
    int ClearExposureSequence();
    int AddToExposureSequence(double exposureTime_ms);
    int SendExposureSequence() const;
    int StartSequenceAcquisition(double interval);
    int StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow);
    int StopSequenceAcquisition();

    // Action Handlers
    int OnEnableTemperature(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnEnableAnalogGain(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnEnableDigitalGain(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnEnableAutoWhitebalance(MM::PropertyBase* pProp, MM::ActionType eAct);

    int OnBinning(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnFrameRate(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnPixelFormat(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnPixelType(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnConversionMode(MM::PropertyBase* pProp, MM::ActionType eAct);

    int OnAutoWhitebalance(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnAnalogMaster(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnAnalogRed(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnAnalogGreen(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnAnalogBlue(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnDigitalMaster(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnDigitalRed(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnDigitalGreen(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnDigitalBlue(MM::PropertyBase* pProp, MM::ActionType eAct);

    int OnSensorTemp(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
    // Gain helper functions
    int CreateGain(std::string gainType, double& gain,
        int(CIDSPeak::* fpt)(MM::PropertyBase* pProp, MM::ActionType eAct));
    int GetGain(const std::string& gainType, double& gain);
    int SetGain(const std::string& gainType, double gain);

    // Image/buffer helper functions
    void GenerateEmptyImage(ImgBuffer& img);
    int ResizeImageBuffer();
    int InsertImage();
    int ClearBuffer();
    int PrepareBuffer();
    int AcquireAndTransferImage(uint64_t timeout_ms, bool insertImage);

    // Thread related
    int RunSequenceOnThread();
    bool IsCapturing();
    void OnThreadExiting() throw();
    int StartAcquisition(long numImages);
    int StopAcquisition();

    // IDS Setters (getting is done from this class directly)
    int setFrameRate(double frameRate);
    int setFrameCount(long count);

    // IDS parameters
    bool initialized_ = false;
    int deviceIdx_ = INT_MAX;
    std::string modelName_;
    std::string serialNumber_;
    std::shared_ptr<peak::core::DeviceDescriptor> descriptor = nullptr;
    std::shared_ptr<peak::core::Device> device = nullptr;
    std::shared_ptr<peak::core::NodeMap> nodeMapRemoteDevice = nullptr;
    std::shared_ptr<peak::core::DataStream> dataStream = nullptr;
    std::shared_ptr<peak::core::NodeMap> nodeMapDataStream = nullptr;

    // Enable optional features
    bool enableAutoWhitebalance_ = false;
    bool enableAnalogGain_ = false;
    bool enableDigitalGain_ = false;
    bool enableTemperature_ = false;
    
    // Exposure time
    double exposureCur_ = 0;
    double exposureMin_ = 0;
    double exposureMax_ = 0;
    double exposureInc_ = 0;

    // FrameRate
    double frameRateCur_ = 0;
    double frameRateMin_ = 0;
    double frameRateMax_ = 0;
    double frameRateInc_ = 0;

    // Autowhitebalance
    std::string whitebalanceCurr_ = "";

    // Analog Gain
    double gainAnalogMaster_ = 0;
    double gainAnalogRed_ = 0;
    double gainAnalogGreen_ = 0;
    double gainAnalogBlue_ = 0;

    // Digital Gain
    double gainDigitalMaster_ = 0;
    double gainDigitalRed_ = 0;
    double gainDigitalGreen_ = 0;
    double gainDigitalBlue_ = 0;

    // Sensor properties
    long sensorHeight_ = 0;
    long sensorWidth_ = 0;
    double sensorTemp_ = 0;

    // Image properties
    ImgBuffer img_;
    unsigned roiX_ = 0;
    unsigned roiY_ = 0;
    unsigned roiOffsetXMin_ = 0;
    unsigned roiOffsetXMax_ = 0;
    unsigned roiOffsetYMin_ = 0;
    unsigned roiOffsetYMax_ = 0;
    unsigned roiWidthMin_ = 0;
    unsigned roiWidthMax_ = 0;
    unsigned roiHeightMin_ = 0;
    unsigned roiHeightMax_ = 0;
    unsigned binSize_ = 0;
    std::string pixelFormat_;
    std::string pixelType_;
    peak::ipl::PixelFormat destinationFormat_ = peak::ipl::PixelFormatName::BGRa8;
    peak::ipl::ConversionMode conversionMode_ = peak::ipl::ConversionMode::Classic;
    int nComponents_ = 1;
    int bitDepth_ = 8;
    int nBuffers_ = 0;
    int64_t payloadSize_ = 0; // Buffer size in bytes

    // Exposure sequence
    std::string acquisitionMode_ = "";
    std::vector<double> exposureSequence_;
    bool isSequenceable_ = false;
    bool sequenceRunning_ = false;
    long sequenceMaxLength_ = 0;
    int sequenceIndex_ = 0;
    MM::MMTime sequenceStartTime_;
    int imageCounter_ = 0;
    bool stopOnOverflow_ = false;

    // ThreadLocks
    MMThreadLock imgPixelsLock_;

    friend class MySequenceThread;
    MySequenceThread* thd_ = nullptr;
    MM::MMTime readoutStartTime_;
};

class MySequenceThread : public MMDeviceThreadBase
{
    friend class CIDSPeak;
    enum { default_numImages = 1, default_intervalMS = 100 };
public:
    MySequenceThread(CIDSPeak* pCam);
    ~MySequenceThread();
    void Stop();
    void Start(long numImages, double intervalMs);
    bool IsStopped();
    void Suspend();
    bool IsSuspended();
    void Resume();
    double GetIntervalMs() { return intervalMs_; }
    //void SetIntervalMs(double intervalms);
    void SetLength(long images) { numImages_ = images; }
    long GetLength() const { return numImages_; }
    long GetImageCounter() { return imageCounter_; }
    MM::MMTime GetStartTime() { return startTime_; }
    MM::MMTime GetActualDuration() { return actualDuration_; }
private:
    int svc(void) throw();
    double intervalMs_;
    long numImages_;
    long imageCounter_;
    bool stop_;
    bool suspend_;
    CIDSPeak* camera_;
    MM::MMTime startTime_;
    MM::MMTime actualDuration_;
    MM::MMTime lastFrameTime_;
    MMThreadLock stopLock_;
    MMThreadLock suspendLock_;
};

#endif //_IDSPeak_H_