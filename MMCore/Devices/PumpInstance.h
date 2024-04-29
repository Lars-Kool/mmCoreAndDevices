// PROJECT:       Micro-Manager
// SUBSYSTEM:     MMCore
//
// DESCRIPTION:   Pump device instance wrapper
//
// COPYRIGHT:     Institut Pierre-Gilles de Gennes, Paris, 2024,
//                All Rights reserved
//
// LICENSE:       This file is distributed under the "Lesser GPL" (LGPL) license.
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
// AUTHOR:        Lars Kool, Institut Pierre-Gilles de Gennes

#pragma once

#include "DeviceInstanceBase.h"

class PumpInstance : public DeviceInstanceBase<MM::Pump>
{
public:
    PumpInstance(CMMCore* core,
        std::shared_ptr<LoadedDeviceAdapter> adapter,
        const std::string& name,
        MM::Device* pDevice,
        DeleteDeviceFunction deleteFunction,
        const std::string& label,
        mm::logging::Logger deviceLogger,
        mm::logging::Logger coreLogger) :
        DeviceInstanceBase<MM::Pump>(core, adapter, name, pDevice, deleteFunction, label, deviceLogger, coreLogger)
    {}

    // Volume controlled pump specific
    int Home();
    int Stop();
    int invertDirection(bool state);
    int isDirectionInverted(bool& state);
    int setVolumeUl(double volUl);
    int getVolumeUl(double& volUl);
    int setMaxVolumeUl(double volUl);
    int getMaxVolumeUl(double& volUl);
    int setFlowrateUlPerSec(double flowrate);
    int getFlowrateUlPerSec(double& flowrate);
    int Dispense();
    int DispenseDuration(double durSec);
    int DispenseVolume(double volUl);

    // Pressure controller specific
    int Calibrate();
    int setPressure(double pressure);
    int getPressure(double& pressure);
};
