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

#include "PumpInstance.h"

// Volume controlled pump functions
int PumpInstance::Home() { return GetImpl()->Home(); }
int PumpInstance::Stop() { return GetImpl()->Stop(); }
int PumpInstance::invertDirection(bool state) { return GetImpl()->InvertDirection(state); }
int PumpInstance::isDirectionInverted(bool& state) { return GetImpl()->IsDirectionInverted(state); }
int PumpInstance::setVolumeUl(double volume) { return GetImpl()->SetVolumeUl(volume); }
int PumpInstance::getVolumeUl(double& volume) { return GetImpl()->GetVolumeUl(volume); }
int PumpInstance::setMaxVolumeUl(double volume) { return GetImpl()->SetMaxVolumeUl(volume); }
int PumpInstance::getMaxVolumeUl(double& volume) { return GetImpl()->GetMaxVolumeUl(volume); }
int PumpInstance::setFlowrateUlPerSec(double flowrate) { return GetImpl()->SetFlowrateUlPerSecond(flowrate); }
int PumpInstance::getFlowrateUlPerSec(double& flowrate) { return GetImpl()->GetFlowrateUlPerSecond(flowrate); }
int PumpInstance::Dispense() { return GetImpl()->Dispense(); }
int PumpInstance::DispenseDuration(double durSec) { return GetImpl()->DispenseDuration(durSec); }
int PumpInstance::DispenseVolume(double volUl) { return GetImpl()->DispenseVolume(volUl); }

// Pressure controlled pump functions
int PumpInstance::Calibrate() { return GetImpl()->Calibrate(); }
int PumpInstance::setPressure(double pressure) { return GetImpl()->SetPressure(pressure); }
int PumpInstance::getPressure(double& pressure) { return GetImpl()->GetPressure(pressure); }