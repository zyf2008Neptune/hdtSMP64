#pragma once

#include "hdtSkyrimSystem.h"

//
namespace hdt::util
{
    auto splitArmorAddonFormID(const std::string& nodeName) -> uint32_t;
    auto UInt32toString(uint32_t formID) -> std::string;
    auto transferCurrentPosesBetweenSystems(SkyrimSystem* src, SkyrimSystem* dst) -> void;
} // namespace hdt::util
