#pragma once

#include "hdtSkyrimSystem.h"

//
namespace hdt
{
    namespace util
    {
        uint32_t splitArmorAddonFormID(const std::string& nodeName);
        std::string UInt32toString(uint32_t formID);
        void transferCurrentPosesBetweenSystems(hdt::SkyrimSystem* src, hdt::SkyrimSystem* dst);
    }
}
