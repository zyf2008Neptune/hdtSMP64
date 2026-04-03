#include "config.h"

#include <algorithm>
#include <cstdint>
#include <locale>
#include <sstream>
#include <string>

#include <LinearMath/btMinMax.h>
#include <SKSE/Logger.h>

#include "ActorManager.h"
#include "hdtSkinnedMesh/hdtConstraintGroup.h"
#include "hdtSkyrimPhysicsWorld.h"
#include "Hooks.h"
#include "NetImmerseUtils.h"
#include "XmlInspector/XmlInspector.hpp"
#include "XmlReader.h"
#ifdef CUDA
#	include "hdtSkinnedMesh/hdtCudaInterface.h"
#endif

namespace hdt
{
    int g_logLevel;

    static auto solver(XMLReader& reader) -> void
    {
        while (reader.Inspect())
        {
            switch (reader.GetInspected())
            {
            case XMLReader::Inspected::StartTag:
                if (reader.GetLocalName() == "numIterations")
                {
                    SkyrimPhysicsWorld::get()->getSolverInfo().m_numIterations = btClamped(reader.readInt(), 4, 128);
                }
                else if (reader.GetLocalName() == "groupIterations")
                {
                    ConstraintGroup::MaxIterations = btClamped(reader.readInt(), 0, 4096);
                }
                else if (reader.GetLocalName() == "groupEnableMLCP")
                {
                    ConstraintGroup::EnableMLCP = reader.readBool();
                }
                else if (reader.GetLocalName() == "erp")
                {
                    SkyrimPhysicsWorld::get()->getSolverInfo().m_erp = btClamped(reader.readFloat(), 0.01f, 1.0f);
                }
                else if (reader.GetLocalName() == "min-fps")
                {
                    SkyrimPhysicsWorld::get()->min_fps = (btClamped(reader.readInt(), 1, 300));
                    SkyrimPhysicsWorld::get()->m_timeTick = 1.0f / SkyrimPhysicsWorld::get()->min_fps;
                }
                else if (reader.GetLocalName() == "maxSubSteps")
                {
                    SkyrimPhysicsWorld::get()->m_maxSubSteps = btClamped(reader.readInt(), 1, 60);
                }
                else
                {
                    logger::warn("Unknown config : {}", reader.GetLocalName());
                    reader.skipCurrentElement();
                }
                break;
            case XMLReader::Inspected::EndTag:
                return;
            }
        }
    }

    static auto wind(XMLReader& reader) -> void
    {
        while (reader.Inspect())
        {
            switch (reader.GetInspected())
            {
            case XMLReader::Inspected::StartTag:
                if (reader.GetLocalName() == "windStrength")
                {
                    SkyrimPhysicsWorld::get()->m_windStrength = btClamped(reader.readFloat(), 0.f, 1000.f);
                }
                else if (reader.GetLocalName() == "enabled")
                {
                    SkyrimPhysicsWorld::get()->m_enableWind = reader.readBool();
                }
                else if (reader.GetLocalName() == "distanceForNoWind")
                {
                    SkyrimPhysicsWorld::get()->m_distanceForNoWind = btClamped(reader.readFloat(), 0.f, 10000.f);
                }
                else if (reader.GetLocalName() == "distanceForMaxWind")
                {
                    SkyrimPhysicsWorld::get()->m_distanceForMaxWind = btClamped(reader.readFloat(), 0.f, 10000.f);
                }
                else
                {
                    logger::warn("Unknown config : ", reader.GetLocalName());
                    reader.skipCurrentElement();
                }
                break;
            case XMLReader::Inspected::EndTag:
                return;
            }
        }
    }

    static auto smp(XMLReader& reader) -> void
    {
        while (reader.Inspect())
        {
            switch (reader.GetInspected())
            {
            case XMLReader::Inspected::StartTag:
                if (reader.GetLocalName() == "logLevel")
                {
                    g_logLevel = reader.readInt();
                }
                else if (reader.GetLocalName() == "backupNodeByName")
                {
                    // Parse the string return value from reader.readText(); so we can have single strings instead of the group, example text -> "Virtual Hands, Virtual Body, Virtual Belly"... said text in a array like so -> { "Virtual Hands", "Virtual Body", "Virtual Belly"

                    std::stringstream ss(reader.readText());
                    std::string item;

                    while (std::getline(ss, item, ','))
                    {
                        // Remove leading space
                        if (!item.empty() && item[0] == ' ')
                        {
                            item.erase(0, 1);
                        }

                        Hooks::BipedAnimHooks::BackupNodes.push_back(item);
                    }
                }
                else if (reader.GetLocalName() == "enableNPCFaceParts")
                {
                    ActorManager::instance()->m_skinNPCFaceParts = reader.readBool();
                }
                else if (reader.GetLocalName() == "disableSMPHairWhenWigEquipped")
                {
                    ActorManager::instance()->m_disableSMPHairWhenWigEquipped = reader.readBool();
                }
                else if (reader.GetLocalName() == "clampRotations")
                {
                    SkyrimPhysicsWorld::get()->m_clampRotations = reader.readBool();
                }
                else if (reader.GetLocalName() == "rotationSpeedLimit")
                {
                    SkyrimPhysicsWorld::get()->m_rotationSpeedLimit = reader.readFloat();
                }
                else if (reader.GetLocalName() == "unclampedResets")
                {
                    SkyrimPhysicsWorld::get()->m_unclampedResets = reader.readBool();
                }
                else if (reader.GetLocalName() == "unclampedResetAngle")
                {
                    SkyrimPhysicsWorld::get()->m_unclampedResetAngle = reader.readFloat();
                }
                else if (reader.GetLocalName() == "percentageOfFrameTime")
                {
                    SkyrimPhysicsWorld::get()->m_percentageOfFrameTime = std::clamp(reader.readInt() * 10, 1, 1000);
                }
                else if (reader.GetLocalName() == "useRealTime")
                {
                    SkyrimPhysicsWorld::get()->m_useRealTime = reader.readBool();
                }
#ifdef CUDA
				else if (reader.GetLocalName() == "enableCuda")
                {
                    CudaInterface::enableCuda = reader.readBool();
                }
                else if (reader.GetLocalName() == "cudaDevice")
                {
                    int device = reader.readInt();
                    if (device >= 0 && device < CudaInterface::instance()->deviceCount())
                    {
                        CudaInterface::currentDevice = device;
                    }
                }
#else
                else if (reader.GetLocalName() == "enableCuda")
                {
                    if (reader.readBool())
                    {
                        logger::warn("CUDA isn't built into this version.");
                    }
                }
                else if (reader.GetLocalName() == "cudaDevice") {
                }
#endif
                else if (reader.GetLocalName() == "minCullingDistance")
                {
                    ActorManager::instance()->m_minCullingDistance = reader.readFloat();
                }
                else if (reader.GetLocalName() == "maximumActiveSkeletons")
                {
                    ActorManager::instance()->m_maxActiveSkeletons = reader.readInt();
                }
                else if (reader.GetLocalName() == "autoAdjustMaxSkeletons")
                {
                    ActorManager::instance()->m_autoAdjustMaxSkeletons = reader.readBool();
                }
                else if (reader.GetLocalName() == "sampleSize")
                {
                    SkyrimPhysicsWorld::get()->m_sampleSize = std::max(reader.readInt(), 1);
                }

                else if (reader.GetLocalName() == "disable1stPersonViewPhysics")
                {
                    ActorManager::instance()->m_disable1stPersonViewPhysics = reader.readBool();
                }
                else
                {
                    logger::warn("Unknown config : {}", reader.GetLocalName());
                    reader.skipCurrentElement();
                }
                break;
            case XMLReader::Inspected::EndTag:
                return;
            }
        }
    }

    static auto config(XMLReader& reader) -> void
    {
        while (reader.Inspect())
        {
            switch (reader.GetInspected())
            {
            case XMLReader::Inspected::StartTag:
                if (reader.GetLocalName() == "solver")
                {
                    solver(reader);
                }
                else if (reader.GetLocalName() == "wind")
                {
                    wind(reader);
                }
                else if (reader.GetLocalName() == "smp")
                {
                    smp(reader);
                }
                else
                {
                    logger::warn("Unknown config : {}", reader.GetLocalName());
                    reader.skipCurrentElement();
                }
                break;
            case XMLReader::Inspected::EndTag:
                return;
            }
        }
    }

    auto loadConfig() -> void
    {
        auto bytes = readAllFile2("data/skse/plugins/hdtSkinnedMeshConfigs/configs.xml");
        if (bytes.empty())
        {
            return;
        }

        // Store original locale

        const auto saved_locale = std::locale();

        // Set locale to en_US
        std::locale::global(std::locale("en_US"));

        XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());

        while (reader.Inspect())
        {
            if (reader.GetInspected() == XMLReader::Inspected::StartTag)
            {
                if (reader.GetLocalName() == "configs")
                {
                    config(reader);
                }
                else
                {
                    logger::warn("Unknown config : {}", reader.GetLocalName());
                    reader.skipCurrentElement();
                }
            }
        }

        // Restore original locale
        std::locale::global(saved_locale);
    }
}
