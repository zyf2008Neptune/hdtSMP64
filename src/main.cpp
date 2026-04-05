#include <algorithm>
#include <cstdint>
#include <numeric>
#include <string>

#include <boost/beast/core/string.hpp>
#include <RE/B/BSLightingShaderMaterial.h>
#include <RE/B/BSTextureSet.h>
#include <RE/N/NiSourceTexture.h>

#include "ActorManager.h"
#include "config.h"
#include "dhdtOverrideManager.h"
#include "dhdtPapyrusFunctions.h"
#include "Events.h"
#include "hdtSkyrimPhysicsWorld.h"
#include "Hooks.h"
#include "PluginInterfaceImpl.h"
#include "WeatherManager.h"

static auto checkOldPlugins() -> void
{
    const auto framework = GetModuleHandleA("hdtSSEFramework");
    const auto physics = GetModuleHandleA("hdtSSEPhysics");
    const auto hh = GetModuleHandleA("hdtSSEHighHeels");

    if (physics)
    {
        MessageBox(
            nullptr,
            TEXT("hdtSSEPhysics.dll is loaded. This is an older verson of HDT-SMP and conflicts with hdtSMP64.dll. "
                "Please remove it."),
            TEXT("hdtSMP64"), MB_OK);
    }

    if (framework && !hh)
    {
        MessageBox(nullptr,
                   TEXT("hdtSSEFramework.dll is loaded but hdtSSEHighHeels.dll is not being used. You no longer need "
                       "hdtSSEFramework.dll with this version of SMP. Please remove it."),
                   TEXT("hdtSMP64"), MB_OK);
    }
}

static auto GetTextureFromIndex(RE::BSLightingShaderMaterial* material, const std::uint32_t index)
    -> RE::NiSourceTexturePtr*
{
    switch (index)
    {
    case RE::BSTextureSet::Texture::kDiffuse:
        return std::addressof(material->diffuseTexture);
    case RE::BSTextureSet::Texture::kNormal:
        return std::addressof(material->normalTexture);
    case RE::BSTextureSet::Texture::kEnvironmentMask:
    {
        if (material->GetFeature() == RE::BSShaderMaterial::Feature::kFaceGen)
        {
            return std::addressof(dynamic_cast<RE::BSLightingShaderMaterialFacegen*>(
                    static_cast<RE::BSLightingShaderMaterialBase*>(material))
                ->subsurfaceTexture);
        }
        if (material->GetFeature() == RE::BSShaderMaterial::Feature::kGlowMap)
        {
            return std::addressof(dynamic_cast<RE::BSLightingShaderMaterialFacegen*>(
                    static_cast<RE::BSLightingShaderMaterialBase*>(material))
                ->subsurfaceTexture);
        }
        return std::addressof(material->rimSoftLightingTexture);
    }
    break;
    case RE::BSTextureSet::Texture::kGlowMap:
    {
        if (material->GetFeature() == RE::BSShaderMaterial::Feature::kFaceGen)
        {
            return std::addressof(dynamic_cast<RE::BSLightingShaderMaterialFacegen*>(
                    static_cast<RE::BSLightingShaderMaterialBase*>(material))
                ->detailTexture);
        }
        if (material->GetFeature() == RE::BSShaderMaterial::Feature::kParallax)
        {
            return std::addressof(dynamic_cast<RE::BSLightingShaderMaterialParallax*>(
                    static_cast<RE::BSLightingShaderMaterialBase*>(material))
                ->heightTexture);
        }
        if (material->GetFeature() == RE::BSShaderMaterial::Feature::kParallax ||
            material->GetFeature() == RE::BSShaderMaterial::Feature::kParallaxOcc)
        {
            return std::addressof(dynamic_cast<RE::BSLightingShaderMaterialParallaxOcc*>(
                    static_cast<RE::BSLightingShaderMaterialBase*>(material))
                ->heightTexture);
        }
    }
    break;
    case RE::BSTextureSet::Texture::kHeight:
    {
        if (material->GetFeature() == RE::BSShaderMaterial::Feature::kEye)
        {
            return std::addressof(
                dynamic_cast<RE::BSLightingShaderMaterialEye*>(static_cast<RE::BSLightingShaderMaterialBase*>(material))
                ->envTexture);
        }
        if (material->GetFeature() == RE::BSShaderMaterial::Feature::kEnvironmentMap)
        {
            return std::addressof(dynamic_cast<RE::BSLightingShaderMaterialEnvmap*>(
                    static_cast<RE::BSLightingShaderMaterialBase*>(material))
                ->envTexture);
        }
        if (material->GetFeature() == RE::BSShaderMaterial::Feature::kMultilayerParallax)
        {
            return std::addressof(dynamic_cast<RE::BSLightingShaderMaterialMultiLayerParallax*>(
                    static_cast<RE::BSLightingShaderMaterialBase*>(material))
                ->envTexture);
        }
    }
    break;
    case RE::BSTextureSet::Texture::kEnvironment:
    {
        if (material->GetFeature() == RE::BSShaderMaterial::Feature::kEye)
        {
            return std::addressof(
                dynamic_cast<RE::BSLightingShaderMaterialEye*>(static_cast<RE::BSLightingShaderMaterialBase*>(material))
                ->envMaskTexture);
        }
        if (material->GetFeature() == RE::BSShaderMaterial::Feature::kEnvironmentMap)
        {
            return std::addressof(dynamic_cast<RE::BSLightingShaderMaterialEnvmap*>(
                    static_cast<RE::BSLightingShaderMaterialBase*>(material))
                ->envTexture);
        }
        if (material->GetFeature() == RE::BSShaderMaterial::Feature::kMultilayerParallax)
        {
            return std::addressof(dynamic_cast<RE::BSLightingShaderMaterialMultiLayerParallax*>(
                    static_cast<RE::BSLightingShaderMaterialBase*>(material))
                ->envMaskTexture);
        }
    }
    break;
    case RE::BSTextureSet::Texture::kMultilayer:
    {
        if (material->GetFeature() == RE::BSShaderMaterial::Feature::kFaceGen)
        {
            return std::addressof(dynamic_cast<RE::BSLightingShaderMaterialFacegen*>(
                    static_cast<RE::BSLightingShaderMaterialBase*>(material))
                ->tintTexture);
        }
        if (material->GetFeature() == RE::BSShaderMaterial::Feature::kMultilayerParallax)
        {
            return std::addressof(dynamic_cast<RE::BSLightingShaderMaterialMultiLayerParallax*>(
                    static_cast<RE::BSLightingShaderMaterialBase*>(material))
                ->layerTexture);
        }
    }
    break;
    case RE::BSTextureSet::Texture::kBacklightMask:
        return std::addressof(material->specularBackLightingTexture);
        break;
    }

    return nullptr;
}

static auto DumpNodeChildren(RE::NiAVObject* node) -> void
{
    logger::info("{} {} [{:.2f}, {:.2f}, {:.2f}]", node->GetRTTI()->name, node->name, node->world.translate.x,
                 node->world.translate.y, node->world.translate.z);

    if (node->extraDataSize > 0)
    {
        for (uint16_t i = 0; i < node->extraDataSize; i++)
        {
            logger::info("{} {}", node->extra[i]->GetRTTI()->name, node->extra[i]->name);
        }
    }

    RE::NiNode* niNode = node->AsNode();
    auto& children = niNode->GetChildren();
    if (niNode && !children.empty())
    {
        for (const auto& object : children)
        {
            if (object)
            {
                RE::NiNode* childNode = object->AsNode();
                if (RE::BSGeometry* geometry = object->AsGeometry())
                {
                    logger::info("{} {} [{:.2f}, {:.2f}, {:.2f}] - Geometry", object->GetRTTI()->name, object->name,
                                 geometry->world.translate.x, geometry->world.translate.y, geometry->world.translate.z);

                    if (geometry->GetGeometryRuntimeData().skinInstance &&
                        geometry->GetGeometryRuntimeData().skinInstance->skinData)
                    {
                        for (uint32_t boneIdx = 0;
                             boneIdx < geometry->GetGeometryRuntimeData().skinInstance->skinData->bones; boneIdx++)
                        {
                            const auto bone = geometry->GetGeometryRuntimeData().skinInstance->bones[boneIdx];
                            logger::info("Bone {} - {} {} [{:.2f}, {:.2f}, {:.2f}]", boneIdx, bone->GetRTTI()->name,
                                         bone->name, bone->world.translate.x, bone->world.translate.y,
                                         bone->world.translate.z);
                        }
                    }

                    if (const auto shaderProperty = geometry->GetGeometryRuntimeData().shaderProperty.get())
                    {
                        if (const auto lightingShader = netimmerse_cast<RE::BSLightingShaderProperty*>(shaderProperty))
                        {
                            const auto material = dynamic_cast<RE::BSLightingShaderMaterial*>(lightingShader->material);

                            for (auto i = 0; i < RE::BSTextureSet::Textures::kTotal; ++i)
                            {
                                const auto textureID = static_cast<RE::BSTextureSet::Textures::Texture>(i);

                                auto texturePath = material->textureSet->GetTexturePath(textureID);
                                if (!texturePath)
                                {
                                    continue;
                                }

                                auto textureName = "";
                                const RE::NiSourceTexturePtr* texture = GetTextureFromIndex(material, textureID);
                                if (texture && texture->get())
                                {
                                    textureName = texture->get()->name.c_str();
                                }

                                logger::info("Texture {} - {} ({})", i, texturePath, textureName);
                            }

                            logger::info("Flags - {:08X}", lightingShader->flags.underlying());
                        }
                    }
                }
                else if (childNode)
                {
                    DumpNodeChildren(childNode);
                }
                else
                {
                    logger::info("{} {} [{:.2f}, {:.2f}, {:.2f}]", object->GetRTTI()->name, object->name,
                                 object->world.translate.x, object->world.translate.y, object->world.translate.z);
                }
            }
        }
    }
}

static auto SMPDebug_PrintDetailed(const bool includeItems) -> void
{
    static std::map<hdt::ActorManager::SkeletonState, const char*> stateStrings = {
        {hdt::ActorManager::SkeletonState::e_InactiveNotInScene, "Not in scene"},
        {hdt::ActorManager::SkeletonState::e_InactiveUnseenByPlayer, "Unseen by player"},
        {hdt::ActorManager::SkeletonState::e_InactiveTooFar, "Deactivated for performance"},
        {hdt::ActorManager::SkeletonState::e_ActiveIsPlayer, "Is player character"},
        {hdt::ActorManager::SkeletonState::e_ActiveNearPlayer, "Is near player"}};

    auto skeletons = hdt::ActorManager::instance()->getSkeletons();
    std::vector<int> order(skeletons.size());
    std::ranges::iota(order, 0);
    std::ranges::sort(order, [&](const int a, const int b) { return skeletons.at(a).state < skeletons.at(b).state; });

    for (const int i : order)
    {
        auto& skeleton = skeletons[i];

        RE::TESObjectREFR* skelOwner = nullptr;
        const RE::TESFullName* ownerName = nullptr;

        if (skeleton.skeleton->GetUserData())
        {
            skelOwner = skeleton.skeleton->GetUserData();
            if (skelOwner->GetBaseObject())
            {
                ownerName = skyrim_cast<RE::TESFullName*>(skelOwner->GetBaseObject());
            }
        }

        RE::ConsoleLog::GetSingleton()->Print(
            "[HDT-SMP] %s skeleton - owner %s (refr formid %08x, base formid %08x) - %s",
            skeleton.state > hdt::ActorManager::SkeletonState::e_SkeletonActive ? "active" : "inactive",
            ownerName ? ownerName->GetFullName() : "unk_name", skelOwner ? skelOwner->formID : 0x00000000,
            skelOwner && skelOwner->GetBaseObject() ? skelOwner->GetBaseObject()->formID : 0x00000000,
            stateStrings[skeleton.state]);

        if (includeItems)
        {
            for (const auto& armor : skeleton.getArmors())
            {
                RE::ConsoleLog::GetSingleton()->Print(
                    "[HDT-SMP] -- tracked armor addon %s, %s", armor.armorWorn->name.c_str(),
                    armor.state() != hdt::ActorManager::ItemState::e_NoPhysics
                    ? armor.state() == hdt::ActorManager::ItemState::e_Active
                    ? "has active physics system"
                    : "has inactive physics system"
                    : "has no physics system");

                if (armor.state() != hdt::ActorManager::ItemState::e_NoPhysics)
                {
                    for (const auto& mesh : armor.meshes())
                    {
                        RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] ---- has collision mesh %s",
                                                              mesh->m_name.c_str());
                    }
                }
            }

            if (skeleton.head.headNode)
            {
                for (const auto& headPart : skeleton.head.headParts)
                {
                    RE::ConsoleLog::GetSingleton()->Print(
                        "[HDT-SMP] -- tracked headpart %s, %s", headPart.headPart->name.c_str(),
                        headPart.state() != hdt::ActorManager::ItemState::e_NoPhysics
                        ? headPart.state() == hdt::ActorManager::ItemState::e_Active
                        ? "has active physics system"
                        : "has inactive physics system"
                        : "has no physics system");

                    if (headPart.state() != hdt::ActorManager::ItemState::e_NoPhysics)
                    {
                        for (const auto& mesh : headPart.meshes())
                        {
                            RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] ---- has collision mesh %s",
                                                                  mesh->m_name.c_str());
                        }
                    }
                }
            }
        }
    }
}

static auto SMPDebug_Execute(const RE::SCRIPT_PARAMETER* a_paramInfo, RE::SCRIPT_FUNCTION::ScriptData* a_scriptData,
                             RE::TESObjectREFR* a_thisObj, RE::TESObjectREFR* a_containingObj, RE::Script* a_scriptObj,
                             RE::ScriptLocals* a_locals, [[maybe_unused]] double& a_result, uint32_t& a_opcodeOffsetPtr)
    -> bool
{
    std::string buffer;
    buffer.resize(MAX_PATH);
    std::string buffer2;
    buffer2.resize(MAX_PATH);

    if (!RE::Script::ParseParameters(a_paramInfo, a_scriptData, a_opcodeOffsetPtr, a_thisObj, a_containingObj,
                                     a_scriptObj, a_locals, buffer.c_str(), buffer2.c_str()))
    {
        return false;
    }

    logger::debug("SMPCommand: {} {}"sv, buffer, buffer2);

    //if (boost::beast::iequals(buffer, "reset"))
    if (_strnicmp(buffer.c_str(), "reset", MAX_PATH) == 0)
    {
        logger::debug("smp reset: reloading config and resetting physics world"sv);
        RE::ConsoleLog::GetSingleton()->Print("running full smp reset");
        hdt::loadConfig();
        hdt::SkyrimPhysicsWorld::get()->resetTransformsToOriginal();
        const RE::MenuOpenCloseEvent e{.menuName = {}, .opening = false, .pad09 = {}, .pad0A = {}, .pad0C = {}};
        hdt::ActorManager::instance()->ProcessEvent(&e, nullptr);
        hdt::SkyrimPhysicsWorld::get()->resetSystems();
        return true;
    }

    //if (boost::beast::iequals(buffer, "dumptree"))
    if (_strnicmp(buffer.c_str(), "dumptree", MAX_PATH) == 0)
    {
        if (a_thisObj)
        {
            RE::ConsoleLog::GetSingleton()->Print("dumping targeted reference's node tree");
            DumpNodeChildren(a_thisObj->Get3D1(false));
        }
        else
        {
            RE::ConsoleLog::GetSingleton()->Print("error: you must target a reference to dump their node tree");
        }

        return true;
    }

    //if (boost::beast::iequals(buffer, "detail"))
    if (_strnicmp(buffer.c_str(), "detail", MAX_PATH) == 0)
    {
        SMPDebug_PrintDetailed(true);
        return true;
    }

    //if (boost::beast::iequals(buffer, "list"))
    if (_strnicmp(buffer.c_str(), "list", MAX_PATH) == 0)
    {
        SMPDebug_PrintDetailed(false);
        return true;
    }

    //if (boost::beast::iequals(buffer, "on"))
    if (_strnicmp(buffer.c_str(), "on", MAX_PATH) == 0)
    {
        hdt::SkyrimPhysicsWorld::get()->disabled = false;
        {
            RE::ConsoleLog::GetSingleton()->Print("HDT-SMP enabled");
        }
        return true;
    }

    //if (boost::beast::iequals(buffer, "off"))
    if (_strnicmp(buffer.c_str(), "off", MAX_PATH) == 0)
    {
        hdt::SkyrimPhysicsWorld::get()->disabled = true;
        {
            RE::ConsoleLog::GetSingleton()->Print("HDT-SMP disabled");
        }
        return true;
    }

    //if (boost::beast::iequals(buffer, "QueryOverride"))
    if (_strnicmp(buffer.c_str(), "QueryOverride", MAX_PATH) == 0)
    {
        RE::ConsoleLog::GetSingleton()->Print(
            hdt::Override::OverrideManager::GetSingleton()->queryOverrideData().c_str());
        return true;
    }

    const auto skeletons = hdt::ActorManager::instance()->getSkeletons();

    size_t activeSkeletons = 0;
    size_t armors = 0;
    size_t headParts = 0;
    size_t activeArmors = 0;
    size_t activeHeadParts = 0;
    size_t activeCollisionMeshes = 0;

    for (auto skeleton : skeletons)
    {
        if (skeleton.state > hdt::ActorManager::SkeletonState::e_SkeletonActive)
        {
            activeSkeletons++;
        }

        for (const auto& armor : skeleton.getArmors())
        {
            armors++;

            if (armor.state() == hdt::ActorManager::ItemState::e_Active)
            {
                activeArmors++;

                activeCollisionMeshes += armor.meshes().size();
            }
        }

        if (skeleton.head.headNode)
        {
            for (const auto& headpart : skeleton.head.headParts)
            {
                headParts++;

                if (headpart.state() == hdt::ActorManager::ItemState::e_Active)
                {
                    activeHeadParts++;

                    activeCollisionMeshes += headpart.meshes().size();
                }
            }
        }
    }

    RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] tracked skeletons: %d", skeletons.size());
    RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] active skeletons: %d", activeSkeletons);
    RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] tracked armor addons: %d", armors);
    RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] tracked head parts: %d", headParts);
    RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] active armor addons: %d", activeArmors);
    RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] active head parts: %d", activeHeadParts);
    RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] active collision meshes: %d", activeCollisionMeshes);
    return true;
}

namespace
{
    auto InitializeLog() -> void
    {
#ifndef NDEBUG
        auto sink = std::make_shared<spdlog::sinks::msvc_sink_mt>();
#else
        auto path = logger::log_directory();
        if (!path)
        {
            util::report_and_fail("Failed to find standard logging directory"sv);
        }

        *path /= fmt::format("{}.log"sv, Plugin::NAME);
        auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path->string(), true);
#endif
        //
        const auto level = static_cast<spdlog::level::level_enum>(hdt::g_logLevel);

        //
        auto log = std::make_shared<spdlog::logger>("global log"s, std::move(sink));
        log->set_level(level);
        log->flush_on(level);

        spdlog::set_default_logger(std::move(log));
        spdlog::set_pattern("[%H:%M:%S.%e] [%L] %v"s);
    }
} // namespace

static auto MessageHandler(SKSE::MessagingInterface::Message* a_msg) -> void
{
    switch (a_msg->type)
    {
    case SKSE::MessagingInterface::kInputLoaded:
        Events::Register();
        break;
    case SKSE::MessagingInterface::kSaveGame:
    {
        auto data = hdt::Override::OverrideManager::GetSingleton()->Serialize();
        if (!data.str().empty())
        {
            std::string save_name = static_cast<char*>(a_msg->data);
            std::ofstream ofs("Data/SKSE/Plugins/hdtOverrideSaves/" + save_name + ".dhdt", std::ios::out);
            if (ofs && ofs.is_open())
            {
                ofs << data.str();
            }
        }
    }
    break;
    case SKSE::MessagingInterface::kPreLoadGame:
    {
        std::string save_name = static_cast<char*>(a_msg->data);
        save_name = save_name.substr(0, save_name.find_last_of('.'));

        std::ifstream ifs("Data/SKSE/Plugins/hdtOverrideSaves/" + save_name + ".dhdt", std::ios::in);
        if (ifs && ifs.is_open())
        {
            std::stringstream data;
            data << ifs.rdbuf();
            hdt::Override::OverrideManager::GetSingleton()->Deserialize(data);
        }
    }
    break;
    case SKSE::MessagingInterface::kPostPostLoad:
    {
        hdt::g_pluginInterface.onPostPostLoad();
        checkOldPlugins();
        Hooks::Install();
    }
    break;
    }
}

extern "C" DLLEXPORT auto SKSEAPI SKSEPlugin_Query(const SKSE::QueryInterface* a_skse, SKSE::PluginInfo* a_info) -> bool
{
    a_info->infoVersion = SKSE::PluginInfo::kVersion;
    a_info->name = Plugin::NAME.data();
    a_info->version = Plugin::VERSION.pack();

    if (a_skse->IsEditor())
    {
        logger::critical("Loaded in editor, marking as incompatible"sv);
        return false;
    }

    const auto ver = a_skse->RuntimeVersion();
    if (REL::Module::IsSE() && ver < SKSE::RUNTIME_SSE_1_5_39 || REL::Module::IsVR() && ver < SKSE::RUNTIME_LATEST_VR)
    {
        logger::critical(FMT_STRING("Unsupported runtime version {}"), ver.string());
        return false;
    }

    return true;
}

extern "C" DLLEXPORT constinit auto SKSEPlugin_Version = []()
{
    SKSE::PluginVersionData v;

    v.PluginVersion(Plugin::VERSION);
    v.PluginName(Plugin::NAME);
    v.UsesAddressLibrary();
    v.CompatibleVersions(
        {SKSE::RUNTIME_SSE_LATEST_SE, SKSE::RUNTIME_SSE_LATEST, SKSE::RUNTIME_1_6_1179, SKSE::RUNTIME_LATEST_VR});
    v.UsesNoStructs();

    return v;
}();

extern "C" DLLEXPORT auto SKSEAPI SKSEPlugin_Load(const SKSE::LoadInterface* a_skse) -> bool
{
#ifndef NDEBUG
    const auto start = std::chrono::high_resolution_clock::now();

    while (!IsDebuggerPresent())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // break after 15 seconds of idle.
        if (std::chrono::high_resolution_clock::now() - start > std::chrono::seconds(15))
        {
            break;
        }
    }
#endif

    SKSE::Init(a_skse);

    hdt::loadConfig();

    InitializeLog();

    if constexpr (Plugin::BUILD_INFO.empty())
    {
        logger::critical("{} v{} ({})"sv, Plugin::NAME, Plugin::VERSION.string(), Plugin::AVX_VARIANT);
    }
    else
    {
        logger::critical("{} v{}-{} ({})"sv, Plugin::NAME, Plugin::VERSION.string(), Plugin::BUILD_INFO,
                         Plugin::AVX_VARIANT);
    }

    hdt::logConfig();

    const auto messaging = SKSE::GetMessagingInterface();
    if (!messaging->RegisterListener("SKSE", MessageHandler))
    {
        return false;
    }

    //
    Events::Sources::FrameEventSource::GetSingleton()->AddEventSink(hdt::ActorManager::instance());
    Events::Sources::FrameEventSource::GetSingleton()->AddEventSink(hdt::SkyrimPhysicsWorld::get());

    //
    Events::Sources::FrameSyncEventSource::GetSingleton()->AddEventSink(hdt::SkyrimPhysicsWorld::get());

    //
    Events::Sources::ShutdownEventEventSource::GetSingleton()->AddEventSink(hdt::ActorManager::instance());
    Events::Sources::ShutdownEventEventSource::GetSingleton()->AddEventSink(hdt::SkyrimPhysicsWorld::get());

    //
    Events::Sources::ArmorAttachEventSource::GetSingleton()->AddEventSink(hdt::ActorManager::instance());

    //
    Events::Sources::ArmorDetachEventSource::GetSingleton()->AddEventSink(hdt::ActorManager::instance());

    //
    Events::Sources::SkinSingleHeadGeometryEventSource::GetSingleton()->AddEventSink(hdt::ActorManager::instance());

    //
    Events::Sources::SkinAllHeadGeometryEventSource::GetSingleton()->AddEventSink(hdt::ActorManager::instance());

    //
    SKSE::GetCameraEventSource()->AddEventSink(hdt::SkyrimPhysicsWorld::get());

    //
    hdt::g_pluginInterface.init(a_skse);

    //
    const auto unusedCommand = RE::SCRIPT_FUNCTION::LocateConsoleCommand("ShowRenderPasses");
    if (unusedCommand)
    {
        static RE::SCRIPT_PARAMETER params[1];
        params[0].paramType = RE::SCRIPT_PARAM_TYPE::kChar;
        params[0].paramName = "String (optional)";
        params[0].optional = true;

        unusedCommand->functionName = "SMPDebug";
        unusedCommand->shortName = "smp";
        unusedCommand->helpString = "smp <reset>";
        unusedCommand->referenceFunction = false;
        unusedCommand->numParams = 1;
        unusedCommand->params = params;
        unusedCommand->executeFunction = SMPDebug_Execute;
        unusedCommand->editorFilter = false;
    }

    //
    hdt::papyrus::RegisterAllFunctions(SKSE::GetPapyrusInterface());

    return true;
}
