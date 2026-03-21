#include "ActorManager.h"

#include <algorithm>
#include <cstdint>
#include <fmt/format.h>
#include <memory>
#include <mutex>
#include <ranges>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <LinearMath/btScalar.h>
#include <RE/A/Actor.h>
#include <RE/B/BGSBipedObjectForm.h>
#include <RE/B/BSContainer.h>
#include <RE/B/BSTEvent.h>
#include <RE/B/BSTSmartPointer.h>
#include <RE/C/ConsoleLog.h>
#include <RE/E/ExtraContainerChanges.h>
#include <RE/E/ExtraWorn.h>
#include <RE/E/ExtraWornLeft.h>
#include <RE/I/InventoryEntryData.h>
#include <RE/M/MenuOpenCloseEvent.h>
#include <RE/N/NiAVObject.h>
#include <RE/N/NiCloningProcess.h>
#include <RE/N/NiNode.h>
#include <RE/N/NiPoint3.h>
#include <RE/N/NiStream.h>
#include <RE/P/PlayerCamera.h>
#include <RE/RTTI.h>
#include <RE/T/TESBoundObject.h>
#include <RE/T/TESNPC.h>
#include <REL/ID.h>
#include <REL/Relocation.h>
#include <SKSE/Logger.h>

#include "Events.h"
#include "NetImmerseUtils.h"
#include "PCH.h"
#include "WeatherManager.h"
#include "dhdtOverrideManager.h"
#include "hdtDefaultBBP.h"
#include "hdtSkinnedMesh/hdtBulletHelper.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBody.h"
#include "hdtSkyrimPhysicsWorld.h"
#include "hdtSkyrimSystem.h"

namespace hdt
{
    // NiAVObject* Actor::CalculateLOS_1405FD2C0(Actor *aActor, NiPoint3 *aTargetPosition, NiPoint3 *aRayHitPosition,
    // float aViewCone) Used to ray cast from the actor. Will return nonNull if it hits something with position at
    // aTargetPosition. Pass in 2pi to aViewCone to ignore LOS of actor.

    using T_Actor_CalculateLOS = RE::NiAVObject* (*)(RE::Actor* aActor, RE::NiPoint3* aTargetPosition,
                                                     RE::NiPoint3* aRayHitPosition, float aViewCone);
    using T_TESNPC_GetFaceGeomPath = bool (*)(RE::TESNPC* a_npc, char* a_buf);
    using T_NiStream_constructor = RE::NiStream* (*)(RE::NiStream*);
    using T_NiStream_deconstructor = RE::NiStream* (*)(RE::NiStream*);

    REL::Relocation<T_Actor_CalculateLOS> Actor_CalculateLOS{REL::VariantID(36754, 37770, 0x0605B10)}; // 0x5FD2C0
    REL::Relocation<T_TESNPC_GetFaceGeomPath> TESNPC_GetFaceGeomPath{
        REL::VariantID(24222, 24726, 0x0372B30)}; // 0x363210
    REL::Relocation<T_NiStream_constructor> NiStream_constructor{REL::VariantID(68971, 70324, 0x0C9EC40)}; // 0xC59690
    REL::Relocation<T_NiStream_deconstructor> NiStream_deconstructor{
        REL::VariantID(68972, 70325, 0x0C9EEA0)}; // 0xC598F0

    static auto IsHair(RE::TESBoundObject* a_ref) -> bool
    {
        if (a_ref)
        {
            const auto bipedForm = skyrim_cast<RE::BGSBipedObjectForm*>(a_ref);
            if (bipedForm)
            {
                return bipedForm->bipedModelData.bipedObjectSlots.any(RE::BIPED_MODEL::BipedObjectSlot::kHair,
                                                                      RE::BIPED_MODEL::BipedObjectSlot::kLongHair);
            }
        }

        return false;
    };

    class HairVisitor // public RE::InventoryChanges::IItemChangeVisitor
    {
    public:
        HairVisitor(RE::TESBoundObject*& a_dstObject) :
            _object(a_dstObject) {}

        ~HairVisitor() = default;

        auto Visit(const RE::InventoryEntryData* a_entryData) -> RE::BSContainer::ForEachResult
        {
            if (a_entryData && a_entryData->extraLists && IsHair(a_entryData->object))
            {
                for (const auto it : *a_entryData->extraLists)
                {
                    if (it)
                    {
                        if (it->HasType<RE::ExtraWorn>() || it->HasType<RE::ExtraWornLeft>())
                        {
                            _object = a_entryData->object;
                            return RE::BSContainer::ForEachResult::kStop;
                        }
                    }
                }
            }

            //
            return RE::BSContainer::ForEachResult::kContinue;
        }

    private:
        RE::TESBoundObject* _object;
    };

    static auto isFirstPersonSkeleton(RE::NiNode* npc) -> bool
    {
        if (!npc)
        {
            return false;
        }

        return findNode(npc, "Camera1st [Cam1]") ? true : false;
    }

    static auto getNpcNode(RE::NiNode* skeleton) -> RE::NiNode*
    {
        // TODO: replace this with a generic skeleton fixing configuration option
        // hardcode an exception for lurker skeletons because they are made incorrectly
        auto shouldFix = false;
        if (skeleton->GetUserData() && skeleton->GetUserData()->GetObjectReference())
        {
            const auto npcForm = skyrim_cast<RE::TESNPC*>(skeleton->GetUserData()->GetObjectReference());
            if (npcForm && npcForm->race &&
                (std::string(npcForm->race->skeletonModels[0].GetModel()) !=
                    R"(Actors\DLC02\BenthicLurker\Character Assets\skeleton.nif)"))
            {
                shouldFix = true;
            }
        }

        return findNode(skeleton, shouldFix ? "NPC Root [Root]" : "NPC");
    }

    auto ActorManager::instance() -> ActorManager*
    {
        static ActorManager s;
        return std::addressof(s);
    }

    auto ActorManager::ProcessEvent(const Events::ArmorAttachEvent* e, RE::BSTEventSource<Events::ArmorAttachEvent>*)
        -> RE::BSEventNotifyControl
    {
        // No armor is ever attached to a lurker skeleton, thus we don't need to test.
        if (e->skeleton == nullptr || !findNode(e->skeleton, "NPC"))
        {
            return RE::BSEventNotifyControl::kContinue;
        }

        std::scoped_lock l(m_lock);
        if (m_shutdown)
        {
            return RE::BSEventNotifyControl::kContinue;
        }

        fixArmorNameMaps();

        auto& skeleton = getSkeletonData(e->skeleton);
        if (e->hasAttached)
        {
            // Check override data for current armoraddon
            if (e->skeleton->GetUserData())
            {
                const auto actor_formID = e->skeleton->GetUserData()->formID;
                if (actor_formID)
                {
                    const std::string physics_file_path_override =
                        Override::OverrideManager::GetSingleton()->checkOverride(
                            actor_formID, skeleton.getArmors().back().physicsFile.first);
                    if (!physics_file_path_override.empty())
                    {
                        RE::ConsoleLog::GetSingleton()->Print("[DynamicHDT] -- ArmorAddon %s is overridden ",
                                                              e->attachedNode->name.c_str());
                        skeleton.getArmors().back().physicsFile.first = physics_file_path_override;
                    }
                }
            }

            skeleton.attachArmor(e->armorModel, e->attachedNode);
        }
        else
        {
            skeleton.addArmor(e->armorModel);
        }

        return RE::BSEventNotifyControl::kContinue;
    }

    auto ActorManager::ProcessEvent(const Events::ArmorDetachEvent* e, RE::BSTEventSource<Events::ArmorDetachEvent>*)
        -> RE::BSEventNotifyControl
    {
        if (!e->actor || !e->hasDetached || !instance()->m_disableSMPHairWhenWigEquipped)
        {
            return RE::BSEventNotifyControl::kContinue;
        }

        std::scoped_lock l(m_lock);
        if (m_shutdown)
        {
            return RE::BSEventNotifyControl::kContinue;
        }

        fixArmorNameMaps();

        Skeleton* s = get3rdPersonSkeleton(e->actor);
        setHeadActiveIfNoHairArmor(e->actor, s);

        return RE::BSEventNotifyControl::kContinue;
    }

    // @brief This happens on a closing RaceSex menu, and on 'smp reset'.
    auto ActorManager::ProcessEvent(const RE::MenuOpenCloseEvent*, RE::BSTEventSource<RE::MenuOpenCloseEvent>*)
        -> RE::BSEventNotifyControl
    {
        // The ActorManager members are protected from parallel events by ActorManager.m_lock.
        std::scoped_lock l(m_lock);
        if (m_shutdown)
        {
            return RE::BSEventNotifyControl::kContinue;
        }

        logger::debug("Processing MenuOpenCloseEvent.");

        fixArmorNameMaps();

        setSkeletonsActive();

        for (auto& i : m_skeletons)
        {
            i.reloadMeshes();
        }

        return RE::BSEventNotifyControl::kContinue;
    }

    auto ActorManager::ProcessEvent(const Events::FrameEvent*, RE::BSTEventSource<Events::FrameEvent>*)
        -> RE::BSEventNotifyControl
    {
        std::scoped_lock l(m_lock);

        fixArmorNameMaps();

        setSkeletonsActive(true);

        return RE::BSEventNotifyControl::kContinue;
    }

    auto ActorManager::ProcessEvent(const Events::ShutdownEvent*, RE::BSTEventSource<Events::ShutdownEvent>*)
        -> RE::BSEventNotifyControl
    {
        m_shutdown = true;
        std::scoped_lock l(m_lock);

        m_skeletons.clear();

        return RE::BSEventNotifyControl::kContinue;
    }

    auto ActorManager::ProcessEvent(const Events::SkinSingleHeadGeometryEvent* e,
                                    RE::BSTEventSource<Events::SkinSingleHeadGeometryEvent>*)
        -> RE::BSEventNotifyControl
    {
        // This case never happens to a lurker skeleton, thus we don't need to test.
        const auto npc = findNode(e->skeleton, "NPC");
        if (npc == nullptr)
        {
            return RE::BSEventNotifyControl::kContinue;
        }

        std::scoped_lock l(m_lock);
        if (m_shutdown)
        {
            return RE::BSEventNotifyControl::kContinue;
        }

        fixArmorNameMaps();

        auto& skeleton = getSkeletonData(e->skeleton);
        skeleton.npc = hdt::make_nismart(getNpcNode(e->skeleton));

        skeleton.processGeometry(e->headNode, e->geometry);

        const auto headPartIter = std::ranges::find_if(
            skeleton.head.headParts, [e](const Head::HeadPart& p) { return p.headPart == e->geometry; });

        if (headPartIter != skeleton.head.headParts.end())
        {
            if (headPartIter->origPartRootNode)
            {
                logger::debug("Renaming nodes in original part {} back.", headPartIter->origPartRootNode->name);

                for (const auto& entry : skeleton.head.renameMap)
                {
                    // This case never happens to a lurker skeleton, thus we don't need to test.
                    const auto node = findNode(headPartIter->origPartRootNode.get(), entry.second);
                    if (node)
                    {
                        logger::debug("Rename node {} -> {}.", entry.second.c_str(), entry.first.c_str());
                        node->name = entry.first;
                    }
                }
            }
            headPartIter->origPartRootNode = nullptr;
        }

        if (!skeleton.head.isFullSkinning)
        {
            skeleton.scanHead();
        }

        return RE::BSEventNotifyControl::kContinue;
    }

    auto ActorManager::ProcessEvent(const Events::SkinAllHeadGeometryEvent* e,
                                    RE::BSTEventSource<Events::SkinAllHeadGeometryEvent>*) -> RE::BSEventNotifyControl
    {
        // This case never happens to a lurker skeleton, thus we don't need to test.
        const auto npc = findNode(e->skeleton, "NPC");
        if (!npc)
        {
            return RE::BSEventNotifyControl::kContinue;
        }

        std::scoped_lock l(m_lock);
        if (m_shutdown)
        {
            return RE::BSEventNotifyControl::kContinue;
        }

        fixArmorNameMaps();

        auto& skeleton = getSkeletonData(e->skeleton);
        skeleton.npc = hdt::make_nismart(npc);
        if (e->skeleton->GetUserData())
        {
            skeleton.skeletonOwner.reset(e->skeleton->GetUserData());
        }

        if (e->hasSkinned)
        {
            skeleton.scanHead();
            skeleton.head.isFullSkinning = false;
            if (skeleton.head.npcFaceGeomNode)
            {
                logger::debug("NPC face geometry no longer needed, clearing reference.");
                skeleton.head.npcFaceGeomNode = nullptr;
            }
        }
        else
        {
            skeleton.head.isFullSkinning = true;
        }

        return RE::BSEventNotifyControl::kContinue;
    }

    auto ActorManager::armorPrefix(IDType id) -> std::string
    {
        return fmt::format("hdtSSEPhysics_AutoRename_Armor_{:08X}", id);
    }

    auto ActorManager::headPrefix(IDType id) -> std::string
    {
        return fmt::format("hdtSSEPhysics_AutoRename_Head_{:08X} ", id);
    }

    auto ActorManager::fixArmorNameMaps() -> void
    {
        auto& skeletons = instance()->getSkeletons();
        for (auto& skeleton : skeletons)
        {
            if (skeleton.mustFixOneArmorMap)
            {
                auto& armors = skeleton.getArmors();
                for (auto& armor : armors)
                {
                    if (armor.mustFixNameMap)
                    {
                        if (armor.armorWorn)
                        {
                            if (!armor.armorWorn->name.empty())
                            {
                                std::string armorNewMeshName(armor.armorWorn->name);
                                if (!armorNewMeshName.empty() && armor.armorCurrentMeshName != armorNewMeshName)
                                {
                                    auto& armorNameMap = armor.physicsFile.second;
                                    DefaultBBP::NameMap_t tempNameMap;
                                    for (auto& [setName, set] : armorNameMap)
                                    {
                                        // ... and we found the old mesh name in the armor nameMap,...
                                        if (armor.armorCurrentMeshName == setName)
                                        {
                                            // We add the new mesh name to the list of mesh names for the original mesh
                                            // name (sic).
                                            set.insert({armorNewMeshName});
                                            // We plan a new entry in the armor nameMap.
                                            tempNameMap.insert({armorNewMeshName, {armorNewMeshName}});
                                            // This armor is fixed.
                                            armor.mustFixNameMap = false;
                                            armor.armorCurrentMeshName = armorNewMeshName;
                                        }
                                    }
                                    // We add the planned entries.
                                    for (auto& [setName, set] : tempNameMap)
                                    {
                                        armorNameMap.insert({setName, set});
                                    }
                                }
                            }
                        }
                    }
                }
                skeleton.mustFixOneArmorMap = false;
            }
        }
    }

    // @brief To avoid calculating headparts when they're hidden by a wig,
    // we mark the head as not active when there is an armor on hair or long hair slots.
    // We do this during the attach/detach armor events, and on the events leading to scanning the head.
    // Then when checking which skeletons are active to calculate the frame,
    // we only allow the activation of headparts that are on active heads.
    // @param Actor * actor is expected not null.
    auto ActorManager::setHeadActiveIfNoHairArmor([[maybe_unused]] RE::Actor* actor,
                                                  [[maybe_unused]] Skeleton* skeleton) -> void
    {
        RE::TESBoundObject* ref = nullptr;

        //
        const RE::ExtraContainerChanges* extraContainerchanges =
            actor->extraList.GetByType<RE::ExtraContainerChanges>();
        if (extraContainerchanges && extraContainerchanges->changes && extraContainerchanges->changes->entryList)
        {
            //
            HairVisitor visitor(ref);

            //
            for (const auto it : *extraContainerchanges->changes->entryList)
            {
                if (visitor.Visit(it) == RE::BSContainer::ForEachResult::kStop)
                {
                    break;
                }
            }
        }

        //
        if (skeleton)
        {
            skeleton->head.isActive = ref == nullptr;
        }
    }

    inline auto ActorManager::getCameraNode() -> RE::NiNode*
    {
        return RE::PlayerCamera::GetSingleton()->cameraRoot.get();
    }

    // @brief This function is called by different events, with different locking needs, and is therefore extracted from
    // the events.
    auto ActorManager::setSkeletonsActive(const bool updateMetrics) -> void
    {
        if (m_shutdown)
        {
            return;
        }

        // We get the player character and its cell.
        // TODO Isn't there a more performing way to find the PC?? A singleton? And if it's the right way, why isn't it
        // in utils functions?
        const auto& playerCharacter =
            std::ranges::find_if(m_skeletons, [](const Skeleton& s) { return s.isPlayerCharacter(); });
        const auto playerCell = (playerCharacter != m_skeletons.end() && playerCharacter->skeleton->parent)
            ? playerCharacter->skeleton->parent->parent
            : nullptr;

        const auto cameraNode = getCameraNode();
        if (!cameraNode)
        {
            return;
        }

        // We get the camera, its position and orientation.
        const auto cameraTransform = cameraNode->world;
        const auto cameraPosition = cameraTransform.translate;
        const auto cameraOrientation =
            cameraTransform.rotate * RE::NiPoint3(0., 1., 0.); // The camera matrix is relative to the world.
        this->m_cameraPositionDuringFrame = cameraPosition;

        std::ranges::for_each(
            m_skeletons, [&](Skeleton& skel)
            {
                skel.calculateDistanceAndOrientationDifferenceFromSource(cameraPosition, cameraOrientation);
            });

        // We sort the skeletons depending on the angle and distance.
        std::ranges::sort(m_skeletons,
                          [](auto&& a_lhs, auto&& a_rhs)
                          {
                              auto cr = a_rhs.m_cosAngleFromCameraDirectionTimesSkeletonDistance;
                              auto cl = a_lhs.m_cosAngleFromCameraDirectionTimesSkeletonDistance;
                              auto dr = a_rhs.m_distanceFromCamera2;
                              auto dl = a_lhs.m_distanceFromCamera2;
                              return
                                  // If one of the skeletons is at distance zero (1st person player) from the camera
                                  (btFuzzyZero(dl) || btFuzzyZero(dr))
                                  // then it is first.
                                  ? (dl < dr)
                                  // If one of the skeletons is exacly on the side of the camera (cos = 0)
                                  : (btFuzzyZero(cl) || btFuzzyZero(cr))
                                  // then it is last.
                                  ? abs(cl) > abs(cr)
                                  // If both are on the same side of the camera (product of cos > 0):
                                  // we want first the smallest angle (so the highest cosinus), and the smallest
                                  // distance, so we want the smallest distance / cosinus. cl = cosinus * distance,
                                  // dl = distance� => distance / cosinus = dl/cl So we want dl/cl < dr/cr.
                                  // Moreover, this test manages the case where one of the skeletons is behind the
                                  // camera and the other in front of the camera too; the one behind the camera is
                                  // last (the one with cos(angle) = cr < 0).
                                  : (dl * cr < dr * cl);
                          });

        // We set which skeletons are active and we count them.
        activeSkeletons = 0;
        for (auto& i : m_skeletons)
        {
            if (i.skeleton->_refCount == 1)
            {
                i.clear();
                i.skeleton = nullptr;
            }
            else if (i.hasPhysics && i.updateAttachedState(playerCell, activeSkeletons >= maxActiveSkeletons))
            {
                activeSkeletons++;
                // check wind obstructions
                const auto world = SkyrimPhysicsWorld::get();
                const auto wind = getWindDirection();
                if (world->m_enableWind && wind && !(btFuzzyZero(magnitude(*wind))))
                {
                    const auto owner = skyrim_cast<RE::Actor*>(i.skeletonOwner.get());
                    if (owner)
                    {
                        auto windray = *wind * -1; // reverse wind raycast to find obstruction
                        RE::NiPoint3 hitLocation;
                        // Raycast for object in direction of wind
                        const auto object = Actor_CalculateLOS(owner, &windray, &hitLocation, 6.28);
                        if (object)
                        {
                            // object found
                            auto diff = (owner->data.location - hitLocation);
                            diff.z = 0; // remove z component difference
                            const auto dist = magnitude(diff);
                            // wind is a linear reduction, with a minimum floor since objects may have a minimum
                            // distance windfactor = 0 when dist <= m_distanceForNoWind, = 1 when dist >=
                            // m_distanceForMaxWind, and is linear with dist between these 2 values.
                            const auto windFactor =
                                std::clamp((dist - world->m_distanceForNoWind) /
                                           (world->m_distanceForMaxWind - world->m_distanceForNoWind),
                                           0.f, 1.f);
                            if (!btFuzzyZero(windFactor - i.getWindFactor()))
                            {
                                logger::debug("{} blocked by {} with distance {:.2f}; setting windFactor {:.2f}.",
                                              i.name(), object->name, dist, windFactor);
                                i.updateWindFactor(windFactor);
                            }
                        }
                    }
                    else
                    {
                        logger::debug(
                            "{} is active skeleton, but failed to cast to Actor, no wind obstruction check possible.",
                            i.name());
                    }
                }
            }
        }

        m_skeletons.erase(std::ranges::remove_if(m_skeletons, [](const Skeleton& i) { return !i.skeleton; }).begin(),
                          m_skeletons.end());

        for (auto& i : m_skeletons)
        {
            i.cleanArmor();
            i.cleanHead();
        }

        const auto world = SkyrimPhysicsWorld::get();

        // We share the same doMetrics condition here and in hdtSkyrimPhysicsWorld to avoid any gap between both.
        // The evaluation is done here rather than in hdtSkyrimPhysicsWorld because this event is called first.
        world->m_doMetrics = updateMetrics && // do not do metrics on a MenuOpenCloseEvent
            !world->isSuspended() && // do not do metrics while paused
            frameCount++ % world->min_fps ==
            0; // check every min-fps frames (i.e., a stable 60 fps should wait for 1 second)

        if (world->m_doMetrics)
        {
            const auto averageProcessingTimeInMainLoop = world->m_averageSMPProcessingTimeInMainLoop;
            // 30% of processing time is in hdt per profiling;
            // Setting it higher provides more time for hdt processing and can activate more skeletons.
            const auto target_time = world->m_timeTick * world->m_percentageOfFrameTime;
            auto averageTimePerSkeletonInMainLoop = 0.f;
            if (activeSkeletons > 0)
            {
                averageTimePerSkeletonInMainLoop = averageProcessingTimeInMainLoop / activeSkeletons;
            }

            logger::trace(
                "msecs/activeSkeleton {:.2f} activeSkeletons/maxActive/total {}/{}/{} processTimeInMainLoop/targetTime "
                "{:.2f}/{:.2f}",
                averageTimePerSkeletonInMainLoop, activeSkeletons, maxActiveSkeletons, m_skeletons.size(),
                averageProcessingTimeInMainLoop, target_time);

            if (m_autoAdjustMaxSkeletons)
            {
                maxActiveSkeletons += target_time > averageProcessingTimeInMainLoop ? 2 : -2;
                // clamp the value to the m_maxActiveSkeletons value
                maxActiveSkeletons = std::clamp(maxActiveSkeletons, 1, m_maxActiveSkeletons);
                frameCount = 1;
            }
            else if (maxActiveSkeletons != m_maxActiveSkeletons)
            {
                maxActiveSkeletons = m_maxActiveSkeletons;
            }
        }
    }

    auto ActorManager::PhysicsItem::setPhysics(const RE::BSTSmartPointer<SkyrimSystem>& system, const bool active)
        -> void
    {
        clearPhysics();
        m_physics = system;
        if (active)
        {
            SkyrimPhysicsWorld::get()->addSkinnedMeshSystem(m_physics.get());
        }
    }

    auto ActorManager::PhysicsItem::clearPhysics() -> void
    {
        if (state() == ItemState::e_Active)
        {
            m_physics->m_world->removeSkinnedMeshSystem(m_physics.get());
        }
        m_physics = nullptr;
    }

    auto ActorManager::PhysicsItem::state() const -> ItemState
    {
        return m_physics ? (m_physics->m_world ? ItemState::e_Active : ItemState::e_Inactive) : ItemState::e_NoPhysics;
    }

    auto ActorManager::PhysicsItem::meshes() const -> const std::vector<RE::BSTSmartPointer<SkinnedMeshBody>>&
    {
        return m_physics->meshes();
    }

    auto ActorManager::PhysicsItem::updateActive(const bool active) const -> void
    {
        if (active && state() == ItemState::e_Inactive)
        {
            SkyrimPhysicsWorld::get()->addSkinnedMeshSystem(m_physics.get());
        }
        else if (!active && state() == ItemState::e_Active)
        {
            m_physics->m_world->removeSkinnedMeshSystem(m_physics.get());
        }
    }

    auto ActorManager::PhysicsItem::setWindFactor(const float a_windFactor) const -> void
    {
        if (m_physics)
        {
            m_physics->m_windFactor = a_windFactor;
        }
    }

    auto ActorManager::getSkeletons() -> std::vector<Skeleton>& { return m_skeletons; }

    auto ActorManager::skeletonNeedsParts(RE::NiNode* skeleton) -> bool
    {
        return !isFirstPersonSkeleton(skeleton);
        /*
        auto iter = std::find_if(m_skeletons.begin(), m_skeletons.end(), [=](Skeleton& i)
        {
            return i.skeleton == skeleton;
        });
        if (iter != m_skeletons.end())
        {
            return (iter->head.headNode == 0);
        }
        */
    }

    auto ActorManager::getSkeletonData(RE::NiNode* skeleton) -> Skeleton&
    {
        const auto iter = std::ranges::find_if(m_skeletons, [=](const Skeleton& i) { return i.skeleton == skeleton; });

        if (iter != m_skeletons.end())
        {
            return *iter;
        }

        if (!isFirstPersonSkeleton(skeleton))
        {
            const auto ownerIter = std::ranges::find_if(m_skeletons,
                                                        [=](const Skeleton& i)
                                                        {
                                                            return !isFirstPersonSkeleton(i.skeleton.get()) &&
                                                                i.skeletonOwner && skeleton->GetUserData() &&
                                                                i.skeletonOwner.get() == skeleton->GetUserData();
                                                        });

            if (ownerIter != m_skeletons.end())
            {
                logger::debug("New skeleton found for formid {:08X}.", skeleton->GetUserData()->formID);
                ownerIter->cleanHead(true);
            }
        }

        m_skeletons.emplace_back();
        m_skeletons.back().skeleton = hdt::make_nismart(skeleton);
        return m_skeletons.back();
    }

    auto ActorManager::get3rdPersonSkeleton(const RE::Actor* actor) -> Skeleton*
    {
        for (auto& i : m_skeletons)
        {
            const auto owner = skyrim_cast<RE::Actor*>(i.skeletonOwner.get());
            if (actor == owner && i.skeleton && !isFirstPersonSkeleton(i.skeleton.get()))
            {
                return &i;
            }
        }
        return 0;
    }

    auto ActorManager::Skeleton::doSkeletonMerge(RE::NiNode* dst, RE::NiNode* src, std::string_view prefix,
                                                 std::unordered_map<RE::BSFixedString, RE::BSFixedString>& map) -> void
    {
        const auto& children = src->GetChildren();

        for (const auto& i : children)
        {
            const auto srcChild = castNiNode(i.get());
            if (!srcChild)
            {
                continue;
            }

            if (srcChild->name.empty())
            {
                doSkeletonMerge(dst, srcChild, prefix, map);
                continue;
            }

            // FIXME: This was previously only in doHeadSkeletonMerge.
            // But surely non-head skeletons wouldn't have this anyway?
            if (std::string(srcChild->name.c_str()) != std::string("BSFaceGenNiNodeSkinned"))
            {
                logger::debug("Skipping facegen ninode in skeleton merge.");
                continue;
            }

            // TODO check it's not a lurker skeleton
            const auto dstChild = findNode(dst, srcChild->name);
            if (dstChild)
            {
                doSkeletonMerge(dstChild, srcChild, prefix, map);
            }
            else
            {
                dst->AttachChild(cloneNodeTree(srcChild, prefix, map), false);
            }
        }
    }

    auto ActorManager::Skeleton::cloneNodeTree(RE::NiNode* src, std::string_view prefix,
                                               std::unordered_map<RE::BSFixedString, RE::BSFixedString>& map)
        -> RE::NiNode*
    {
        //
        RE::NiCloningProcess c;

        //
        c.appendChar = '$';
        c.copyType = 1;
        c.scale = {1.0f, 1.0f, 1.0f};

        const auto ret = dynamic_cast<RE::NiNode*>(src->CreateClone(c));
        src->ProcessClone(c);

        // FIXME: cloneHeadNodeTree just did this for ret, not both. Don't know if that matters. Armor parts need it on
        // both.
        renameTree(src, prefix, map);
        renameTree(ret, prefix, map);

        return ret;
    }

    auto ActorManager::Skeleton::renameTree(RE::NiNode* root, std::string_view prefix,
                                            std::unordered_map<RE::BSFixedString, RE::BSFixedString>& map) -> void
    {
        if (!root->name.empty())
        {
            std::string newName{prefix};
            newName += root->name;
            if (map.insert(std::make_pair<RE::BSFixedString, RE::BSFixedString>(root->name.c_str(), newName)).second)
            {
                logger::debug("Rename Bone {} -> {}.", root->name, newName.c_str());
            }

            setNiNodeName(root, newName.c_str());
        }

        auto& children = root->GetChildren();
        for (const auto& i : children)
        {
            const auto child = castNiNode(i.get());
            if (child)
            {
                renameTree(child, prefix, map);
            }
        }
    }

    auto ActorManager::Skeleton::doSkeletonClean(RE::NiNode* dst, std::string_view prefix) -> void
    {
        auto& children = dst->GetChildren();

        for (uint16_t i = children.size(); i-- > 0;)
        {
            const auto child = castNiNode(children[i].get());
            if (!child)
            {
                continue;
            }

            if (prefix == std::string_view(child->name).substr(0, prefix.size()))
            {
                dst->DetachChildAt2(i);
            }
            else
            {
                doSkeletonClean(child, prefix);
            }
        }
    }

    // returns the name of the skeleton owner
    auto ActorManager::Skeleton::name() const -> std::string
    {
        if (skeleton->GetUserData() && skeleton->GetUserData()->GetObjectReference())
        {
            const auto bname = skyrim_cast<RE::TESFullName*>(skeleton->GetUserData()->GetObjectReference());
            if (bname)
            {
                return bname->GetFullName();
            }
        }
        return {};
    }

    auto ActorManager::Skeleton::addArmor(RE::NiNode* armorModel) -> void
    {
        const IDType id = !armors.empty() ? armors.back().id + 1 : 0;
        const auto prefix = armorPrefix(id);
        // FIXME we probably could simplify this by using findNode as surely we don't merge Armors with lurkers
        // skeleton?
        npc = hdt::make_nismart(getNpcNode(skeleton.get()));
        const auto physicsFile = DefaultBBP::instance()->scanBBP(armorModel);

        armors.emplace_back();
        armors.back().id = id;
        armors.back().prefix = prefix;
        armors.back().physicsFile = physicsFile;

        doSkeletonMerge(npc.get(), armorModel, prefix, armors.back().renameMap);
    }

    auto ActorManager::Skeleton::attachArmor([[maybe_unused]] RE::NiNode* armorModel, RE::NiAVObject* attachedNode)
        -> void
    {
        if (armors.empty() || armors.back().hasPhysics())
        {
            logger::trace("Not attaching armor - no record or physics already exists");
        }

        Armor& armor = armors.back();

        // The name of the attachedNode provided here will have been changed by the Skyrim exe between this event and
        // the next.
        armor.armorWorn = hdt::make_nismart(attachedNode);
        // That's why we set here the need to fix this armor in fixArmorNameMaps() (see its comment)
        // to avoid this name change breaking processes like 'smp reset' when looking for the armor name in the armor
        // nameMap.
        armor.armorCurrentMeshName = !attachedNode->name.empty() ? attachedNode->name : "";
        armor.mustFixNameMap = true;
        mustFixOneArmorMap = true;

        if (!isFirstPersonSkeleton(skeleton.get()))
        {
            // FIXME we probably could simplify this by using findNode as surely we don't attach Armors to lurkers
            // skeleton?
            auto renameMap = armor.renameMap;
            auto system = SkyrimSystemCreator().createOrUpdateSystem(getNpcNode(skeleton.get()), attachedNode,
                                                                     &armor.physicsFile, std::move(renameMap), nullptr);
            if (system)
            {
                armor.setPhysics(system, isActive);
                hasPhysics = true;
            }
        }

        if (instance()->m_disableSMPHairWhenWigEquipped && skeleton && skeleton->GetUserData())
        {
            RE::Actor* actor = RE::TESForm::LookupByID<RE::Actor>(skeleton->GetUserData()->formID);
            if (actor)
            {
                setHeadActiveIfNoHairArmor(actor, this);
            }
        }
    }

    auto ActorManager::Skeleton::cleanArmor() -> void
    {
        for (auto& i : armors)
        {
            if (!i.armorWorn)
            {
                continue;
            }

            if (i.armorWorn->parent)
            {
                continue;
            }

            i.clearPhysics();
            if (npc)
            {
                doSkeletonClean(npc.get(), i.prefix);
            }

            i.prefix = {};
        }

        armors.erase(std::ranges::remove_if(armors, [](const Armor& i) { return i.prefix.empty(); }).begin(),
                     armors.end());
    }

    auto ActorManager::Skeleton::cleanHead(const bool cleanAll) -> void
    {
        for (auto& headPart : head.headParts)
        {
            if (!headPart.headPart->parent || cleanAll)
            {
                if (cleanAll)
                {
                    logger::debug("Cleaning headpart {} due to clean all.", headPart.headPart->name);
                }
                else
                {
                    logger::debug("Headpart {} disconnected.", headPart.headPart->name);
                }

                auto renameIt = this->head.renameMap.begin();
                while (renameIt != this->head.renameMap.end())
                {
                    auto erase = false;

                    if (headPart.renamedBonesInUse.contains(renameIt->first))
                    {
                        auto findNode = this->head.nodeUseCount.find(renameIt->first);
                        if (findNode != this->head.nodeUseCount.end())
                        {
                            findNode->second -= 1;
                            logger::debug("Decrementing use count by 1, it is now {}.", findNode->second);
                            if (findNode->second <= 0)
                            {
                                logger::debug("Node no longer in use, cleaning from skeleton.");
                                const auto removeObj = findObject(npc.get(), renameIt->second);
                                if (removeObj)
                                {
                                    logger::debug("Found node {}, removing.", removeObj->name);
                                    const auto parent = removeObj->parent;
                                    if (parent)
                                    {
                                        parent->DetachChild2(removeObj);
                                    }
                                }
                                this->head.nodeUseCount.erase(findNode);
                                erase = true;
                            }
                        }
                    }

                    if (erase)
                    {
                        renameIt = this->head.renameMap.erase(renameIt);
                    }
                    else
                    {
                        ++renameIt;
                    }
                }

                headPart.headPart = nullptr;
                headPart.origPartRootNode = nullptr;
                headPart.clearPhysics();
                headPart.renamedBonesInUse.clear();
            }
        }

        head.headParts.erase(
            std::ranges::remove_if(head.headParts, [](const Head::HeadPart& i) { return !i.headPart; }).begin(),
            head.headParts.end());
    }

    auto ActorManager::Skeleton::clear() -> void
    {
        std::ranges::for_each(armors, [](Armor& armor) { armor.clearPhysics(); });
        SkyrimPhysicsWorld::get()->removeSystemByNode(npc.get());
        cleanHead();
        head.headParts.clear();
        head.headNode = nullptr;
        armors.clear();
    }

    auto ActorManager::Skeleton::calculateDistanceAndOrientationDifferenceFromSource(
        const RE::NiPoint3 sourcePosition, const RE::NiPoint3 sourceOrientation) -> void
    {
        if (isPlayerCharacter())
        {
            m_distanceFromCamera2 = 0.f;
            return;
        }

        const auto pos = position();
        if (!pos.has_value())
        {
            m_distanceFromCamera2 = std::numeric_limits<float>::max();
            return;
        }

        // We calculate the vector between camera and the skeleton feets.
        const auto camera2SkeletonVector = pos.value() - sourcePosition;
        // This is the distance (squared) between the camera and the skeleton feets.
        m_distanceFromCamera2 = camera2SkeletonVector.x * camera2SkeletonVector.x +
            camera2SkeletonVector.y * camera2SkeletonVector.y + camera2SkeletonVector.z * camera2SkeletonVector.z;
        // This is |camera2SkeletonVector|*cos(angle between both vectors)
        m_cosAngleFromCameraDirectionTimesSkeletonDistance = camera2SkeletonVector.x * sourceOrientation.x +
            camera2SkeletonVector.y * sourceOrientation.y + camera2SkeletonVector.z * sourceOrientation.z;
    }

    // Is called to print messages only
    auto ActorManager::Skeleton::checkPhysics() -> bool
    {
        hasPhysics = false;
        std::ranges::for_each(armors,
                              [this](const Armor& armor)
                              {
                                  if (armor.state() != ItemState::e_NoPhysics)
                                  {
                                      hasPhysics = true;
                                  }
                              });

        if (!hasPhysics)
        {
            std::ranges::for_each(head.headParts,
                                  [this](const Head::HeadPart& headPart)
                                  {
                                      if (headPart.state() != ItemState::e_NoPhysics)
                                      {
                                          hasPhysics = true;
                                      }
                                  });
        }

        logger::trace("{} isDrawn {}", name(), hasPhysics);

        return hasPhysics;
    }

    auto ActorManager::Skeleton::isActiveInScene() const -> bool
    {
        // TODO: do this better
        // When entering/exiting an interior, NPCs are detached from the scene but not unloaded, so we need to check two
        // levels up. This properly removes exterior cell armors from the physics world when entering an interior, and
        // vice versa.
        return skeleton->parent && skeleton->parent->parent && skeleton->parent->parent->parent;
    }

    auto ActorManager::Skeleton::isPlayerCharacter() const -> bool
    {
        constexpr uint32_t playerFormID = 0x14;
        return skeletonOwner.get() == RE::PlayerCharacter::GetSingleton() ||
            (skeleton->GetUserData() && skeleton->GetUserData()->formID == playerFormID);
    }

    auto ActorManager::Skeleton::isInPlayerView() const -> bool
    {
        // This function is called only when the skeleton isn't the player character.
        // This might change in the future; in that case this test will have to be enabled.
        // if (isPlayerCharacter())
        //	return true;

        // We always enable the skeletons that are just around the camera.
        // It's useful if for example the skeleton origin is very near, behind the camera,
        // but some parts or the skeleton are in front of the camera and need to be animated.
        const auto i = instance();
        const float minDistance = i->m_minCullingDistance;
        if (m_distanceFromCamera2 < minDistance * minDistance)
        {
            return true;
        }

        // We don't enable the skeletons behind the camera or on its side.
        if (m_cosAngleFromCameraDirectionTimesSkeletonDistance <= 0)
        {
            return false;
        }

        // We enable only the skeletons that can see the PC or the camera
        const auto owner = skyrim_cast<RE::Actor*>(this->skeletonOwner.get());
        if (owner)
        {
            RE::NiPoint3 hitLocation;
            const auto object = Actor_CalculateLOS(owner, &(i->m_cameraPositionDuringFrame), &hitLocation, 6.28);
            return object ? false : true; // If object, we hit something on the path
        }
        return true; // should never happen, a skeleton without owner?
    }

    auto ActorManager::Skeleton::position() const -> std::optional<RE::NiPoint3>
    {
        if (npc)
        {
            // This works for lurker skeletons.
            const auto rootNode = findNode(npc.get(), "NPC Root [Root]");
            if (rootNode)
            {
                return {rootNode->world.translate};
            }
        }

        return {};
    }

    auto ActorManager::Skeleton::updateWindFactor(const float a_windFactor) -> void
    {
        this->currentWindFactor = a_windFactor;
        std::ranges::for_each(armors, [=](const Armor& armor) { armor.setWindFactor(a_windFactor); });
        std::ranges::for_each(head.headParts,
                              [=](const Head::HeadPart& headPart) { headPart.setWindFactor(a_windFactor); });
    }

    auto ActorManager::Skeleton::getWindFactor() const -> float { return this->currentWindFactor; }

    auto ActorManager::Skeleton::updateAttachedState(const RE::NiNode* playerCell, const bool deactivate = false)
        -> bool
    {
        // 1- Skeletons that aren't active in any scene are always detached, unless they are in the
        // same cell as the player character (workaround for issue in Ancestor Glade).
        // 2- Player character is always attached.
        // 3- Otherwise, attach only if both the camera and this skeleton have a position,
        // the distance between them is below the threshold value,
        // and the angle difference between the camera orientation and the skeleton orientation is below the threshold
        // value.
        isActive = false;
        state = SkeletonState::e_InactiveNotInScene;

        if (deactivate)
        {
            state = SkeletonState::e_InactiveTooFar;
        }
        else if (isActiveInScene() || skeleton->parent && skeleton->parent->parent == playerCell)
        {
            if (isPlayerCharacter())
            {
                // That setting defines whether we don't set the PC skeleton as active
                // when it is in 1st person view, to avoid calculating physics uselessly.
                if (!(instance()->m_disable1stPersonViewPhysics // disabling?
                    && RE::PlayerCamera::GetSingleton()->currentState ==
                    RE::PlayerCamera::GetSingleton()->GetRuntimeData().cameraStates[0])) // 1st person view
                {
                    isActive = true;
                    state = SkeletonState::e_ActiveIsPlayer;
                }
            }
            else if (isInPlayerView())
            {
                isActive = true;
                state = SkeletonState::e_ActiveNearPlayer;
            }
            else
            {
                state = SkeletonState::e_InactiveUnseenByPlayer;
            }
        }

        // We update the activity state of armors and head parts, and add and remove SkinnedMeshSystems to these parts
        // in consequence. We set headparts as not active if the head isn't active (for example because it's hidden by a
        // wig).
        std::ranges::for_each(armors, [this](const Armor& armor) { armor.updateActive(isActive); });
        const bool isHeadActive = head.isActive;
        std::ranges::for_each(head.headParts, [isHeadActive, this](const Head::HeadPart& headPart)
        {
            headPart.updateActive(isHeadActive && isActive);
        });
        return isActive;
    }

    auto ActorManager::Skeleton::reloadMeshes() -> void
    {
        for (auto& i : armors)
        {
            i.clearPhysics();
            if (!isFirstPersonSkeleton(skeleton.get()))
            {
                auto renameMap = i.renameMap;
                auto system = SkyrimSystemCreator().createOrUpdateSystem(npc.get(), i.armorWorn.get(), &i.physicsFile,
                                                                         std::move(renameMap), nullptr);
                if (system)
                {
                    i.setPhysics(system, isActive);
                    hasPhysics = true;
                }
            }
        }
        scanHead();
    }

    auto ActorManager::Skeleton::scanHead() -> void
    {
        if (isFirstPersonSkeleton(this->skeleton.get()))
        {
            logger::debug("Not scanning head of first person skeleton.");
            return;
        }

        if (!this->head.headNode)
        {
            logger::debug("Actor has no head node.");
            return;
        }

        std::unordered_set<std::string> physicsDupes;

        if (instance()->m_disableSMPHairWhenWigEquipped && skeleton && skeleton->GetUserData())
        {
            RE::Actor* actor = RE::TESForm::LookupByID<RE::Actor>(skeleton->GetUserData()->formID);
            if (actor)
            {
                setHeadActiveIfNoHairArmor(actor, this);
            }
        }

        for (auto& headPart : this->head.headParts)
        {
            // always regen physics for all head parts
            headPart.clearPhysics();

            if (headPart.physicsFile.first.empty())
            {
                logger::debug("No physics file for headpart {}.", headPart.headPart->name);
                continue;
            }

            if (physicsDupes.contains(headPart.physicsFile.first))
            {
                logger::debug("Previous head part generated physics system for file {}, skipping.",
                              headPart.physicsFile.first.c_str());
                continue;
            }

            logger::debug("Try create system for headpart {} physics file {}.", headPart.headPart->name,
                          headPart.physicsFile.first.c_str());
            physicsDupes.insert(headPart.physicsFile.first);
            auto renameMap = this->head.renameMap;
            auto system = SkyrimSystemCreator().createOrUpdateSystem(
                npc.get(), this->head.headNode.get(), &headPart.physicsFile, std::move(renameMap), nullptr);
            if (system)
            {
                logger::debug("Success.");
                headPart.setPhysics(system, isActive);
                hasPhysics = true;
            }
        }
    }

    auto ActorManager::Skeleton::processGeometry(RE::BSFaceGenNiNode* headNode, RE::BSGeometry* geometry) -> void
    {
        if (this->head.headNode && this->head.headNode != headNode)
        {
            logger::debug("Completely new head attached to skeleton, clearing tracking.");
            for (auto& headPart : this->head.headParts)
            {
                headPart.clearPhysics();
                headPart.headPart = nullptr;
                headPart.origPartRootNode = nullptr;
            }

            this->head.headParts.clear();

            if (npc)
            {
                doSkeletonClean(npc.get(), this->head.prefix);
            }

            this->head.prefix = {};
            this->head.headNode = nullptr;
            this->head.renameMap.clear();
            this->head.nodeUseCount.clear();
        }

        // clean swapped out headparts
        cleanHead();

        this->head.headNode = hdt::make_nismart(headNode);
        ++this->head.id;
        this->head.prefix = headPrefix(this->head.id);

        const auto it = std::ranges::find_if(this->head.headParts,
                                             [geometry](const Head::HeadPart& p) { return p.headPart == geometry; });

        if (it != this->head.headParts.end())
        {
            logger::debug("Geometry is already added as head part.");
            return;
        }

        this->head.headParts.emplace_back();

        head.headParts.back().headPart = hdt::make_nismart(geometry);
        head.headParts.back().clearPhysics();

        // Skinning
        logger::debug("Skinning geometry to skeleton.");

        if (!geometry->GetGeometryRuntimeData().skinInstance ||
            !geometry->GetGeometryRuntimeData().skinInstance->skinData)
        {
            logger::error("Geometry is missing skin instance - how?");
            return;
        }

        const auto fmd = geometry->GetExtraData<RE::BSFaceGenModelExtraData>("FMD");

        RE::BSGeometry* origGeom = nullptr;
        RE::NiGeometry* origNiGeom = nullptr;

        if (fmd && fmd->m_model && fmd->m_model->modelMeshData && fmd->m_model->modelMeshData->faceNode)
        {
            logger::debug("Original part node found via facegen extra model data.");
            const auto origRootNode = fmd->m_model->modelMeshData->faceNode->AsNode();
            head.headParts.back().physicsFile = DefaultBBP::instance()->scanBBP(origRootNode);
            head.headParts.back().origPartRootNode = hdt::make_nismart(origRootNode);

            auto& children = origRootNode->GetChildren();
            for (const auto& i : children)
            {
                if (i)
                {
                    const auto geo = i->AsGeometry();

                    if (geo)
                    {
                        origGeom = geo;
                        break;
                    }
                }
            }
        }
        else
        {
            logger::debug("No facegen extra model data available, loading original facegeometry.");
            if (!head.npcFaceGeomNode)
            {
                if (skeleton->GetUserData() && skeleton->GetUserData()->GetObjectReference())
                {
                    auto skeletonNpc = skyrim_cast<RE::TESNPC*>(skeleton->GetUserData()->GetObjectReference());
                    if (skeletonNpc)
                    {
                        char filePath[MAX_PATH];
                        if (TESNPC_GetFaceGeomPath(skeletonNpc, filePath))
                        {
                            logger::debug("Loading facegeometry from path {}.", filePath);
                            uint8_t niStreamMemory[sizeof(RE::NiStream)] = {};
                            auto niStream = reinterpret_cast<RE::NiStream*>(niStreamMemory);
                            NiStream_constructor(niStream);

                            RE::BSResourceNiBinaryStream binaryStream(filePath);
                            if (!binaryStream.good())
                            {
                                logger::error("Somehow NPC facegeometry was not found.");
                                NiStream_deconstructor(niStream);
                            }
                            else
                            {
                                niStream->Load1(&binaryStream);
                                if (niStream->topObjects[0])
                                {
                                    const auto rootFadeNode = niStream->topObjects[0]->AsFadeNode();
                                    if (rootFadeNode)
                                    {
                                        logger::debug("NPC facegeometry root fadeNode found.");
                                        head.npcFaceGeomNode = hdt::make_nismart(rootFadeNode);
                                    }
                                    else
                                    {
                                        logger::debug("NPC facegeometry root wasn't fadeNode as expected.");
                                    }
                                }
                                NiStream_deconstructor(niStream);
                            }
                        }
                    }
                }
            }
            else
            {
                logger::debug("Using cached facegeometry.");
            }
            if (head.npcFaceGeomNode)
            {
                head.headParts.back().physicsFile = DefaultBBP::instance()->scanBBP(head.npcFaceGeomNode.get());
                const auto obj = findObject(head.npcFaceGeomNode.get(), geometry->name);
                if (obj)
                {
                    const auto ob = obj->AsGeometry();
                    if (ob)
                    {
                        origGeom = ob;
                    }
                    else
                    {
                        const auto on = obj->AsNiGeometry();
                        if (on)
                        {
                            origNiGeom = on;
                        }
                    }
                }
            }
        }

        auto hasMerged = false;
        auto hasRenames = false;

        for (uint32_t boneIdx = 0; boneIdx < geometry->GetGeometryRuntimeData().skinInstance->skinData->bones;
             boneIdx++)
        {
            RE::BSFixedString boneName("");

            // skin the way the game does via FMD
            if (boneIdx <= 7)
            {
                if (fmd)
                {
                    boneName = fmd->bones[boneIdx];
                }
            }

            if (boneName.empty())
            {
                if (origGeom)
                {
                    boneName = origGeom->GetGeometryRuntimeData().skinInstance->bones[boneIdx]->name;
                }
                else if (origNiGeom)
                {
                    boneName = origNiGeom->GetRuntimeData().spSkinInstance->bones[boneIdx]->name;
                }
            }

            auto renameIt = this->head.renameMap.find(boneName.c_str());
            if (renameIt != this->head.renameMap.end())
            {
                logger::debug("Found renamed bone {} -> {}.", boneName, renameIt->second.c_str());
                boneName = renameIt->second;
                hasRenames = true;
            }

            auto boneNode = findNode(this->npc.get(), boneName);

            if (!boneNode && !hasMerged)
            {
                logger::debug("Bone not found on skeleton, trying skeleton merge.");
                if (this->head.headParts.back().origPartRootNode)
                {
                    doSkeletonMerge(npc.get(), head.headParts.back().origPartRootNode.get(), head.prefix,
                                    head.renameMap);
                }
                else if (this->head.npcFaceGeomNode)
                {
                    // Facegen data doesn't have any tree structure to the skeleton. We need to make any new
                    // nodes children of the head node, so that they move properly when there's no physics.
                    // This case never happens to a lurker skeleton, thus we don't need to test.
                    RE::NiNode* npcHeadNode = findNode(head.npcFaceGeomNode.get(), "NPC Head [Head]");
                    if (npcHeadNode)
                    {
                        RE::NiTransform invTransform = npcHeadNode->local.Invert();
                        auto& children = head.npcFaceGeomNode->GetChildren();
                        for (uint16_t i = 0; i < children.size(); ++i)
                        {
                            const auto child = castNiNode(children[i].get());

                            // This case never happens to a lurker skeleton, thus we don't need to test.
                            if (child && !findNode(npc.get(), child->name))
                            {
                                child->local = invTransform * child->local;
                                head.npcFaceGeomNode->DetachChildAt2(i);
                                npcHeadNode->AttachChild(child, false);
                            }
                        }
                    }
                    doSkeletonMerge(npc.get(), this->head.npcFaceGeomNode.get(), head.prefix, head.renameMap);
                }

                hasMerged = true;

                auto postMergeRenameIt = this->head.renameMap.find(boneName.c_str());

                if (postMergeRenameIt != this->head.renameMap.end())
                {
                    logger::debug("Found renamed bone {} -> {}.", boneName, postMergeRenameIt->second.c_str());
                    boneName = postMergeRenameIt->second;
                    hasRenames = true;
                }

                boneNode = findNode(this->npc.get(), boneName);
            }

            if (!boneNode)
            {
                logger::error("Bone {} not found after skeleton merge, geometry cannot be fully skinned.", boneName);
                continue;
            }

            geometry->GetGeometryRuntimeData().skinInstance->bones[boneIdx] = boneNode;
            geometry->GetGeometryRuntimeData().skinInstance->boneWorldTransforms[boneIdx] = &boneNode->world;
        }

        geometry->GetGeometryRuntimeData().skinInstance->rootParent = headNode;

        if (hasRenames)
        {
            for (const auto& key : head.renameMap | std::views::keys)
            {
                if ((this->head.headParts.back().origPartRootNode &&
                        findObject(this->head.headParts.back().origPartRootNode.get(), key)) ||
                    (this->head.npcFaceGeomNode && findObject(this->head.npcFaceGeomNode.get(), key)))
                {
                    auto findNode = this->head.nodeUseCount.find(key);
                    if (findNode != this->head.nodeUseCount.end())
                    {
                        findNode->second += 1;
                        logger::debug("Incrementing use count by 1, it is now {}.", findNode->second);
                    }
                    else
                    {
                        this->head.nodeUseCount.insert(std::make_pair(key, static_cast<uint8_t>(1)));
                        logger::debug("First use of bone, count 1.");
                    }
                    head.headParts.back().renamedBonesInUse.insert(key);
                }
            }
        }

        logger::debug("Done skinning part.");
    }
} // namespace hdt
