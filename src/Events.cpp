#include "Events.h"
#include "ActorManager.h"
#include "hdtSkyrimPhysicsWorld.h"

namespace Events
{
    namespace Sources
    {
        auto FrameEventSource::GetSingleton() -> FrameEventSource*
        {
            static FrameEventSource singleton;
            return std::addressof(singleton);
        }

        auto FrameSyncEventSource::GetSingleton() -> FrameSyncEventSource*
        {
            static FrameSyncEventSource singleton;
            return std::addressof(singleton);
        }

        auto ShutdownEventEventSource::GetSingleton() -> ShutdownEventEventSource*
        {
            static ShutdownEventEventSource singleton;
            return std::addressof(singleton);
        }

        auto SkinAllHeadGeometryEventSource::GetSingleton() -> SkinAllHeadGeometryEventSource*
        {
            static SkinAllHeadGeometryEventSource singleton;
            return std::addressof(singleton);
        }

        auto SkinSingleHeadGeometryEventSource::GetSingleton() -> SkinSingleHeadGeometryEventSource*
        {
            static SkinSingleHeadGeometryEventSource singleton;
            return std::addressof(singleton);
        }

        auto ArmorAttachEventSource::GetSingleton() -> ArmorAttachEventSource*
        {
            static ArmorAttachEventSource singleton;
            return std::addressof(singleton);
        }

        auto ArmorDetachEventSource::GetSingleton() -> ArmorDetachEventSource*
        {
            static ArmorDetachEventSource singleton;
            return std::addressof(singleton);
        }
    } // namespace Sources

    namespace Sinks
    {
        auto FreezeEventHandler::GetSingleton() -> FreezeEventHandler*
        {
            static FreezeEventHandler singleton;
            return std::addressof(singleton);
        }

        auto FreezeEventHandler::Register() -> void { RE::UI::GetSingleton()->AddEventSink(GetSingleton()); }

        auto FreezeEventHandler::Unregister() -> void { RE::UI::GetSingleton()->RemoveEventSink(GetSingleton()); }

        auto FreezeEventHandler::ProcessEvent(
            const RE::MenuOpenCloseEvent* a_event,
            [[maybe_unused]] RE::BSTEventSource<RE::MenuOpenCloseEvent>* a_eventSource) -> RE::BSEventNotifyControl
        {
            if (a_event && a_event->opening &&
                (a_event->menuName == "Loading Menu" || a_event->menuName == "RaceSex Menu"))
            {
                logger::debug("{} detected, scheduling physics reset on world un-suspend.", a_event->menuName);
                hdt::SkyrimPhysicsWorld::get()->suspend(true);
            }

            if (a_event && !a_event->opening && (a_event->menuName == "RaceSex Menu"))
            {
                logger::debug("Racemenu closed, reloading meshes.");
                hdt::ActorManager::instance()->ProcessEvent(a_event, a_eventSource);
            }

            return RE::BSEventNotifyControl::kContinue;
        }
    } // namespace Sinks

    auto Register() -> void { Sinks::FreezeEventHandler::Register(); }

    auto Unregister() -> void { Sinks::FreezeEventHandler::Unregister(); }
} // namespace Events
