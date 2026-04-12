#pragma once

namespace Events
{
    struct FrameEvent
    {
        bool gamePaused;
    };

    struct FrameSyncEvent
    {};

    struct ShutdownEvent
    {};

    struct SkinAllHeadGeometryEvent
    {
        RE::NiNode* skeleton = nullptr;
        RE::BSFaceGenNiNode* headNode = nullptr;
        bool hasSkinned = false;
    };

    struct SkinSingleHeadGeometryEvent
    {
        RE::NiNode* skeleton = nullptr;
        RE::BSFaceGenNiNode* headNode = nullptr;
        RE::BSGeometry* geometry = nullptr;
    };

    struct ArmorAttachEvent
    {
        RE::NiNode* armorModel = nullptr;
        RE::NiNode* skeleton = nullptr;
        RE::NiAVObject* attachedNode = nullptr;
        bool hasAttached = false;
    };

    struct ArmorDetachEvent
    {
        RE::Actor* actor = nullptr;
        bool hasDetached = false;
    };

    namespace Sources
    {
        class FrameEventSource : public RE::BSTEventSource<FrameEvent>
        {
        public:
            static auto GetSingleton() -> FrameEventSource*;
        };

        class FrameSyncEventSource : public RE::BSTEventSource<FrameSyncEvent>
        {
        public:
            static auto GetSingleton() -> FrameSyncEventSource*;
        };

        class ShutdownEventEventSource : public RE::BSTEventSource<ShutdownEvent>
        {
        public:
            static auto GetSingleton() -> ShutdownEventEventSource*;
        };

        class SkinAllHeadGeometryEventSource : public RE::BSTEventSource<SkinAllHeadGeometryEvent>
        {
        public:
            static auto GetSingleton() -> SkinAllHeadGeometryEventSource*;
        };

        class SkinSingleHeadGeometryEventSource : public RE::BSTEventSource<SkinSingleHeadGeometryEvent>
        {
        public:
            static auto GetSingleton() -> SkinSingleHeadGeometryEventSource*;
        };

        class ArmorAttachEventSource : public RE::BSTEventSource<ArmorAttachEvent>
        {
        public:
            static auto GetSingleton() -> ArmorAttachEventSource*;
        };

        class ArmorDetachEventSource : public RE::BSTEventSource<ArmorDetachEvent>
        {
        public:
            static auto GetSingleton() -> ArmorDetachEventSource*;
        };
    } // namespace Sources

    namespace Sinks
    {
        class FreezeEventHandler : public RE::BSTEventSink<RE::MenuOpenCloseEvent>
        {
        public:
            static auto GetSingleton() -> FreezeEventHandler*;
            static auto Register() -> void;
            static auto Unregister() -> void;

        public:
            auto ProcessEvent(const RE::MenuOpenCloseEvent* a_event,
                              RE::BSTEventSource<RE::MenuOpenCloseEvent>* a_eventSource)
                -> RE::BSEventNotifyControl override;

        private:
        };
    } // namespace Sinks

    //
    auto Register() -> void;
    auto Unregister() -> void;
} // namespace Events
