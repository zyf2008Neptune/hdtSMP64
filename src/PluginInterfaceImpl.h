#pragma once

#include "PluginAPI.h"

namespace hdt
{
    class PluginInterfaceImpl final : public PluginInterface
    {
    public:
        PluginInterfaceImpl() = default;
        ~PluginInterfaceImpl() override = default;

        PluginInterfaceImpl(const PluginInterfaceImpl&) = delete;
        auto operator=(const PluginInterfaceImpl&) -> PluginInterfaceImpl& = delete;

        auto getVersionInfo() const -> const VersionInfo& override { return m_versionInfo; }

        auto addListener(IPreStepListener* l) -> void override;
        auto removeListener(IPreStepListener* l) -> void override;

        auto setPlayerBodyCameraDriven(bool active) -> void override;
        auto isPlayerBodyCameraDriven() const -> bool { return m_playerBodyCameraDriven; }

        auto addListener(IPostStepListener* l) -> void override;
        auto removeListener(IPostStepListener* l) -> void override;

        auto onPostPostLoad() -> void;

        auto onPreStep(const PreStepEvent& e) -> void { m_preStepDispatcher.SendEvent(std::addressof(e)); }
        auto onPostStep(const PostStepEvent& e) -> void { m_postStepDispatcher.SendEvent(std::addressof(e)); }

        auto init(const SKSE::LoadInterface* skse) -> void;

    private:
        VersionInfo m_versionInfo{.interfaceVersion = INTERFACE_VERSION, .bulletVersion = BULLET_VERSION};
        RE::BSTEventSource<PreStepEvent> m_preStepDispatcher;
        RE::BSTEventSource<PostStepEvent> m_postStepDispatcher;
        bool m_playerBodyCameraDriven{false};
        SKSE::PluginHandle m_sksePluginHandle;
        SKSE::MessagingInterface* m_skseMessagingInterface;
    };

    extern PluginInterfaceImpl g_pluginInterface;
} // namespace hdt
