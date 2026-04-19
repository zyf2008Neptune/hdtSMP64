#include "PluginInterfaceImpl.h"

hdt::PluginInterfaceImpl hdt::g_pluginInterface;

auto hdt::PluginInterfaceImpl::addListener(IPreStepListener* l) -> void
{
    if (l)
    {
        m_preStepDispatcher.AddEventSink(l);
    }
}

auto hdt::PluginInterfaceImpl::removeListener(IPreStepListener* l) -> void
{
    if (l)
    {
        m_preStepDispatcher.RemoveEventSink(l);
    }
}

auto hdt::PluginInterfaceImpl::addListener(IPostStepListener* l) -> void
{
    if (l)
    {
        m_postStepDispatcher.AddEventSink(l);
    }
}

auto hdt::PluginInterfaceImpl::removeListener(IPostStepListener* l) -> void
{
    if (l)
    {
        m_postStepDispatcher.RemoveEventSink(l);
    }
}

auto hdt::PluginInterfaceImpl::onPostPostLoad() -> void
{
    // Send ourselves to any plugin that registered during the PostLoad event
    if (m_skseMessagingInterface)
    {
        m_skseMessagingInterface->Dispatch(MSG_STARTUP, this, 0, nullptr);
    }
}

auto hdt::PluginInterfaceImpl::init(const SKSE::LoadInterface* skse) -> void
{
    // We need to have our SKSE plugin handle and the messaging interface in order to reach our plugins later
    if (skse)
    {
        m_sksePluginHandle = skse->GetPluginHandle();
        m_skseMessagingInterface =
            static_cast<SKSE::MessagingInterface*>(skse->QueryInterface(SKSE::LoadInterface::kMessaging));
    }
    if (!m_skseMessagingInterface)
    {
        logger::warn("Failed to get a messaging interface. Plugins will not work.");
    }
}
