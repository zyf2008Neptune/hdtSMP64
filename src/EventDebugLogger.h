#pragma once

#include "Events.h"

namespace hdt
{
    class EventDebugLogger : public RE::BSTEventSink<Events::ArmorAttachEvent>,
                             public RE::BSTEventSink<RE::TESCellAttachDetachEvent>,
                             public RE::BSTEventSink<RE::TESMoveAttachDetachEvent>
    {
    protected:
        auto ProcessEvent(const RE::TESCellAttachDetachEvent* evn,
                          RE::BSTEventSource<RE::TESCellAttachDetachEvent>* dispatcher)
            -> RE::BSEventNotifyControl override;
        auto ProcessEvent(const RE::TESMoveAttachDetachEvent* evn,
                          RE::BSTEventSource<RE::TESMoveAttachDetachEvent>* dispatcher)
            -> RE::BSEventNotifyControl override;
        auto ProcessEvent(const Events::ArmorAttachEvent*, RE::BSTEventSource<Events::ArmorAttachEvent>*)
            -> RE::BSEventNotifyControl override;
    };
} // namespace hdt
