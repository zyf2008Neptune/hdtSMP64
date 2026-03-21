#pragma once

#include <unordered_set>

#include <RE/B/BSFixedString.h>

namespace hdt
{
    class ForceUpdateList
    {
        using nodeList_t = struct
        {
            std::unordered_set<RE::BSFixedString> nodes;
            std::unordered_set<RE::BSFixedString> nodes_mov;
        };

    public:
        static auto GetSingleton() -> ForceUpdateList*;
        auto isAmong(const RE::BSFixedString& node_name) const -> int;

    private:
        ForceUpdateList();
        nodeList_t m_list;
    };
}
