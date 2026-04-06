#pragma once

namespace hdt
{
    class ForceUpdateList
    {
        typedef struct
        {
            std::unordered_set<RE::BSFixedString> nodes;
            std::unordered_set<RE::BSFixedString> nodes_mov;
        } nodeList_t;

    public:
        static auto GetSingleton() -> ForceUpdateList*;
        auto isAmong(const RE::BSFixedString& node_name) -> int;

    private:
        ForceUpdateList();
        nodeList_t m_list;
    };
} // namespace hdt
