#pragma once

#include <hdtSerialization.h>
#include "DynamicHDT.h"

extern bool g_hasPapyrusExtension;

namespace hdt::Override
{

    // The formID of the armoraddon in ArmorAttachEvent cannot be acquired, which makes it impossible to check override
    // by the formID upon attaching armoraddon.
    class OverrideManager : public Serializer<>
    {
    public:
        ~OverrideManager() override = default;

        // Override virtual methods inherited from Serializer
        auto FormatVersion() -> uint32_t override { return 1; }

        auto StorageName() -> uint32_t override { return 'APFW'; }

        auto Serialize() -> std::stringstream override;

        auto Deserialize(std::stringstream&) -> void override;
        // Inherit End

        static auto GetSingleton() -> OverrideManager*;

        auto queryOverrideData() const -> std::string;

        auto registerOverride(uint32_t actor_formID, std::string old_file_path, std::string_view new_file_path) -> bool;

        auto checkOverride(uint32_t actor_formID, std::string_view old_file_path) -> std::string;

    protected:
        OverrideManager() = default;
        std::unordered_map<uint32_t, std::unordered_map<std::string, std::string>> m_ActorPhysicsFileSwapList;
    };
} // namespace hdt::Override
