#include "DynamicHDT.h"

#include <boost/beast/core/string.hpp>

#include "hdtSkinnedMesh/hdtSkinnedMeshSystem.h"
#include "hdtSkyrimSystem.h"

auto hdt::util::splitArmorAddonFormID(const std::string& nodeName) -> uint32_t
{
    try
    {
        return std::stoul(nodeName.substr(1, 8), nullptr, 16);
    }
    catch (...)
    {
        return 0;
    }
}

auto hdt::util::UInt32toString(uint32_t formID) -> std::string { return fmt::format("{:08X}", formID); }

auto _deprefix(std::string_view str_with_prefix) -> std::string
{
    std::string str_no_prefix{str_with_prefix};
    static constexpr auto autoRenameSubstr = "hdtSSEPhysics_AutoRename_"sv;
    static constexpr auto str_size = autoRenameSubstr.size();
    if (str_with_prefix.size() >= autoRenameSubstr.size() &&
        boost::beast::iequals(str_with_prefix.substr(0, str_size), autoRenameSubstr))
    {
        str_no_prefix = str_with_prefix.substr(str_with_prefix.find(' ') + 1);
    }

    return str_no_prefix;
}

auto _match_name(const RE::BSFixedString& a, const RE::BSFixedString& b) -> bool
{
    if (a.empty() || b.empty())
    {
        return false;
    }

    return boost::beast::iequals(_deprefix(a), _deprefix(b));
}

auto hdt::util::transferCurrentPosesBetweenSystems(hdt::SkyrimSystem* src, hdt::SkyrimSystem* dst) -> void
{
    for (const auto& b1 : src->getBones())
    {
        if (!b1)
        {
            continue;
        }
        for (const auto& b2 : dst->getBones())
        {
            if (!b2)
            {
                continue;
            }

            if (_match_name(b1->m_name, b2->m_name))
            {
                b2->m_rig.setWorldTransform(b1->m_rig.getWorldTransform());
                b2->m_rig.setAngularVelocity(b1->m_rig.getAngularVelocity());
                b2->m_rig.setLinearVelocity(b1->m_rig.getLinearVelocity());
                break;
            }
        }
    }
}
