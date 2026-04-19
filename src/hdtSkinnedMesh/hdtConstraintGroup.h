#pragma once
#include "hdtBoneScaleConstraint.h"

namespace hdt
{
    class ConstraintGroup : public RefObject
    {
    public:
        auto scaleConstraint() const -> void
        {
            for (auto& i : m_constraints)
            {
                i->scaleConstraint();
            }
        }
        std::vector<RE::BSTSmartPointer<BoneScaleConstraint>> m_constraints;
    };
} // namespace hdt
