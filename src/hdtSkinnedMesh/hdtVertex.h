#pragma once

#include "hdtBone.h"

namespace hdt
{
    struct alignas(16) Vertex
    {
        Vertex()
        {
            ZeroMemory(this, sizeof(*this));
        }

        Vertex(const float x, const float y, const float z) :
            Vertex() { m_skinPos.setValue(x, y, z); }

        // skin info;
        btVector3 m_skinPos;
        float m_weight[4];
        U32 m_boneIdx[4];

        [[nodiscard]] auto getBoneIdx(const int i) const -> U32 { return m_boneIdx[i]; }

        auto setBoneIdx(const int i, const U32 idx) -> void
        {
            m_boneIdx[i] = idx;
        }

        auto sortWeight() -> void;
    };

    struct alignas(16) VertexPos
    {
        // position info

        auto set(const btVector3& p, const float m) -> void
        {
            m_data = p.get128();
            m_data.m128_f32[3] = m;
        }

        auto set(const btVector4& pm) -> void
        {
            m_data = pm.get128();
        }

        [[nodiscard]] auto pos() const -> btVector3 { return m_data; }
        [[nodiscard]] auto marginMultiplier4() const -> __m128 { return pshufd<0xFF>(m_data); }
        [[nodiscard]] auto marginMultiplier() const -> float { return _mm_cvtss_f32(marginMultiplier4()); }

        __m128 m_data;
    };
}
