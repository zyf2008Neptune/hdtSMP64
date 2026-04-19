#pragma once

#include "hdtBulletHelper.h"

namespace hdt
{
    struct Aabb
    {
        Aabb() { invalidate(); }

        Aabb(const __m128 mmin, const __m128 mmax) : m_min(mmin), m_max(mmax) {}

        __m128 m_min;
        __m128 m_max;

        [[nodiscard]] auto collideWith(const Aabb& rhs) const -> bool
        {
            const auto flag0 = _mm_cmplt_ps(rhs.m_max, m_min);
            const auto flag1 = _mm_cmplt_ps(m_max, rhs.m_min);
            const auto flag = _mm_movemask_ps(_mm_or_ps(flag0, flag1));
            return !(flag & 0x7);
        }

        auto invalidate() -> void
        {
            m_min = setAll(FLT_MAX);
            m_max = setAll(-FLT_MAX);
        }

        auto merge(const btVector3& p) -> void
        {
            m_min = _mm_min_ps(m_min, p.get128());
            m_max = _mm_max_ps(m_max, p.get128());
        }

        auto merge(const Aabb& rhs) -> void
        {
            m_min = _mm_min_ps(m_min, rhs.m_min);
            m_max = _mm_max_ps(m_max, rhs.m_max);
        }
    };

    struct BoundingSphere
    {
        BoundingSphere() = default;

        BoundingSphere(const btVector3& center, const float radius) : m_centerRadius(center)
        {
            m_centerRadius[3] = radius;
        }

        [[nodiscard]] auto center() const -> btVector3 { return m_centerRadius; }

        [[nodiscard]] auto radius() const -> float { return m_centerRadius.w(); }

        // SIMD stuff: splat radius from W into all lanes, then build AABB as center +/- radius
        [[nodiscard]] auto getAabb() const -> Aabb
        {
            const __m128 c = m_centerRadius.get128();
            const __m128 r = _mm_shuffle_ps(c, c, _MM_SHUFFLE(3, 3, 3, 3));
            return {_mm_sub_ps(c, r), _mm_add_ps(c, r)};
        }

        btVector4 m_centerRadius;
    };
} // namespace hdt
