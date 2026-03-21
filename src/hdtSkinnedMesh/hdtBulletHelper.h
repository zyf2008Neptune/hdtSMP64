#pragma once

#include <atomic>
#include <cassert>
#include <cfloat>
#include <limits>

#include <xmmintrin.h>

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btScalar.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

namespace hdt
{
    using I8 = int8_t;
    using I16 = int16_t;
    using I32 = int32_t;
    using I64 = int64_t;

    using U8 = uint8_t;
    using U16 = uint16_t;
    using U32 = uint32_t;
    using U64 = uint64_t;

    template <int imm>
    auto pshufd(const __m128 m) -> __m128
    {
        return _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(m), imm));
    }

    inline auto setAll(const float f) -> __m128 { return pshufd<0>(_mm_load_ss(&f)); }
    inline auto setAll0(const __m128 m) -> __m128 { return pshufd<0>(m); }
    inline auto setAll1(const __m128 m) -> __m128 { return pshufd<0x55>(m); }
    inline auto setAll2(const __m128 m) -> __m128 { return pshufd<0xAA>(m); }
    inline auto setAll3(const __m128 m) -> __m128 { return pshufd<0xFF>(m); }

    inline auto operator+=(__m128& l, const __m128 r) -> __m128&
    {
        l = _mm_add_ps(l, r);
        return l;
    }

    inline auto operator-=(__m128& l, const __m128 r) -> __m128&
    {
        l = _mm_sub_ps(l, r);
        return l;
    }

    inline auto operator*=(__m128& l, const __m128 r) -> __m128&
    {
        l = _mm_mul_ps(l, r);
        return l;
    }

    inline auto operator+=(__m128& l, const float r) -> __m128&
    {
        l = _mm_add_ps(l, setAll(r));
        return l;
    }

    inline auto operator-=(__m128& l, const float r) -> __m128&
    {
        l = _mm_sub_ps(l, setAll(r));
        return l;
    }

    inline auto operator*=(__m128& l, const float r) -> __m128&
    {
        l = _mm_mul_ps(l, setAll(r));
        return l;
    }

    inline auto cross(const __m128 a, const __m128 b) -> __m128
    {
        __m128 T, V;

        T = pshufd<_MM_SHUFFLE(3, 0, 2, 1)>(a);
        V = pshufd<_MM_SHUFFLE(3, 0, 2, 1)>(b);

        V = _mm_mul_ps(V, a);
        T = _mm_mul_ps(T, b);
        V = _mm_sub_ps(V, T);

        V = pshufd<_MM_SHUFFLE(3, 0, 2, 1)>(V);
        return V;
    }

    inline auto rsqrt(const float number) -> float
    {
        if (number <= 0.0f)
        {
            return std::numeric_limits<float>::infinity();
        }

        static constexpr auto threehalfs = 1.5F;
        const float x2 = number * 0.5F;

        // safe bit-level cast (C++20+)
        auto i = std::bit_cast<U32>(number);
        i = 0x5f375a86u - (i >> 1); // initial magic approximation
        auto res = std::bit_cast<float>(i);

        // one Newton-Raphson iteration is often sufficient:
        res = res * (threehalfs - (x2 * res * res));
        return res;
    }

    template <class T>
    auto abs(T rhs) -> T
    {
        return rhs < 0 ? -rhs : rhs;
    }

    template <>
    inline auto abs(const float rhs) -> float
    {
        return _mm_cvtss_f32(_mm_andnot_ps(_mm_set_ss(-0.f), _mm_set_ss(rhs)));
    }

    template <class T>
    auto min(const T& a, const T& b) restrict(cpu, amp) -> T
    {
        return a < b ? a : b;
    }

    template <class T>
    auto max(const T& a, const T& b) restrict(cpu, amp) -> T
    {
        return a < b ? b : a;
    }

    inline auto aligned(const int x, const int a) -> int { return (x + a - 1) & -a; }

    template <int a>
    auto aligned(const int x) -> int
    {
        return (x + a - 1) & -a;
    }

    inline auto aligned2Pow(const U32 lim) -> U32
    {
        unsigned long size;
        _BitScanReverse(&size, lim);
        size = 1 << size;
        return size;
    }

    ATTRIBUTE_ALIGNED16(class) btQsTransform
    {
        btQuaternion m_basis;
        btVector4 m_originScale;

    public:
        BT_DECLARE_ALIGNED_ALLOCATOR();

        btQsTransform() :
            m_basis(btQuaternion::getIdentity()), m_originScale(0, 0, 0, 1) {}

        btQsTransform(const btQuaternion& r, const btVector3& t, const float s = 1.0f) :
            m_basis(r), m_originScale(t.get128())
        {
            setScale(s);
        }

        explicit btQsTransform(const btTransform& t, const float s = 1.0f)
        {
            m_basis = t.getRotation();
            m_originScale = t.getOrigin();
            setScale(s);
        }

        btQsTransform(const btQsTransform& rhs) :
            m_basis(rhs.m_basis), m_originScale(rhs.m_originScale.get128())
        {
            assert(getScale() > 0);
        }

        auto getBasis() const -> btQuaternion { return m_basis; }
        auto getBasis() -> btQuaternion& { return m_basis; }
        auto setBasis(const btQuaternion& q) -> void { m_basis = q; }
        auto setBasis(const btMatrix3x3& m) -> void { m.getRotation(m_basis); }

        auto getScale() const -> float { return m_originScale[3]; }
        auto getScale() -> float& { return m_originScale[3]; }
        auto getScaleReg() const -> float { return _mm_cvtss_f32(setAll3(m_originScale.get128())); }

        auto setScale(const float s) -> void
        {
            assert(s > 0);
            m_originScale[3] = s;
        }

        auto getOrigin() const -> btVector3 { return m_originScale; }

        auto setOrigin(const btVector3& vec) -> void
        {
            const float s = getScale();
            m_originScale = vec.mVec128;
            setScale(s);
        }

        auto setOrigin(const float x, const float y, const float z) -> void
        {
            m_originScale[0] = x;
            m_originScale[1] = y;
            m_originScale[2] = z;
        }

        auto operator=(const btQsTransform& rhs) -> btQsTransform&
        {
            m_basis = rhs.m_basis;
            m_originScale = rhs.m_originScale;
            assert(getScale() > 0);
            return *this;
        }

        auto operator*(const btQsTransform& rhs) const -> btQsTransform
        {
            return btQsTransform(m_basis * rhs.m_basis, *this * rhs.getOrigin(), getScale() * rhs.getScale());
        }

        auto operator*(const btVector3& rhs) const -> btVector3
        {
            return getOrigin() + quatRotate(m_basis, rhs * getScale());
        }

        auto operator*=(const btQsTransform& rhs) -> void
        {
            const float s = getScale();
            setOrigin(getOrigin() + quatRotate(m_basis, rhs.getOrigin() * s));
            m_basis *= rhs.m_basis;
            setScale(s * rhs.getScale());
        }

        auto inverse() const -> btQsTransform
        {
            const auto r = m_basis.inverse();
            const auto s = 1.0f / getScale();
            return btQsTransform(r, quatRotate(r, -getOrigin() * s), s);
        }

        auto asTransform() const -> btTransform { return btTransform(m_basis, m_originScale); }
    };

    ATTRIBUTE_ALIGNED16(class) btMatrix4x3 : public btMatrix3x3
    {
    public:
        btMatrix4x3() = default;

        btMatrix4x3(const btQsTransform& t)
        {
            this->setRotation(t.getBasis());
            __m128 scale = pshufd<0xFF>(t.getOrigin().get128());
            m_row[0] = _mm_mul_ps(m_row[0], scale);
            m_row[1] = _mm_mul_ps(m_row[1], scale);
            m_row[2] = _mm_mul_ps(m_row[2], scale);
            m_row[0].m128_f32[3] = t.getOrigin()[0];
            m_row[1].m128_f32[3] = t.getOrigin()[1];
            m_row[2].m128_f32[3] = t.getOrigin()[2];
        }

        /*btMatrix4x3(const btMatrix4x3& rhs)
        {
        _mm_store_ps(m_row[0].m128_f32, rhs.m_row[0]);
        _mm_store_ps(m_row[1].m128_f32, rhs.m_row[1]);
        _mm_store_ps(m_row[2].m128_f32, rhs.m_row[2]);
        }*/

        auto operator*(const btVector3& rhs) const -> btVector3
        {
#ifdef BT_ALLOW_SSE4
            auto v = _mm_blend_ps(rhs.get128(), _mm_set_ps1(1), 0x8);
            __m128 xmm0 = _mm_dp_ps(m_row[0], v, 0xF1);
            __m128 xmm1 = _mm_dp_ps(m_row[1], v, 0xF2);
            __m128 xmm2 = _mm_dp_ps(m_row[2], v, 0xF4);
            xmm0 = _mm_or_ps(xmm0, xmm1);
            xmm0 = _mm_or_ps(xmm0, xmm2);
#else
            auto v = rhs.get128();
            v.m128_f32[3] = 1;

            __m128 xmm0 = _mm_mul_ps(m_row[0], v);
            __m128 xmm1 = _mm_mul_ps(m_row[1], v);
            __m128 xmm2 = _mm_mul_ps(m_row[2], v);

            xmm0 = _mm_hadd_ps(xmm0, xmm1);
            xmm2 = _mm_hadd_ps(xmm2, xmm2);
            xmm0 = _mm_hadd_ps(xmm0, xmm2);
#endif
            return xmm0;
        }

        auto mulPack(const btVector3& rhs, float packW) const -> __m128
        {
#ifdef BT_ALLOW_SSE4
            auto v = _mm_blend_ps(rhs.get128(), _mm_set_ps1(1), 0x8);
            __m128 xmm0 = _mm_dp_ps(m_row[0], v, 0xF1); // x, 0, 0, 0
            __m128 xmm1 = _mm_dp_ps(m_row[1], v, 0xF2); // 0, y, 0, 0
            xmm0 = _mm_or_ps(xmm0, xmm1); // x, y, 0, 0
            xmm1 = _mm_dp_ps(m_row[2], v, 0xF1); // z, 0, 0, 0
            xmm1 = _mm_unpacklo_ps(xmm1, _mm_set_ss(packW)); // z, w, 0, 0
            xmm0 = _mm_movelh_ps(xmm0, xmm1); // x, y, z, w
#else
            auto v = rhs.get128();
            v.m128_f32[3] = 1;
            auto w = _mm_load_ss(&packW);

            __m128 xmm0 = _mm_mul_ps(m_row[0], v);
            __m128 xmm1 = _mm_mul_ps(m_row[1], v);
            __m128 xmm2 = _mm_mul_ps(m_row[2], v);

            xmm0 = _mm_hadd_ps(xmm0, xmm1);
            xmm2 = _mm_hadd_ps(xmm2, w);
            xmm0 = _mm_hadd_ps(xmm0, xmm2);
#endif
            return xmm0;
        }

        /*inline btMatrix4x3& operator =(const btMatrix4x3& rhs)
        {
        _mm_store_ps(m_row[0].m128_f32, rhs.m_row[0]);
        _mm_store_ps(m_row[1].m128_f32, rhs.m_row[1]);
        _mm_store_ps(m_row[2].m128_f32, rhs.m_row[2]);
        return *this;
        }*/

        __m128 m_row[3];
    };

    ATTRIBUTE_ALIGNED16(class) btMatrix4x3T : public btMatrix3x3
    {
    public:
        btMatrix4x3T() = default;

        btMatrix4x3T(const btQsTransform& t)
        {
            btMatrix3x3 rot;
            rot.setRotation(t.getBasis());
            rot = rot.transpose();
            __m128 scale = pshufd<0xFF>(t.getOrigin().get128());
            m_col[0] = rot[0].get128() * scale;
            m_col[1] = rot[1].get128() * scale;
            m_col[2] = rot[2].get128() * scale;
            m_col[3] = t.getOrigin().get128();
        }

        auto operator*(const btVector3& rhs) const -> btVector3
        {
            return m_col[0] * rhs[0] + m_col[1] * rhs[1] + m_col[2] * rhs[2] + m_col[3];
        }

        auto operator*(const btMatrix4x3T& r) const -> btMatrix4x3T // should not overload operator pointer
        {
            btMatrix4x3T ret;
            ret.m_col[0] = m_col[0] * r.m_col[0][0] + m_col[1] * r.m_col[0][1] + m_col[2] * r.m_col[0][2];
            ret.m_col[1] = m_col[0] * r.m_col[1][0] + m_col[1] * r.m_col[1][1] + m_col[2] * r.m_col[1][2];
            ret.m_col[2] = m_col[0] * r.m_col[2][0] + m_col[1] * r.m_col[2][1] + m_col[2] * r.m_col[2][2];
            ret.m_col[3] = *this * r.m_col[3];
            return ret;
        }

        auto basis() const -> btMatrix3x3 { return this->transpose(); }
        auto toTransform() const -> btTransform { return btTransform(this->transpose(), m_col[3]); }
        /*inline btMatrix4x3& operator =(const btMatrix4x3& rhs)
        {
        _mm_store_ps(m_row[0].m128_f32, rhs.m_row[0]);
        _mm_store_ps(m_row[1].m128_f32, rhs.m_row[1]);
        _mm_store_ps(m_row[2].m128_f32, rhs.m_row[2]);
        return *this;
        }*/

        btVector3 m_col[4];
    };

    template <>
    inline auto abs(const btVector3 rhs) -> btVector3
    {
        return _mm_andnot_ps(_mm_set_ps1(-0.f), rhs.get128());
    }

    template <class T>
    using vectorA16 = std::vector<T>;

    class SpinLock
    {
    public:
        auto lock() -> void
        {
            long count = 0;
            while (m_flag.test_and_set(std::memory_order_acquire))
            {
                if (++count > 10000)
                {
                    SwitchToThread();
                }
            }
        }

        auto unlock() -> void { m_flag.clear(std::memory_order_release); }

    protected:
        std::atomic_flag m_flag = ATOMIC_FLAG_INIT;
    };
} // namespace hdt
