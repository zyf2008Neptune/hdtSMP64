#pragma once

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <RE/N/NiMatrix3.h>
#include <RE/N/NiPoint3.h>
#include <RE/N/NiTransform.h>

#include "hdtSkinnedMesh/hdtBulletHelper.h"

namespace hdt
{
	auto convertNi(const RE::NiMatrix3& rhs) -> btQuaternion;

	inline auto convertNi(const RE::NiPoint3& rhs) -> btVector3
	{
		return { rhs.x, rhs.y, rhs.z };
	}

	inline auto convertNi(const RE::NiTransform& rhs) -> btQsTransform
	{
		btQsTransform ret;
		ret.setBasis(convertNi(rhs.rotate));
		ret.setOrigin(convertNi(rhs.translate));
		ret.setScale(rhs.scale);
		return ret;
	}

	auto convertBt(const btVector3& rhs) -> RE::NiPoint3;
	auto convertBt(const btMatrix3x3& rhs) -> RE::NiMatrix3;
	auto convertBt(const btQuaternion& rhs) -> RE::NiMatrix3;
	auto convertBt(const btQsTransform& rhs) -> RE::NiTransform;

	inline constexpr auto scaleRealWorld = 0.01425f;
	inline constexpr auto scaleSkyrim = 1.0f / scaleRealWorld;
}
