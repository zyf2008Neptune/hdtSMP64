#pragma once

#include <memory>

#include "StringImpl.h"

namespace hdt
{
	class IDStr
	{
	public:
		// 1
		IDStr() = default;

		// 2 (copy)
		IDStr(const IDStr& a_rhs) :
			_ptr(a_rhs._ptr) {}

		// 3 (move)
		IDStr(IDStr&& a_rhs) noexcept :
			_ptr(std::exchange(a_rhs._ptr, nullptr)) {}

		// 4 (init)
		IDStr(const char* str) :
			_ptr(!str ? nullptr : StringManager::instance()->get(str, str + strlen(str))) {}

		// 5 (init)
		IDStr(const std::string& str) :
			_ptr(!str.c_str() ? nullptr : StringManager::instance()->get(str.c_str(), str.c_str() + str.length())) {}

		// 0 (copy)
		auto operator=(const IDStr& a_rhs) -> IDStr&
		{
			if (this != std::addressof(a_rhs)) {
				_ptr = a_rhs._ptr;
			}

			return *this;
		}

		// (move)
		auto operator=(IDStr&& a_rhs) noexcept -> IDStr&
		{
			if (this != std::addressof(a_rhs)) {
				_ptr = a_rhs._ptr;
				a_rhs._ptr = nullptr;
			}

			return *this;
		}

		//
		explicit constexpr operator bool() const noexcept
		{
			return static_cast<bool>(_ptr.get());
		}

		auto operator==(const IDStr& lhs) const noexcept -> bool
		{
			return _ptr == lhs._ptr;
		}

		constexpr auto operator*() const noexcept -> IString&
		{
			return *_ptr;
		}

		constexpr auto operator->() const noexcept -> IString*
		{
			return _ptr.get();
		}

		constexpr operator IString*() const noexcept
		{
			return _ptr.get();
		}

		operator RE::BSTSmartPointer<IString>() const noexcept
		{
			return _ptr;
		}

		constexpr auto get() const noexcept -> IString*
		{
			return _ptr.get();
		}

		auto operator==(const IDStr& a_rhs) -> bool
		{
			return _ptr == a_rhs._ptr;
		}

		auto operator!=(const IDStr& a_rhs) -> bool
		{
			return !(*this == a_rhs);
		}

	private:
		RE::BSTSmartPointer<IString> _ptr{ nullptr };
	};
}
