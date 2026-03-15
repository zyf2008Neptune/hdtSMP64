#pragma once

namespace hdt
{
	// Unified Const String Class
	struct IString : RE::BSIntrusiveRefCounted
	{
		virtual ~IString() = default;

		// add
		virtual auto cstr() const -> const char* = 0;
		virtual auto size() const -> size_t = 0;
	};
}
