#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <RE/B/BSTSmartPointer.h>

#include "IString.h"

namespace hdt
{
	class StringImpl : public IString
	{
	public:
		StringImpl(size_t hash, std::string&& str);
		~StringImpl() override = default;

		auto cstr() const -> const char* override { return m_str.c_str(); }
		auto size() const -> size_t override { return m_str.size(); }

		auto hash() const -> size_t { return m_hash; }
		auto str() const -> std::string { return m_str; }

		auto GetRefCount() const noexcept -> std::uint32_t { return _refCount; }

	protected:
		size_t m_hash;
		std::string m_str;
	};

	class StringManager final
	{
	public:
		static constexpr std::uint32_t BucketCount = 65536;

		//
		class Bucket
		{
		public:
			auto get(size_t hash, std::string&& str) -> StringImpl*;
			auto clean() -> void;

		protected:
			std::vector<RE::BSTSmartPointer<StringImpl>> m_list;
			std::mutex m_lock;
		};

	public:
		static auto instance() -> StringManager*;

		//
		auto get(const char* begin, const char* end) -> StringImpl*;

	private:
		Bucket m_buckets[BucketCount];
		RE::BSTSmartPointer<StringImpl> m_empty;
		std::thread m_gcThread;
		std::atomic_bool m_gcExit;

	private:
		StringManager();
		~StringManager();
	};
}
