#include "StringImpl.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <RE/B/BSTSmartPointer.h>

namespace hdt
{
	StringImpl::StringImpl(size_t hash, std::string&& str) :
		m_hash(hash),
		m_str(std::move(str)) {}


	auto StringManager::instance() -> StringManager*
	{
		static StringManager s;
		return std::addressof(s);
	}

	auto StringManager::get(const char* begin, const char* end) -> StringImpl*
	{
		std::string str(begin, end);
		size_t hash = std::hash<std::string>()(str);
		auto& bucket = m_buckets[hash % BucketCount];
		return bucket.get(hash, std::move(str));
	}

	auto StringManager::Bucket::get(size_t hash, std::string&& str) -> StringImpl*
	{
		std::scoped_lock l(m_lock);

		auto iter = std::ranges::find_if(m_list, [&, hash](const auto& i) {
			return i->hash() == hash && i->str() == str;
		});

		if (iter != m_list.end()) {
			return iter->get();
		} else {
			return m_list.emplace_back(RE::make_smart<StringImpl>(hash, std::move(str))).get();
		}
	}

	auto StringManager::Bucket::clean() -> void
	{
		std::scoped_lock l(m_lock);

		m_list.erase(std::ranges::remove_if(m_list, [](const auto& i) { return i->GetRefCount() == 1; }).begin(), m_list.end());
	}

	StringManager::StringManager()
	{
		m_gcExit = false;
		m_gcThread = std::thread([this]() {
			while (!m_gcExit) {
				for (auto& i : m_buckets) {
					if (m_gcExit) {
						break;
					}

					i.clean();

					if (m_gcExit) {
						break;
					}

					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}

				if (m_gcExit) {
					break;
				}

				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		});
	}

	StringManager::~StringManager()
	{
		m_gcExit = true;
		if (m_gcThread.joinable()) {
			m_gcThread.join();
		}
	}
}
