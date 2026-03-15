#pragma once

#include <cstdint>
#include <fstream>
#include <string>

#include <RE/B/BSDynamicTriShape.h>
#include <RE/B/BSFixedString.h>
#include <RE/B/BSResourceNiBinaryStream.h>
#include <RE/B/BSTriShape.h>
#include <RE/N/NiAVObject.h>
#include <RE/N/NiNode.h>

namespace hdt
{
	static auto setNiNodeName(RE::NiNode* node, const char* name) -> void
	{
		node->name = name;
	}

	static auto castNiNode(RE::NiAVObject* obj) -> RE::NiNode*
	{
		return obj ? obj->AsNode() : nullptr;
	}

	inline auto castBSTriShape(RE::NiAVObject* obj) -> RE::BSTriShape*
	{
		return obj ? obj->AsTriShape() : nullptr;
	}

	static auto castBSDynamicTriShape(RE::NiAVObject* obj) -> RE::BSDynamicTriShape*
	{
		return obj ? obj->AsDynamicTriShape() : nullptr;
	}

	static auto findObject(RE::NiAVObject* obj, const RE::BSFixedString& name) -> RE::NiAVObject*
	{
		return obj->GetObjectByName(name);
	}

	static auto findNode(RE::NiNode* obj, const RE::BSFixedString& name) -> RE::NiNode*
	{
		auto ret = obj->GetObjectByName(name);
		return ret ? ret->AsNode() : nullptr;
	}

	static auto readAllFile(const char* path) -> std::string
	{
		RE::BSResourceNiBinaryStream stream(path);
		if (!stream.good()) {
			return "";
		}

		//
		std::string file{};

		//
		size_t required = stream.stream->totalSize;

		//
		file.resize(required);

		//
		stream.read(file.data(), static_cast<uint32_t>(required));

		//
		return file;
	}

	static auto readAllFile2(const char* path) -> std::string
	{
		std::ifstream stream(path, std::ios::binary);
		if (!stream.is_open()) {
			return "";
		}

		stream.seekg(0, std::ios::end);
		auto size = stream.tellg();
		stream.seekg(0, std::ios::beg);
		std::string ret;
		ret.resize(size);
		stream.read(ret.data(), size);
		return ret;
	}

	static auto updateTransformUpDown(RE::NiAVObject* obj, bool dirty) -> void
	{
		if (!obj) {
			return;
		}

		RE::NiUpdateData ctx = {
			0.f,
			dirty ? RE::NiUpdateData::Flag::kDirty : RE::NiUpdateData::Flag::kNone
		};

		//
		obj->UpdateWorldData(&ctx);

		//
		RE::NiNode* node = castNiNode(obj);
		if (node) {
			for (auto& child : node->GetChildren()) {
				if (child) {
					updateTransformUpDown(child.get(), dirty);
				}
			}
		}
	}
}
