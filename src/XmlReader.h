#pragma once

#include "XmlInspector/XmlInspector.hpp"
#include "XmlInspector/CharactersReader.hpp"

#include "hdtSkinnedMesh/hdtBulletHelper.h"

namespace hdt
{
	class XMLReader : public Xml::Inspector<Xml::Encoding::Utf8Writer>
	{
	private:
		using Base = Inspector<Xml::Encoding::Utf8Writer>;
		bool isEmptyStart;

	public:
		XMLReader(BYTE* data, size_t count) :
			Base(data, data + count) {}

		using Inspected = Xml::Inspected;

		auto Inspect() -> bool;
		auto GetInspected() -> Xml::Inspected;

		auto skipCurrentElement() -> void;
		auto nextStartElement() -> void;

		auto hasAttribute(const std::string& name) -> bool;
		auto getAttribute(const std::string& name) -> std::string;
		auto getAttribute(const std::string& name, const std::string& def) -> std::string;

		auto getAttributeAsFloat(const std::string& name) -> float;
		auto getAttributeAsInt(const std::string& name) -> int;
		auto getAttributeAsBool(const std::string& name) -> bool;

		auto readText() -> std::string;
		auto readFloat() -> float;
		auto readInt() -> int;
		auto readBool() -> bool;

		auto readVector3() -> btVector3;
		auto readQuaternion() -> btQuaternion;
		auto readAxisAngle() -> btQuaternion;
		auto readTransform() -> btTransform;
	};
}
