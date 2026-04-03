#include "XmlReader.h"

namespace hdt
{
    static auto convertFloat(const std::string& str) -> float
    {
        // Replace decimal comma with point
        std::string s = str;
        size_t start_pos = s.find(",");
        if (start_pos != std::string::npos)
        {
            s.replace(start_pos, 1, ".");
        }

        errno = 0; // Reinitializing the error global variable (thread-safe)
        float ret = strtof(s.c_str(), nullptr);
        if (errno != 0) // Checking if there has been an error
        {
            throw std::string("not a float value");
        }
        return ret;
    }

    static auto convertInt(const std::string& str) -> int
    {
        auto begin = str.c_str();
        char* end;

        auto radix = 10;
        if (!str.compare(0, 2, "0x"))
        {
            radix = 16;
            begin += 2;
        }
        else if (str.length() > 1 && str[0] == '0')
        {
            begin += 1;
            radix = 8;
        }

        int ret = strtol(str.c_str(), &end, radix);
        if (end != str.c_str() + str.length())
        {
            throw std::string("not a int value");
        }
        return ret;
    }

    static auto convertBool(const std::string& str) -> bool
    {
        if (str == "true" || str == "1")
        {
            return true;
        }
        if (str == "false" || str == "0")
        {
            return false;
        }
        throw std::string("not a boolean");
    }

    auto XMLReader::Inspect() -> bool
    {
        if (Base::GetInspected() == Inspected::EmptyElementTag && isEmptyStart)
        {
            return isEmptyStart = false, true;
        }
        if (!Base::Inspect())
        {
            return false;
        }
        if (Base::GetInspected() == Inspected::EmptyElementTag)
        {
            isEmptyStart = true;
        }
        return true;
    }

    auto XMLReader::GetInspected() const -> Xml::Inspected
    {
        auto ret = Base::GetInspected();
        if (ret == Inspected::EmptyElementTag)
        {
            if (isEmptyStart)
            {
                return Inspected::StartTag;
            }
            return Inspected::EndTag;
        }
        return ret;
    }

    auto XMLReader::skipCurrentElement() -> void
    {
        if (GetInspected() == Inspected::EndTag)
        {
            return;
        }

        auto currentDepth = 1;
        while (currentDepth && Inspect())
        {
            switch (GetInspected())
            {
            case Inspected::StartTag:
            {
                ++currentDepth;
                break;
            }
            case Inspected::EndTag:
            {
                --currentDepth;
                break;
            }
            }
        }
    }

    auto XMLReader::nextStartElement() -> void
    {
        while (Inspect() && GetInspected() != Inspected::StartTag)
        {
            ;
        }
    }

    auto XMLReader::hasAttribute(const std::string& a_name) const -> bool
    {
        for (auto i = 0; i < GetAttributesCount(); ++i)
        {
            auto attr = GetAttributeAt(i);
            if (attr.Name == a_name)
            {
                return true;
            }
        }
        return false;
    }

    auto XMLReader::getAttribute(const std::string& a_name) const -> std::string
    {
        for (auto i = 0; i < GetAttributesCount(); ++i)
        {
            auto attr = GetAttributeAt(i);
            if (attr.Name == a_name)
            {
                return attr.Value;
            }
        }
        throw std::string("missing attribute : " + a_name);
    }

    auto XMLReader::getAttribute(const std::string& a_name, const std::string& def) const -> std::string
    {
        for (auto i = 0; i < GetAttributesCount(); ++i)
        {
            auto attr = GetAttributeAt(i);
            if (attr.Name == a_name)
            {
                return attr.Value;
            }
        }
        return def;
    }

    auto XMLReader::getAttributeAsFloat(const std::string& a_name) const -> float
    {
        return convertFloat(getAttribute(a_name));
    }

    auto XMLReader::getAttributeAsInt(const std::string& a_name) const -> int
    {
        return convertInt(getAttribute(a_name));
    }

    auto XMLReader::getAttributeAsBool(const std::string& a_name) const -> bool
    {
        return convertBool(getAttribute(a_name));
    }

    auto XMLReader::readText() -> std::string
    {
        Inspect();
        auto ret = GetValue();
        skipCurrentElement();
        return ret;
    }

    auto XMLReader::readFloat() -> float
    {
        Inspect();
        auto ret = convertFloat(GetValue());
        skipCurrentElement();
        return ret;
    }

    auto XMLReader::readInt() -> int
    {
        Inspect();
        auto ret = convertInt(GetValue());
        skipCurrentElement();
        return ret;
    }

    auto XMLReader::readBool() -> bool
    {
        Inspect();
        auto ret = convertBool(GetValue());
        skipCurrentElement();
        return ret;
    }

    auto XMLReader::readVector3() -> btVector3
    {
        float x = getAttributeAsFloat("x");
        float y = getAttributeAsFloat("y");
        float z = getAttributeAsFloat("z");
        skipCurrentElement();
        return {x, y, z};
    }

    auto XMLReader::readQuaternion() -> btQuaternion
    {
        float x = getAttributeAsFloat("x");
        float y = getAttributeAsFloat("y");
        float z = getAttributeAsFloat("z");
        float w = getAttributeAsFloat("w");
        skipCurrentElement();
        btQuaternion q(x, y, z, w);
        if (btFuzzyZero(q.length2()))
        {
            q = btQuaternion::getIdentity();
        }
        else
        {
            q.normalize();
        }
        return q;
    }

    auto XMLReader::readAxisAngle() -> btQuaternion
    {
        float x = getAttributeAsFloat("x");
        float y = getAttributeAsFloat("y");
        float z = getAttributeAsFloat("z");
        float w = getAttributeAsFloat("angle");
        skipCurrentElement();
        btQuaternion q;
        btVector3 axis(x, y, z);
        if (axis.fuzzyZero())
        {
            axis.setX(1);
            w = 0;
        }
        else
        {
            axis.normalize();
        }
        q.setRotation(axis, w);
        return q;
    }

    auto XMLReader::readTransform() -> btTransform
    {
        btTransform ret(btTransform::getIdentity());
        while (Inspect())
        {
            switch (GetInspected())
            {
            case Inspected::StartTag:
            {
                if (GetName() == "basis")
                {
                    ret.setRotation(readQuaternion());
                }
                else if (GetName() == "basis-axis-angle")
                {
                    ret.setRotation(readAxisAngle());
                }
                else if (GetName() == "origin")
                {
                    ret.setOrigin(readVector3());
                }
                break;
            }
            case Inspected::EndTag:
            {
                return ret;
            }
            }
        }
        return ret;
    }
}
