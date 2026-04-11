#pragma once

namespace hdt
{
    class SerializerBase;

    extern SKSE::PluginHandle g_PluginHandle;

    extern std::vector<SerializerBase*> g_SerializerList;

    struct _uint32_to_str_t
    {
        union _uin32_cstr
        {
            uint32_t _number;
            char _buffer[4];
        };

        auto operator()(uint32_t number) const -> std::string
        {
            _uin32_cstr _union{};
            _union._number = number;
            auto str = std::string(_union._buffer, 4);
            return std::string(str.rbegin(), str.rend());
        }
    };

    extern _uint32_to_str_t UInt32toStr;

    class SerializerBase
    {
    public:
        SerializerBase() = default;
        virtual ~SerializerBase() = default;
        virtual auto StorageName() -> uint32_t = 0;
        virtual auto FormatVersion() -> uint32_t = 0;

        virtual auto SaveData(SKSE::SerializationInterface*) -> void = 0;
        virtual auto ReadData(SKSE::SerializationInterface*, uint32_t) -> void = 0;

        static inline auto GetSerializerList() -> std::vector<SerializerBase*>& { return g_SerializerList; };

        static auto Save(SKSE::SerializationInterface* intfc) -> void
        {
            for (auto data_block : g_SerializerList)
            {
                // Console_Print("[HDT-SMP] Saving data, type: %s version: %08X", UInt32toStr(data_block->StorageName()).c_str(), data_block->FormatVersion());
                data_block->SaveData(intfc);
            }
        };

        static auto Load(SKSE::SerializationInterface* intfc) -> void
        {
            uint32_t type, version, length;
            //auto load_begin = clock();
            while (intfc->GetNextRecordInfo(type, version, length))
            {
                auto record = std::ranges::find_if(g_SerializerList,
                                                   [type, version](SerializerBase* a_srlzr)
                                                   {
                                                       return type == a_srlzr->StorageName() && version == a_srlzr->
                                                           FormatVersion();
                                                   });

                if (record == g_SerializerList.end())
                {
                    continue;
                }

                //_MESSAGE("[HDT-SMP] Reading data, type: %s version: %08X length: %d", UInt32toStr(type).c_str(), version, length);
                (*record)->ReadData(intfc, length);
            }
            //Less than a microsecond
            //Console_Print("[HDT-SMP] Serializer loading cost: %.3f sec.", (clock() - load_begin) / 1000.0f);
        };
    };

    template <class _Storage_t = void, class _Stream_t = std::stringstream>
    class Serializer : public SerializerBase
    {
    public:
        Serializer()
        {
            g_SerializerList.push_back(this);
        };

        ~Serializer() override = default;

        virtual auto Serialize() -> _Stream_t = 0;
        virtual auto Deserialize(_Stream_t&) -> _Storage_t = 0;

        auto SaveData(SKSE::SerializationInterface*) -> void override;
        auto ReadData(SKSE::SerializationInterface*, uint32_t) -> void override;

    protected:
        static inline auto _toString(_Stream_t& _stream) -> std::string
        {
            return _stream.rdbuf()->str();
        };
    };

    template <class _Storage_t, class _Stream_t>
    inline auto Serializer<_Storage_t, _Stream_t>::SaveData(SKSE::SerializationInterface* intfc) -> void
    {
        _Stream_t s_data_block = this->Serialize();
        if (intfc->OpenRecord(this->StorageName(), this->FormatVersion()))
        {
            intfc->WriteRecordData(_toString(s_data_block).c_str(),
                                   static_cast<uint32_t>(_toString(s_data_block).length()));
        }
        // Console_Print("Writing Data: \"%s\" \nStatus: %s", _toString(s_data_block).c_str(), success?"Succeeded":"Failed");
    }

    template <class _Storage_t, class _Stream_t>
    inline auto Serializer<_Storage_t, _Stream_t>::ReadData(SKSE::SerializationInterface* intfc,
                                                            uint32_t length) -> void
    {
        std::string s_data;
        s_data.resize(length);

        intfc->ReadRecordData(s_data.data(), length);

        _Stream_t _stream;
        _stream << s_data;
        this->Deserialize(_stream);
    }
}
