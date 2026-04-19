#pragma once

//
auto GetTextureFromIndex(RE::BSLightingShaderMaterial* material, std::uint32_t index) -> RE::NiSourceTexturePtr*;

//
auto DumpNodeChildren(RE::NiAVObject* node) -> void;
