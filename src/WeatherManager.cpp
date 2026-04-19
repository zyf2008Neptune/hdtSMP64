#include "WeatherManager.h"
#include "hdtSkyrimPhysicsWorld.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace hdt
{
    auto WeatherManager::get() -> WeatherManager*
    {
        static WeatherManager instance;
        return &instance;
    }

    auto WeatherManager::randomGenerator(const int min, const int max) -> int
    {
        std::uniform_int_distribution dist(min, max);
        return dist(m_rng);
    }

    auto WeatherManager::randomGeneratorLowMoreProbable(const int lowerMin, const int lowerMax, const int higherMin,
                                                        const int higherMax, const int probability) -> int
    {
        std::uniform_int_distribution dist(1, probability);

        if (dist(m_rng) == 1)
        {
            std::uniform_int_distribution higher(higherMin, higherMax);
            return higher(m_rng);
        }

        std::uniform_int_distribution lower(lowerMin, lowerMax);
        return lower(m_rng);
    }

    auto WeatherManager::clearWind(const float cooldown) -> void
    {
        SkyrimPhysicsWorld::get()->setWind(RE::NiPoint3{0, 0, 0}, 0.f, 1);
        m_precipDirection = RE::NiPoint3{0, 0, 0};
        m_cooldown = cooldown;
    }

    auto WeatherManager::runWeatherTickImpl(const float delta) -> void
    {
        m_cooldown -= delta;
        if (m_cooldown > 0.f)
        {
            return;
        }

        const auto world = SkyrimPhysicsWorld::get();
        const auto skyPtr = RE::Sky::GetSingleton();

        if (!skyPtr)
        {
            clearWind(LONG_COOLDOWN); // remove wind immediately
            return;
        }

        const auto player = RE::PlayerCharacter::GetSingleton();
        if (!player || !player->loadedData)
        {
            clearWind(LONG_COOLDOWN); // remove wind immediately
            return;
        }

        const auto cell = player->parentCell;
        if (!cell)
        {
            clearWind(SHORT_COOLDOWN); // remove wind immediately
            return;
        }

        const auto worldSpace = cell->GetRuntimeData().worldSpace;
        if (!worldSpace)
        {
            clearWind(LONG_COOLDOWN); // remove wind immediately
            return;
        }

        if (std::ranges::contains(NOT_EXTERIOR_WORLDS, worldSpace->formID))
        {
            clearWind(LONG_COOLDOWN); // remove wind immediately
            return;
        }

        static constexpr float DEG_TO_RAD = std::numbers::pi_v<float> / 180.0f;
        const int range = randomGeneratorLowMoreProbable(0, 5, 6, 50, 10);
        const float rangeF = range / 10.0f;

        RE::NiPoint3 direction{0.f, 1.f, 0.f};

        if (skyPtr->currentWeather)
        {
            logger::debug(
                "Wind Speed: {:.2f}, Wind Direction: {:.2f}, Weather Wind Speed: {:.2f} WindDir: {:.2f} WindDirRange: "
                "{:.2f}",
                skyPtr->windSpeed, skyPtr->windAngle, static_cast<float>(skyPtr->currentWeather->data.windSpeed),
                skyPtr->currentWeather->data.windDirection * 180.0f / 256.0f,
                skyPtr->currentWeather->data.windDirectionRange * 360.0f / 256.0f);

            // use weather wind info
            // Wind Speed is the only thing that changes. Wind direction and range are same all the time as set in CK.
            const float theta = (skyPtr->currentWeather->data.windDirection * 180.0f / 256.0f) - 90.0f +
                static_cast<float>(randomGenerator(static_cast<int>(-rangeF), static_cast<int>(rangeF)));
            direction = rotate(direction, RE::NiPoint3(0, 0, 1.0f), theta * DEG_TO_RAD);
            world->setWind(direction, world->m_windStrength * scaleSkyrim * skyPtr->windSpeed);
        }
        else
        {
            logger::debug("Wind Speed: {:.2f}, Wind Direction: {:.2f}", skyPtr->windSpeed, skyPtr->windAngle);

            // use sky wind info
            const int randOffset = randomGenerator(0, 2 * static_cast<int>(rangeF)) - static_cast<int>(rangeF);
            const float theta = (skyPtr->windAngle * 180.0f / 256.0f) - 90.0f + static_cast<float>(randOffset);
            direction = rotate(direction, RE::NiPoint3(0, 0, 1.0f), theta * DEG_TO_RAD);
            world->setWind(direction, world->m_windStrength * scaleSkyrim * skyPtr->windSpeed);
        }

        m_precipDirection = direction;
        m_cooldown = SHORT_COOLDOWN;
    }
} // namespace hdt
