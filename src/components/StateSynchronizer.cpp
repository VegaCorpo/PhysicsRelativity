#include "components/StateSynchronizer.hpp"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <unordered_map>
#include <utils/mass.hpp>
#include <utils/vector3.hpp>
#include <vector>

namespace {
    using common::math::Vector3;
    using ProperTimesByEntity = std::unordered_map<std::size_t, double>;

    constexpr double DEFAULT_SOFTENING_LENGTH = 1e-6;
    constexpr double INITIAL_PROPER_TIME = 0.0;

    [[nodiscard]] std::size_t bodyCountOf(const common::SpecificDataPhysics& world)
    {
        return std::min(
            {world.entitiesId.size(), world.positions.size(), world.velocities.size(), world.masses.size()});
    }

    template <typename Value>
    [[nodiscard]] auto prefixEnd(const std::vector<Value>& values, std::size_t count)
    {
        return std::next(values.begin(), static_cast<std::ptrdiff_t>(count));
    }

    template <typename Component>
    void readVectors(const std::vector<Component>& components, std::vector<Vector3>& vectors)
    {
        std::transform(components.begin(), prefixEnd(components, vectors.size()), vectors.begin(),
                       [](const Component& component) { return Vector3{component.x, component.y, component.z}; });
    }

    template <typename Component>
    void writeVectors(const std::vector<Vector3>& vectors, std::vector<Component>& components)
    {
        const std::size_t count = std::min(vectors.size(), components.size());

        std::transform(vectors.begin(), prefixEnd(vectors, count), components.begin(),
                       [](const Vector3& vector) { return Component{vector.x, vector.y, vector.z}; });
    }

    void readMasses(const common::SpecificDataPhysics& world, physics::RelativisticState& state)
    {
        std::transform(world.masses.begin(), prefixEnd(world.masses, state.size()), state.masses.begin(),
                       common::mass::toKilograms);
    }

    [[nodiscard]] ProperTimesByEntity properTimesByEntity(const physics::RelativisticState& state)
    {
        ProperTimesByEntity properTimes;

        properTimes.reserve(state.size());
        for (std::size_t body = 0; body < state.size(); body += 1)
            properTimes.emplace(state.entitiesId[body], state.properTimes[body]);
        return properTimes;
    }

    [[nodiscard]] double properTimeOf(const ProperTimesByEntity& properTimes, std::size_t entityId)
    {
        const auto found = properTimes.find(entityId);

        return found == properTimes.end() ? INITIAL_PROPER_TIME : found->second;
    }

    void restoreProperTimes(const ProperTimesByEntity& previous, physics::RelativisticState& state)
    {
        std::ranges::transform(state.entitiesId, state.properTimes.begin(),
                               [&previous](std::size_t entityId) { return properTimeOf(previous, entityId); });
    }

    [[nodiscard]] double softeningLengthOf(const common::SpecificDataPhysics& world)
    {
        return world.epsilon != 0.0 ? world.epsilon : DEFAULT_SOFTENING_LENGTH;
    }
} // namespace

void physics::synchronization::syncIn(const common::SpecificDataPhysics& world, RelativisticState& state)
{
    const ProperTimesByEntity previous = properTimesByEntity(state);

    state.resize(bodyCountOf(world));
    std::copy_n(world.entitiesId.begin(), state.size(), state.entitiesId.begin());
    readMasses(world, state);
    readVectors(world.positions, state.positions);
    readVectors(world.velocities, state.velocities);
    restoreProperTimes(previous, state);
    state.softeningLength = softeningLengthOf(world);
}

void physics::synchronization::syncOut(const RelativisticState& state, common::SpecificDataPhysics& world)
{
    writeVectors(state.positions, world.positions);
    writeVectors(state.velocities, world.velocities);
    writeVectors(state.accelerations, world.accelerations);
}
