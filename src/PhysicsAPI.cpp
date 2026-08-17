#include "PhysicsAPI.hpp"

#include "RelativityPhysics.hpp"

extern "C" {
    std::unique_ptr<common::IPhysicsEngine> get_engine()
    {
        return std::make_unique<physics::RelativityPhysics>();
    }
}
