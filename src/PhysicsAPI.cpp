#include "PhysicsAPI.hpp"
#include <types/ModuleInfo.hpp>
#include "forces/NewtonianField.hpp"
#include "integration/Rk4.hpp"
#include "RelativityPhysics.hpp"

extern "C" {
    std::unique_ptr<common::IPhysicsEngine> get_engine()
    {
        return std::make_unique<physics::RelativityPhysics>(std::make_unique<physics::forces::NewtonianField>(),
                                                            std::make_unique<physics::integration::Rk4>());
    }

    common::ModuleInfo get_module_info()
    {
        return {
            .name = ORBITAL_MODULE_NAME,
            .version = ORBITAL_MODULE_VERSION,
            .type = common::ModuleType::PHYSICS,
        };
    }
}
