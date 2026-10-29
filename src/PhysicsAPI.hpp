#pragma once

#include <memory>

#include <interfaces/IPhysicsEngine.hpp>
#include <types/ModuleInfo.hpp>

extern "C" {
    std::unique_ptr<common::IPhysicsEngine> get_engine();
}
