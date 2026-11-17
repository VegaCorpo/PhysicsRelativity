# PhysicsRelativity

The module uses kilometres and seconds. The speed of light is
`common::constants::SPEED_OF_LIGHT` (`299792.458 km/s`, `Common/utils/physicalConstants.hpp`). The functions in
`src/utils/RelativityUtils.hpp` provide checked inertial-frame transformations.

## Architecture

`RelativityPhysics` implements `common::IPhysicsEngine` and only orchestrates. Its acceleration field and its
integrator are injected by `get_engine()` (`src/PhysicsAPI.cpp`) through the generic interfaces of
`Common/integration`, specialised for this module in `src/integration/Bodies.hpp`.

| Unit | Responsibility |
|---|---|
| `components/RelativisticState` | bodies of the current update: masses (kg), positions, velocities, accelerations, Lorentz factors, proper times |
| `components/StateSynchronizer` | copy between `common::SpecificDataPhysics` and the state; proper times survive by entity id |
| `forces/NewtonianField` | Newtonian N-body accelerations, O(N²), softened by `epsilon` (1e-6 km when the scene gives 0) |
| `integration/Rk4` | classical fourth-order Runge-Kutta on `a(x, v)`; its scratch memory comes with `Bodies` |
| `kinematics/ProperTime` | Lorentz factors and `τ += dt / γ` for every sub-step |

Every update is split by `common::integration::TimeStep` into sub-steps of at most 7200 s per force evaluation
(28800 s for RK4). The module always integrates with RK4: `SpecificDataPhysics::integrator` selects between
Verlet and PEFRL, which assume a velocity-independent field and are not valid for the post-Newtonian field.

## Lorentz transformations

`transformToMovingFrame(event, frameVelocity)` expresses an event in a frame
moving at `frameVelocity` relative to the original frame. The event origins
coincide at `t = 0`. The boost accepts any subluminal 3D velocity:

```cpp
using namespace physics::utils;

SpacetimeEvent event{10.0, 1000.0, 0.0, 0.0};
RelativeVelocity boost{0.5 * common::constants::SPEED_OF_LIGHT, 0.0, 0.0};
auto inMovingFrame = transformToMovingFrame(event, boost);
if (inMovingFrame) {
    auto backInOriginalFrame = transformFromMovingFrame(*inMovingFrame, boost);
}
```

Invalid or non-finite inputs return `std::unexpected` with a
`LorentzTransformError`. `rotateSpatially` applies a proper spatial rotation
while leaving time unchanged. `transformBetweenFrames` applies two frame
transformations successively. The interval uses the `(+---)` convention:

```cpp
const double invariant = minkowskiIntervalSquared(event.time, event.x, event.y, event.z);
```

`transformVelocityToMovingFrame` implements relativistic velocity composition
in 3D; inputs must both be subluminal. For collinear velocities it reduces to
the usual `(u - v) / (1 - uv/c²)` law.

## Unit tests

The white-box unit tests follow the structure of `Physics/tests/unit`:

```sh
cmake -S PhysicsRelativity/tests/unit -B build/physics-relativity-unit
cmake --build build/physics-relativity-unit -j
ctest --test-dir build/physics-relativity-unit --output-on-failure
```
