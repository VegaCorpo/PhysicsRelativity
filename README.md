# PhysicsRelativity

The module uses kilometres and seconds. The speed of light is
`RelativisticState::SPEED_OF_LIGHT` (`299792.458 km/s`). The functions in
`src/utils/RelativityUtils.hpp` provide checked inertial-frame transformations.

## Lorentz transformations

`transformToMovingFrame(event, frameVelocity)` expresses an event in a frame
moving at `frameVelocity` relative to the original frame. The event origins
coincide at `t = 0`. The boost accepts any subluminal 3D velocity:

```cpp
using namespace physics::utils;

SpacetimeEvent event{10.0, 1000.0, 0.0, 0.0};
RelativeVelocity boost{0.5 * physics::RelativisticState::SPEED_OF_LIGHT, 0.0, 0.0};
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
