# Using Frame Tags for Type-Safe Transforms

This guide shows how to use Cartan's compile-time frame safety system. Frame tags prevent accidental composition of transforms between incompatible coordinate frames -- errors that are notoriously hard to catch at runtime but trivial to prevent at compile time.

**Prerequisites:** Familiarity with SE(3) rigid body transforms. See the [Lie Basics example](../../examples/lie_basics.cpp) for SE(3) fundamentals.

## 1. Defining Frame Tags

Frame tags are empty structs -- they exist only as type-level markers with zero runtime cost:

```cpp
struct World {};
struct Camera {};
struct Tool {};
struct Base {};
```

These tags carry no data. They serve purely as compile-time identifiers for coordinate frames. You define them once in your application code and use them to annotate every transform.

## 2. Creating Framed Transforms

A `transform<From, To>` wraps an `se3` value with frame annotations:

```cpp
#include <cartan/frames/transform.h>

// Transform from World frame to Camera frame
cartan::transform<World, Camera> T_wc{
    cartan::se3<double>(
        cartan::so3<double>::identity(),
        cartan::vector3<double>(0, 0, 1.5))
};

// Identity transform
auto T_id = cartan::transform<World, World>::identity();

// From a 4x4 matrix (with validation)
Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
auto T_result = cartan::transform<World, Camera>::from_matrix(mat);
if (T_result.has_value())
{
    cartan::transform<World, Camera> T = T_result.value();
}
```

The template parameters read as "transforms **from** the first frame **to** the second frame." So `transform<World, Camera>` takes points expressed in Camera and maps them to World.

## 3. Composition

Transforms compose with `operator*`, but **only when frames match**:

```cpp
cartan::transform<World, Camera> T_wc{ /* ... */ };
cartan::transform<Camera, Tool>  T_ct{ /* ... */ };

// Valid: World <- Camera <- Tool => World <- Tool
auto T_wt = T_wc * T_ct;   // transform<World, Tool>

// COMPILE ERROR: Camera != Tool, frames don't chain
// auto bad = T_wc * T_wt;  // transform<World, Camera> * transform<World, Tool>
```

The compiler enforces that the `To` frame of the left operand matches the `From` frame of the right operand. This catches frame-mismatch bugs that would otherwise produce silently wrong results.

Multi-step composition works naturally:

```cpp
cartan::transform<World, Base>   T_wb{ /* ... */ };
cartan::transform<Base, Camera>  T_bc{ /* ... */ };
cartan::transform<Camera, Tool>  T_ct{ /* ... */ };

// Chains correctly: World <- Base <- Camera <- Tool
auto T_wt = T_wb * T_bc * T_ct;  // transform<World, Tool>
```

## 4. Inverse

Inverting a transform flips the frame tags:

```cpp
cartan::transform<World, Camera> T_wc{ /* ... */ };

// Inverse: Camera <- World
auto T_cw = T_wc.inverse();  // transform<Camera, World>

// Round-trip: should be identity
auto T_id = T_wc * T_cw;     // transform<World, World>
```

This is mathematically correct: if T maps Camera -> World, then T^{-1} maps World -> Camera.

## 5. Accessing the Inner SE(3)

When you need the raw `se3` value for math operations or interop with untagged code, use `.m_value`:

```cpp
cartan::transform<World, Camera> T_wc{ /* ... */ };

// Access the underlying se3
cartan::se3<double> raw = T_wc.m_value;

// Use se3 methods through the wrapper
Eigen::Matrix4d mat = T_wc.matrix();
auto pos = T_wc.translation();
auto rot = T_wc.rotation();
auto twist = T_wc.log();

// Transform a point
cartan::vector3<double> p_camera(0.1, 0.2, 0.3);
auto p_world = T_wc.act(p_camera);
```

The `m_value` member is public (aggregate struct), so bridging between framed and unframed code is always possible.

## 6. Framed Rotations

The same pattern applies to pure rotations with `rotation<From, To>`:

```cpp
#include <cartan/frames/rotation.h>

struct Imu {};
struct Body {};

// Rotation from IMU frame to Body frame
cartan::rotation<Imu, Body> R_ib{cartan::so3<double>::exp(
    cartan::vector3<double>(0.01, -0.02, 0.0))};

// Composition, inverse -- same rules as transform
auto R_bi = R_ib.inverse();            // rotation<Body, Imu>
auto R_id = R_ib * R_bi;               // rotation<Imu, Imu>

// Access underlying so3
cartan::so3<double> raw = R_ib.m_value;
auto axis_angle = R_ib.log();
auto R_mat = R_ib.matrix();
```

## 7. Framed Twists and Wrenches

Velocity twists and force wrenches are tagged with a **single** frame (the frame in which they are expressed):

```cpp
#include <cartan/frames/framed_twist.h>
#include <cartan/frames/framed_wrench.h>

// Spatial twist expressed in the World frame
cartan::framed_twist<World> V_world = cartan::framed_twist<World>::from_vector(
    cartan::vector6<double>{{0, 0, 0.1, 0.5, 0, 0}});

// Access components
auto omega = V_world.omega();  // angular velocity
auto v = V_world.v();          // linear velocity

// Transform twist between frames using the adjoint map
cartan::transform<World, Camera> T_wc{ /* ... */ };
auto V_camera = cartan::adjoint_map(T_wc.inverse(), V_world);
// V_camera is framed_twist<Camera>

// Wrench expressed in the Tool frame
auto W_tool = cartan::framed_wrench<Tool>::from_moment_force(
    cartan::vector3<double>(0, 0, 0.5),   // moment
    cartan::vector3<double>(0, 0, -9.81)); // force

// Transform wrench using coadjoint map
cartan::transform<World, Tool> T_wt{ /* ... */ };
auto W_world = cartan::coadjoint_map(T_wt, W_tool);
// W_world is framed_wrench<World>
```

## 8. When to Use Frame Tags

**Use frame tags in application code** where frame correctness matters:

- Robot control loops (World, Base, Tool, Camera, Sensor frames)
- Sensor fusion (IMU, GPS, Visual odometry frames)
- Multi-robot systems (Robot1, Robot2, SharedMap frames)
- Calibration pipelines (Hand, Eye, Target frames)

**Skip frame tags in library internals** and math-heavy code:

- Inside IK steppers (work directly with `se3`)
- Numerical algorithms that don't care about frame semantics
- Performance-critical inner loops where the template instantiation overhead is undesirable

**Bridge between framed and unframed code** using `.m_value`:

```cpp
// Application layer: framed
cartan::transform<World, Tool> T_wt = /* ... */;

// Pass to library function expecting raw se3
auto fk = some_library_function(T_wt.m_value);

// Wrap result back into framed transform
cartan::transform<World, Tool> result{fk};
```

## Further Reading

- [Frame Tags Design](../background/frame-tags.md) -- design rationale, zero-overhead proof, and comparison with alternative approaches
- [API Reference: Frames](../api/frames.md) -- full API for transform, rotation, framed_twist, and framed_wrench
