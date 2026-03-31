# Frame Tags and Compile-Time Safety

Frame mismatches are among the most common and insidious bugs in robotics
software. Multiplying transformations with incompatible frames compiles
without errors but produces silently wrong results. liepp uses C++ template
parameters to encode reference frame information in the type system, catching
frame composition errors at compile time with zero runtime overhead.

## The Problem

In traditional robotics code, transformations are plain matrices or SE(3)
objects with no frame information:

```cpp
se3<double> T_world_base = /* ... */;
se3<double> T_tool_base  = /* ... */;

// Compiles fine, but frames don't match!
auto T_wrong = T_world_base * T_tool_base;
//              {world->base} * {tool->base}  -- incompatible!
```

The correct composition requires the inverse:
`T_world_base * T_base_tool`. But nothing in the type system prevents the
first version from compiling. The resulting bug may manifest as a robot moving
to the wrong position -- potentially dangerous in physical systems.

These frame mismatch bugs are:
- **Hard to detect:** The code compiles, runs, and produces plausible-looking
  (but wrong) $4 \times 4$ matrices
- **Hard to trace:** The error propagates through chains of transformations
  before manifesting as incorrect behavior
- **Common:** Any non-trivial robotics system juggles world, base, tool, camera,
  sensor, and workpiece frames

## liepp's Solution

liepp wraps SO(3) and SE(3) in frame-tagged templates where the `From` and
`To` frames are type parameters:

```cpp
template <typename From, typename To, typename Scalar = double>
struct transform
{
    se3<Scalar> m_value;

    // Only compiles when To == rhs.From
    template <typename C>
    auto operator*(const transform<To, C, Scalar>& rhs) const
        -> transform<From, C, Scalar>;

    // Inverse flips frames
    transform<To, From, Scalar> inverse() const;
};
```

The `operator*` template constraint ensures that composition is only defined
when the `To` frame of the left operand matches the `From` frame of the right
operand. Mismatches are compile errors.

### The Fix in Practice

```cpp
struct World {};
struct Base {};
struct Tool {};

transform<World, Base> T_wb = /* ... */;
transform<Tool, Base>  T_tb = /* ... */;

// Compile ERROR: transform<World,Base> * transform<Tool,Base>
//                Base != Tool (To of left != From of right)
// auto T_wrong = T_wb * T_tb;

// Correct: compose via the inverse
transform<Base, Tool> T_bt = T_tb.inverse();
auto T_wt = T_wb * T_bt;  // transform<World, Tool> -- OK
```

The compiler catches the frame mismatch before the code ever runs.

## Frame Tag Types

Frame tags are user-defined empty structs. They carry no data and exist purely
in the type system:

```cpp
struct World {};
struct Base {};
struct Flange {};
struct Camera {};
struct Workpiece {};
```

Because they are empty types, frame tags impose **zero runtime cost**:
- `sizeof(transform<World, Base>)` equals `sizeof(se3<double>)`
- No virtual dispatch, no RTTI, no string comparisons
- The tags are erased by the compiler after type checking

### Flexible Tag Design

Frame tags can be any type -- empty structs are the convention, but enum
classes, integral non-type template parameter wrappers, or any other type
work equally well:

```cpp
enum class Frame { world, base, tool };

// Using enum wrapper (not shown: requires NTTP adapter)
transform<Frame::world, Frame::base> T_wb;
```

## Composition Rules

liepp enforces the standard rules of frame composition through template
constraints:

### Transform Composition

```cpp
transform<A, B> * transform<B, C> = transform<A, C>   // OK
transform<A, B> * transform<D, C>                      // Compile error (B != D)
```

### Inverse

```cpp
transform<A, B>.inverse() = transform<B, A>
```

### Rotation Composition

The same rules apply to `rotation<From, To>`:

```cpp
rotation<A, B> * rotation<B, C> = rotation<A, C>       // OK
rotation<A, B>.inverse() = rotation<B, A>
```

### Identity

```cpp
transform<A, A>::identity()   // Identity in frame A
rotation<A, A>::identity()    // Identity rotation in frame A
```

### Chain of Compositions

Frame tags enable long composition chains where each intermediate frame is
verified:

```cpp
transform<World, Base>    T_wb;
transform<Base, Shoulder> T_bs;
transform<Shoulder, Elbow> T_se;
transform<Elbow, Wrist>   T_ew;
transform<Wrist, Tool>    T_wt;

// Entire chain type-checked at compile time
auto T_world_tool = T_wb * T_bs * T_se * T_ew * T_wt;
// Result type: transform<World, Tool>
```

## Framed Twists and Wrenches

liepp extends frame safety to twists and wrenches:

- **`framed_twist<Frame>`**: A spatial or body twist expressed in a specific
  reference frame. Prevents accidentally adding twists from different frames.

- **`framed_wrench<Frame>`**: A wrench expressed in a specific frame.

The frame tag ensures that twist-wrench duality is respected:
$\mathcal{F}^\top \mathcal{V}$ (power) only makes sense when both are
expressed in the same frame.

## Design Trade-offs

### When to Use Frame Tags

Frame tags are most valuable when:
- Multiple reference frames are in play (multi-robot systems, hand-eye calibration)
- Safety is critical (surgical robots, collaborative robots)
- Code is maintained by a team (frame conventions become self-documenting)
- Transforms are composed in complex chains

### When to Use Plain se3/so3

Frame tags add template parameter complexity. Plain `se3<Scalar>` /
`so3<Scalar>` are preferred when:
- Working within a single frame (pure Lie group computations)
- Writing generic algorithms that operate on any transformation
- Interfacing with external libraries that use raw matrices
- Frame semantics are clear from context (e.g., internal FK computation)

liepp's own kinematics engine uses plain `se3<Scalar>` internally -- the
chain model and FK produce untagged transformations. Frame tags are applied at
the application boundary where multiple frames interact.

### Zero-Cost Abstraction

The critical design property is that frame tags are a **zero-cost abstraction**
in the C++ sense:
- They improve safety without runtime penalty
- `transform<A, B>` is an aggregate struct wrapping `se3<Scalar>` with public
  `m_value` -- the compiler generates identical machine code
- Template instantiation happens at compile time; the optimizer sees through
  the wrapper completely

See [API Reference](../api/frames.md) for the complete `rotation` and
`transform` interfaces.

## Bibliography

[1] K. M. Lynch and F. C. Park, "Modern Robotics: Mechanics, Planning, and
Control," Cambridge University Press, 2017.

[2] A. Alexandrescu, "Modern C++ Design: Generic Programming and Design
Patterns Applied," Addison-Wesley, 2001.
