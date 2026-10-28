# Loading Robot Descriptions

`cartan::load_urdf` reads a robot description off disk and returns the serial
chain it describes. Reading it -- XML, xacro expansion, `package://` and
`$(find ...)` resolution, asset lookup -- is done by
[meios](https://github.com/skrede/meios), which the URDF module links.
Everything after the model is in hand is Cartan's own.

This page covers what you set to load a real robot, what the built-in
expression evaluator covers, and what authority you grant by changing it.

<!-- cartan:preamble -->
```cpp
#include <cartan/urdf.h>

#include <string>
#include <iostream>
```

## Loading a description

`load_urdf<Scalar>` returns `cartan::expected<urdf_load_result<Scalar>,
urdf_error>`. Its options object carries the reader's own options directly
rather than mirroring them, so `args`, `package_roots` and the asset policy are
set on `opts.description`:

<!-- cartan:snippet name=load-ur3e needs=urdf -->
```cpp
cartan::load_options opts;
opts.description.package_roots.push_back("/opt/descriptions");
opts.description.args["ur_type"] = "ur3e";
opts.description.args["name"] = "ur3e";

const std::string path = "/opt/descriptions/ur_description/urdf/ur.urdf.xacro";
auto loaded = cartan::load_urdf<double>(path, opts);
if (!loaded)
{
    std::cerr << loaded.error().detail << "\n";
    return 1;
}

std::cout << loaded->chain.num_joints() << " joints\n";
```

The description above is `ur_description` from
[UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
at `4.3.1`; `package_roots` names the directory that package sits in, so
`package://ur_description/...` resolves under it.

Both `args` entries are load-bearing. Without `ur_type` the description falls
back to its own default, `ur5x`, which is a placeholder directory shipping no
`config/`, and the load then fails on a joint-limits file that is not there --
`unresolved_asset`, naming the full path under `config/ur5x/` it could not
reach. The fix is the missing argument, not your `package_roots`.

`Scalar` may be `float` as well as `double`. A description whose numbers are
good `double`s but overflow the narrower type is refused rather than quietly
turned into an infinity, and the refusal names both the joint and the field it
could not hold.

### The asset policy

Cartan sets `on_missing` to `warn` by default, where the reader's own default
is `fail`. A kinematic chain reads no meshes and no textures, so refusing a
description over a visual asset that is not on disk would reject descriptions
that ship without one.

That relaxation is narrow, and worth being precise about: of the reader's
asset-failure reports, only "the file is not there" consults the policy. A
malformed asset URI, a URI in a scheme the reader does not serve, a native
filesystem error carrying an `std::error_code`, and a resource resolving
outside the configured containment roots all stay hard errors under `warn`.
Every structural failure stays hard too -- an include that does not resolve, a
joint naming a link that was not declared, a topology that is not a tree. So
this is not a strictness dial, and it is not a tradeoff.

## A description with no movable joint

A description whose joints are all fixed -- a sensor bracket, a tool adapter, an
assembly bolted together -- poses no inverse-kinematics problem, and `load_urdf`
refuses it with `urdf_failure::no_movable_joint` rather than handing back a chain
with no joints. What such a description does answer is the rigid transform from
its base link to its tool link, which is what `load_urdf_transform` returns:

<!-- cartan:snippet name=load-fixed-assembly-transform needs=urdf -->
```cpp
auto bracket = cartan::load_urdf_transform<double>("sensor_bracket.urdf");
if (!bracket)
{
    std::cerr << bracket.error().detail << "\n";
    return 1;
}

std::cout << bracket->translation().transpose() << "\n";
```

Its scope is the shape the extractor walks: one root, and one leaf after the
fixed-joint merge. A description that branches is refused on this route too, with
`branched_kinematic_tree`, because the transform is not defined when the merge
leaves several leaves to choose between. A description with mobile joints reads
here as well, where the transform is the chain's home pose.

It answers no named intermediate frame. No intermediate frame survives loading,
for any description -- the metadata carries the two endpoint names, the joint
names and the per-link inertials, and no pose -- so `load_urdf_transform` takes a
path and the load options and nothing else.

The Python extension carries it as `cartan.load_urdf_transform`, which returns a
`cartan.SE3` and raises `cartan.UrdfError` on the same refusals.

## Evaluation backends

A xacro description carries expressions, and which evaluator expands them is
the security-relevant choice on this page.

### The default: the built-in evaluator

Leaving `opts.description.backend` unset selects the reader's built-in
evaluator. Cartan never sets it, and no Cartan configuration builds the
optional Python evaluation component, so this is both what you get by omission
and the only evaluator a Cartan build carries.

It needs nothing outside the C++ standard library, and the expressions it
evaluates can reach neither the filesystem, the network, nor the process: the
grammar has no `open` and no way to name one. It handles `<xacro:arg>`,
`<xacro:property>`, `<xacro:macro>`, `<xacro:include>`, a numeric and boolean
expression grammar, and the auxiliary-document lookup a description uses to
read its joint limits out of a YAML sidecar. That lookup is the one thing an
expression touches outside itself, and the reading is done by C++, which
resolves and contains the path before the evaluator sees a byte of it.

That is enough for real industrial arms end to end. The `kuka_kr6_support`
description from
[ros-industrial/kuka_experimental](https://github.com/ros-industrial/kuka_experimental)
(`melodic-devel`) loads under it with 10 links, 9 joints and no diagnostics,
and `ur.urdf.xacro` from `ur_description` at `4.3.1`, resolved for the UR3e,
loads with 13 links, 12 joints and no diagnostics.

What it does not handle is Python the closed grammar does not contain: a named
conversion, a string method other than the named split, a method call on a
loaded mapping, string repetition or ordering, or a unit tag written over
anything but a numeric literal. A description reaching for one of those is
refused with `unsupported_expression` naming the file, line and column -- a
`${float(x)}` conversion, for one, where the arithmetic form of the same
expression evaluates. A construct the grammar cannot parse at all, a
comprehension or an f-string among them, is refused with `expression_error`
instead, so a caller branching on the code sees both.

### The restricted Python backend

`meios::python_evaluator`, declared in `meios/eval/python_evaluator.h`, drives
an embedded CPython over a restricted expression subset. It exists for a
description that needs the Python behavior the closed grammar does not carry --
the constructs listed above. Reaching it takes a meios built with
`MEIOS_EVAL_PYTHON_SUPPORT=ON`, which drives a *found*, never fetched, CPython
and which Cartan's own acquisition pins off, so this is a build you configure
deliberately -- and an explicit assignment:

<!-- cartan:unbuilt kind=illustration reason="constructs a type from the optional meios evaluation component, which no cartan configuration builds" -->
```cpp
#include <meios/eval/python_evaluator.h>
#include <meios/xacro/evaluator_handle.h>

cartan::load_options opts;
opts.description.backend =
    std::make_shared<meios::evaluator_handle>(meios::python_evaluator{});
```

The trust boundary you accept is that expressions written in the description
are evaluated by an interpreter inside your process. The backend refuses the
constructs that reach out of an expression under four named rules --
`dunder-identifier`, `non-allowlisted-builtin`, `format-traversal` and
`uncontained-yaml-path` -- each with a `file:line` diagnostic naming the rule
that refused it.

Two limits arrive with the interpreter rather than with those rules. The
built-in evaluator's execution ceilings do not apply, so an expression like
`${10**10**10}` is bounded by nothing. And the module's embedded-interpreter
link edge cannot be re-resolved from an install tree, so a meios you installed
does not carry this backend at all; it has to be built and linked in-tree.

### The unrestricted Python backend

`meios::unrestricted_python_evaluator` applies none of those rules. **An
expression inside a description evaluates with the process's own authority** --
`${__import__('os').getcwd()}` runs -- and that covers every description an
include pulls in, including ones from packages you did not write.

<!-- cartan:unbuilt kind=illustration reason="constructs a type from the optional meios evaluation component, which no cartan configuration builds" -->
```cpp
#include <meios/eval/unrestricted_python_evaluator.h>
#include <meios/xacro/evaluator_handle.h>

cartan::load_options opts;
opts.description.backend =
    std::make_shared<meios::evaluator_handle>(meios::unrestricted_python_evaluator{});
```

Reach for it only for a particular description file you would be willing to run
as a script. It is not what the Universal Robots family needs: the built-in
evaluator resolves those descriptions unaided -- the 13 links, 12 joints and no
diagnostics above, with no interpreter in the process at all -- so widening the
authority buys nothing there.

## Two routes to a loadable description

Runtime evaluation is what the sections above describe: the description stays
as authored, and the evaluator runs inside the robot application.

Build-time flattening is the alternative. The description is expanded and
resolved once during the build and written out as a plain document, which the
built-in evaluator then handles with no backend at all. Host-language
evaluation moves into a build tool, where it runs on inputs a build already
trusts, and the deployed application carries no interpreter. meios ships
`meios_target_flatten_resource()` for this, alongside `meios_declare_resource()`
and `meios_target_deploy_resources()`:

```cmake
include(MeiosFlattenResource)

meios_target_flatten_resource(my_app
    RESOURCE ur
    INPUT    ur_description/urdf/ur.urdf.xacro
    OUTPUT   urdf/ur3e.urdf
    ARGS     ur_type:=ur3e name:=ur3e)
```

Two things to know before choosing it. `RESOURCE` names a resource declared
with `meios_declare_resource()` rather than a path, and `ARGS` takes the
command line's `key:=value` spelling -- `key=value` is refused. The rule runs
meios's command-line tool, which a cross-compiling build cannot execute;
`MEIOS_CLI_EXECUTABLE` points it at a host-runnable binary there. Nothing in
Cartan's build or test suite depends on this route.

The tradeoff is when the description is resolved: flattening pins it at build
time, so changing an argument or a configuration file means rebuilding, while
runtime evaluation reads whatever is on disk when the application starts.

## Diagnostics

A load that succeeds still reports. `urdf_load_result::diagnostics` holds the
records the reader produced, each tiered `error`, `warn` or `info`, each
carrying the reader's own code and, where the reader had one, a file, line and
element:

<!-- cartan:snippet name=read-diagnostics needs=urdf -->
```cpp
auto loaded = cartan::load_urdf<double>("robot.urdf");
if (!loaded)
{
    std::cerr << loaded.error().detail << "\n";
    return 1;
}

for (const cartan::urdf_diagnostic& record : loaded->diagnostics)
{
    if (record.severity != cartan::urdf_severity::warn) { continue; }
    std::cerr << meios::to_string(record.meios_code) << ": " << record.message << "\n";
}
```

A warn record on a successful load means either that an asset did not resolve,
which is the policy above doing what it was set to do, or that the reader met
an element it does not recognize.

`urdf_load_result::claims` carries the reader's completeness assertions
(`parsed`, `topology_valid`, `deployment_complete`). They are informational and
nothing in the loader branches on them, deliberately: a description that yields
a correct chain can still withhold `parsed` over an element the reader did not
recognize somewhere it does not affect kinematics. The Franka Panda description
does exactly that: it loads, and its chain is the one the study measures, but it
carries 28 warn records for unrecognized attributes and claims `topology_valid`
and `deployment_complete` without `parsed`.

A failure arrives as `urdf_error`. Its `kind` is Cartan's own taxonomy, its
`meios_code` is the reader's code where the reader raised the failure, and its
`location` names the file, line and element for anything caught while reading.

The Python extension carries the same three surfaces under the names
`cartan.UrdfError`, `cartan.UrdfDiagnostic` and `cartan.UrdfFailure`; see
[Python](../python.md).

## Further Reading

- [PoE Walkthrough](poe-walkthrough.md) -- building a chain by hand, once you have one loaded
- [API Reference: Chain](../api/chain.md) -- the `kinematic_chain` a load produces
- [meios evaluation contract](https://github.com/skrede/meios/blob/master/docs/evaluation.md) -- the expression subset, every refusal rule, and the resource helper
