#include "cartan/lie/so3.h"
#include "cartan/lie/se3.h"
#include "cartan/lie/lie_failure.h"

#include "registrations.h"
#include "detail/expected_caster.h"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace
{

using SO3d = cartan::so3<double>;
using SE3d = cartan::se3<double>;
using Vector3d = cartan::vector3<double>;

}

namespace cartan::python
{

void register_lie(nb::module_& m)
{
    nb::class_<SO3d>(m, "SO3", "3D rotation group element (unit quaternion internal)")
        .def_static("identity", &SO3d::identity)
        .def_static("exp", &SO3d::exp,
                    "Exponential map: rotation vector (3,) -> SO(3).",
                    nb::arg("omega").noconvert())
        .def("log", &SO3d::log,
             "Logarithmic map: SO(3) -> rotation vector (3,).")
        .def("matrix", &SO3d::matrix,
             "Return the 3x3 rotation matrix.")
        .def("inverse", &SO3d::inverse)
        .def("act",
             [](const SO3d& self, const Vector3d& p) -> Vector3d {
                 return self.act(p);
             },
             "Rotate a 3D point.",
             nb::arg("p").noconvert())
        .def("__mul__",
             [](const SO3d& a, const SO3d& b) -> SO3d {
                 return a * b;
             })
        .def_static("from_matrix",
            [](const nb::DRef<const cartan::matrix3<double>>& R) -> SO3d {
                auto r = SO3d::from_matrix(R);
                if (!r) throw nb::value_error(cartan::message(r.error()));
                return *std::move(r);
            },
            "Construct an SO(3) from a 3x3 rotation matrix. "
            "Raises ValueError if R is not orthogonal or has det != 1.",
            nb::arg("R").noconvert())
        .def("adjoint",
            [](const SO3d& self) -> cartan::matrix3<double> {
                return self.matrix();
            },
            "SO(3) adjoint -- equal to the rotation matrix itself "
            "(Lynch & Park Eq. 3.84).");

    nb::class_<SE3d>(m, "SE3", "Rigid-body transformation group element")
        .def_static("identity", &SE3d::identity)
        .def_static("exp", &SE3d::exp,
                    "Exponential map: twist (6,) in (omega, rho) order -> SE(3).",
                    nb::arg("twist").noconvert())
        .def("log", &SE3d::log,
             "Logarithmic map: SE(3) -> twist (6,) in (omega, rho) order.")
        .def("matrix", &SE3d::matrix,
             "Return the 4x4 homogeneous transformation matrix.")
        .def("inverse", &SE3d::inverse)
        .def("adjoint", &SE3d::adjoint,
             "Return the 6x6 adjoint matrix (omega-first twist convention).")
        .def("act",
             [](const SE3d& self, const Vector3d& p) -> Vector3d {
                 return self.act(p);
             },
             "Transform a 3D point: R p + t.",
             nb::arg("p").noconvert())
        .def("__mul__",
             [](const SE3d& a, const SE3d& b) -> SE3d {
                 return a * b;
             })
        .def_static("from_matrix",
            [](const nb::DRef<const cartan::matrix4<double>>& T) -> SE3d {
                auto r = SE3d::from_matrix(T);
                if (!r) throw nb::value_error(cartan::message(r.error()));
                return *std::move(r);
            },
            "Construct an SE(3) from a 4x4 homogeneous transformation matrix. "
            "Raises ValueError if T is not a valid rigid-body transform.",
            nb::arg("T").noconvert())
        .def_prop_ro("translation",
            [](const SE3d& self) { return cartan::vector3<double>(self.translation()); },
            "Translation component as a (3,) numpy array (copy).")
        .def_prop_ro("rotation",
            [](const SE3d& self) { return SO3d(self.rotation()); },
            "Rotation component as a cartan.SO3 (copy).");
}

}
