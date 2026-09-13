#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <libgnc/math/vector.hpp>

namespace py = pybind11;

namespace pybind11::detail {
    template<typename T, size_t N>
    struct type_caster<gnc::Vec<T, N>> {
        using VecT = gnc::Vec<T, N>;

        PYBIND11_TYPE_CASTER(VecT, _("Vec"));

        bool load(handle src, bool) {
            if (!py::isinstance<py::sequence>(src)) return false;

            py::sequence seq = py::reinterpret_borrow<py::sequence>(src);

            if (seq.size() != N) return false;

            VecT vec;

            for (size_t i=0; i<N; ++i) {
                vec[i] = py::cast<T>(seq[i]);
            }

            value = std::move(vec);

            return true;
        }

        static handle cast(const VecT& vec, return_value_policy, handle) {
            py::list out;

            for (size_t i=0; i<N; ++i) {
                out.append(vec[i]);
            }

            return out.release();
        }
    };
}