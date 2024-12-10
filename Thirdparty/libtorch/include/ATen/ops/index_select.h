#pragma once

// @generated by torchgen/gen.py from Function.h

#include <ATen/Context.h>
#include <ATen/DeviceGuard.h>
#include <ATen/TensorUtils.h>
#include <ATen/TracerMode.h>
#include <ATen/core/Generator.h>
#include <ATen/core/Reduction.h>
#include <ATen/core/Tensor.h>
#include <c10/core/Scalar.h>
#include <c10/core/Storage.h>
#include <c10/core/TensorOptions.h>
#include <c10/util/Deprecated.h>
#include <optional>



#include <ATen/ops/index_select_ops.h>

namespace at {


// aten::index_select.out(Tensor self, int dim, Tensor index, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & index_select_out(at::Tensor & out, const at::Tensor & self, int64_t dim, const at::Tensor & index) {
    return at::_ops::index_select_out::call(self, dim, index, out);
}
// aten::index_select.out(Tensor self, int dim, Tensor index, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & index_select_outf(const at::Tensor & self, int64_t dim, const at::Tensor & index, at::Tensor & out) {
    return at::_ops::index_select_out::call(self, dim, index, out);
}

// aten::index_select(Tensor self, int dim, Tensor index) -> Tensor
inline at::Tensor index_select(const at::Tensor & self, int64_t dim, const at::Tensor & index) {
    return at::_ops::index_select::call(self, dim, index);
}

// aten::index_select.dimname_out(Tensor self, Dimname dim, Tensor index, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & index_select_out(at::Tensor & out, const at::Tensor & self, at::Dimname dim, const at::Tensor & index) {
    return at::_ops::index_select_dimname_out::call(self, dim, index, out);
}
// aten::index_select.dimname_out(Tensor self, Dimname dim, Tensor index, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & index_select_outf(const at::Tensor & self, at::Dimname dim, const at::Tensor & index, at::Tensor & out) {
    return at::_ops::index_select_dimname_out::call(self, dim, index, out);
}

// aten::index_select.dimname(Tensor self, Dimname dim, Tensor index) -> Tensor
inline at::Tensor index_select(const at::Tensor & self, at::Dimname dim, const at::Tensor & index) {
    return at::_ops::index_select_dimname::call(self, dim, index);
}

}
