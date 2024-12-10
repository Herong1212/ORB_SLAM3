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



#include <ATen/ops/to_mkldnn_ops.h>

namespace at {


// aten::to_mkldnn.out(Tensor self, ScalarType? dtype=None, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & to_mkldnn_out(at::Tensor & out, const at::Tensor & self, ::std::optional<at::ScalarType> dtype=::std::nullopt) {
    return at::_ops::to_mkldnn_out::call(self, dtype, out);
}
// aten::to_mkldnn.out(Tensor self, ScalarType? dtype=None, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & to_mkldnn_outf(const at::Tensor & self, ::std::optional<at::ScalarType> dtype, at::Tensor & out) {
    return at::_ops::to_mkldnn_out::call(self, dtype, out);
}

}
