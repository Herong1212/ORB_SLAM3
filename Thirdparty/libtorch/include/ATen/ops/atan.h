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



#include <ATen/ops/atan_ops.h>

namespace at {


// aten::atan(Tensor self) -> Tensor
inline at::Tensor atan(const at::Tensor & self) {
    return at::_ops::atan::call(self);
}

// aten::atan_(Tensor(a!) self) -> Tensor(a!)
inline at::Tensor & atan_(at::Tensor & self) {
    return at::_ops::atan_::call(self);
}

// aten::atan.out(Tensor self, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & atan_out(at::Tensor & out, const at::Tensor & self) {
    return at::_ops::atan_out::call(self, out);
}
// aten::atan.out(Tensor self, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & atan_outf(const at::Tensor & self, at::Tensor & out) {
    return at::_ops::atan_out::call(self, out);
}

}
