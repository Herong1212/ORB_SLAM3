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



#include <ATen/ops/rrelu_ops.h>

namespace at {


// aten::rrelu(Tensor self, Scalar lower=0.125, Scalar upper=0.3333333333333333, bool training=False, Generator? generator=None) -> Tensor
inline at::Tensor rrelu(const at::Tensor & self, const at::Scalar & lower=0.125, const at::Scalar & upper=0.3333333333333333, bool training=false, ::std::optional<at::Generator> generator=::std::nullopt) {
    return at::_ops::rrelu::call(self, lower, upper, training, generator);
}

// aten::rrelu_(Tensor(a!) self, Scalar lower=0.125, Scalar upper=0.3333333333333333, bool training=False, Generator? generator=None) -> Tensor(a!)
inline at::Tensor & rrelu_(at::Tensor & self, const at::Scalar & lower=0.125, const at::Scalar & upper=0.3333333333333333, bool training=false, ::std::optional<at::Generator> generator=::std::nullopt) {
    return at::_ops::rrelu_::call(self, lower, upper, training, generator);
}

}