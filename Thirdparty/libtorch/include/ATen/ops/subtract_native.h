#pragma once

// @generated by torchgen/gen.py from NativeFunction.h

#include <c10/core/Scalar.h>
#include <c10/core/Storage.h>
#include <c10/core/TensorOptions.h>
#include <c10/util/Deprecated.h>
#include <optional>
#include <c10/core/QScheme.h>
#include <ATen/core/Reduction.h>
#include <ATen/core/Tensor.h>
#include <tuple>
#include <vector>


namespace at {
namespace native {
TORCH_API at::Tensor subtract(const at::Tensor & self, const at::Tensor & other, const at::Scalar & alpha=1);
TORCH_API at::Tensor & subtract_out(const at::Tensor & self, const at::Tensor & other, const at::Scalar & alpha, at::Tensor & out);
TORCH_API at::Tensor & subtract_(at::Tensor & self, const at::Tensor & other, const at::Scalar & alpha=1);
TORCH_API at::Tensor subtract(const at::Tensor & self, const at::Scalar & other, const at::Scalar & alpha=1);
TORCH_API at::Tensor & subtract_(at::Tensor & self, const at::Scalar & other, const at::Scalar & alpha=1);
} // namespace native
} // namespace at
