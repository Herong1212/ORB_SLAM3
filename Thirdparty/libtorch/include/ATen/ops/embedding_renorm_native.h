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
TORCH_API at::Tensor embedding_renorm(const at::Tensor & self, const at::Tensor & indices, double max_norm, double norm_type);
TORCH_API at::Tensor & embedding_renorm_out(const at::Tensor & self, const at::Tensor & indices, double max_norm, double norm_type, at::Tensor & out);
TORCH_API at::Tensor & embedding_renorm_cpu_(at::Tensor & self, const at::Tensor & indices, double max_norm, double norm_type);
TORCH_API at::Tensor & embedding_renorm_cuda_(at::Tensor & self, const at::Tensor & indices, double max_norm, double norm_type);
} // namespace native
} // namespace at