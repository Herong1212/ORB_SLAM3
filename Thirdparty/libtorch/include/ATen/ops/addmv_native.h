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
#include <ATen/ops/addmv_meta.h>

namespace at {
namespace native {
struct TORCH_API structured_addmv_out_cpu : public at::meta::structured_addmv {
void impl(const at::Tensor & self, const at::Tensor & mat, const at::Tensor & vec, const at::Scalar & beta, const at::Scalar & alpha, const at::Tensor & out);
};
struct TORCH_API structured_addmv_out_cuda : public at::meta::structured_addmv {
void impl(const at::Tensor & self, const at::Tensor & mat, const at::Tensor & vec, const at::Scalar & beta, const at::Scalar & alpha, const at::Tensor & out);
};
TORCH_API at::Tensor & addmv_out_sparse_compressed(const at::Tensor & self, const at::Tensor & mat, const at::Tensor & vec, const at::Scalar & beta, const at::Scalar & alpha, at::Tensor & out);
TORCH_API at::Tensor & addmv_out_sparse_compressed_cuda(const at::Tensor & self, const at::Tensor & mat, const at::Tensor & vec, const at::Scalar & beta, const at::Scalar & alpha, at::Tensor & out);
} // namespace native
} // namespace at
