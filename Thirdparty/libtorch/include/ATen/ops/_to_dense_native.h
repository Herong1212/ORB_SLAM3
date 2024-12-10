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
TORCH_API at::Tensor & _to_dense_out(const at::Tensor & self, ::std::optional<at::ScalarType> dtype, ::std::optional<bool> masked_grad, at::Tensor & out);
TORCH_API at::Tensor sparse_to_dense(const at::Tensor & self, ::std::optional<at::ScalarType> dtype=::std::nullopt, ::std::optional<bool> masked_grad=::std::nullopt);
TORCH_API at::Tensor sparse_compressed_to_dense(const at::Tensor & self, ::std::optional<at::ScalarType> dtype=::std::nullopt, ::std::optional<bool> masked_grad=::std::nullopt);
TORCH_API at::Tensor mkldnn_to_dense(const at::Tensor & self, ::std::optional<at::ScalarType> dtype=::std::nullopt, ::std::optional<bool> masked_grad=::std::nullopt);
} // namespace native
} // namespace at