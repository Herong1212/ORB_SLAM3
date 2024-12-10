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
TORCH_API at::Tensor & _to_sparse_csc_out(const at::Tensor & self, ::std::optional<int64_t> dense_dim, at::Tensor & out);
TORCH_API at::Tensor dense_to_sparse_csc(const at::Tensor & self, ::std::optional<int64_t> dense_dim=::std::nullopt);
TORCH_API at::Tensor coo_to_sparse_csc(const at::Tensor & self, ::std::optional<int64_t> dense_dim=::std::nullopt);
TORCH_API at::Tensor sparse_compressed_to_sparse_csc(const at::Tensor & self, ::std::optional<int64_t> dense_dim=::std::nullopt);
} // namespace native
} // namespace at
