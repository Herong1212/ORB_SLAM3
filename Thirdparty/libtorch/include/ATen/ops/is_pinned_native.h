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
TORCH_API bool is_pinned(const at::Tensor & self, ::std::optional<at::Device> device=::std::nullopt);
TORCH_API bool is_pinned_sparse_coo(const at::Tensor & self, ::std::optional<at::Device> device=::std::nullopt);
TORCH_API bool is_pinned_sparse_compressed(const at::Tensor & self, ::std::optional<at::Device> device=::std::nullopt);
} // namespace native
} // namespace at