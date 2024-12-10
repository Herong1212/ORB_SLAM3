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
TORCH_API at::Tensor & slice_copy_Tensor_out_symint(const at::Tensor & self, int64_t dim, ::std::optional<c10::SymInt> start, ::std::optional<c10::SymInt> end, c10::SymInt step, at::Tensor & out);
TORCH_API at::Tensor slice_copy_Tensor_symint(const at::Tensor & self, int64_t dim=0, ::std::optional<c10::SymInt> start=::std::nullopt, ::std::optional<c10::SymInt> end=::std::nullopt, c10::SymInt step=1);
} // namespace native
} // namespace at
