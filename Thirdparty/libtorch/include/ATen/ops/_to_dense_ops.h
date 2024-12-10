#pragma once

// @generated by torchgen/gen.py from Operator.h

#include <tuple>
#include <vector>

// Forward declarations of any types needed in the operator signatures.
// We can't directly include these classes because it will cause circular include dependencies.
// This file is included by TensorBody.h, which defines the Tensor class.
#include <ATen/core/ATen_fwd.h>

namespace at {
namespace _ops {


struct TORCH_API _to_dense {
  using schema = at::Tensor (const at::Tensor &, ::std::optional<at::ScalarType>, ::std::optional<bool>);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::_to_dense")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "_to_dense(Tensor self, ScalarType? dtype=None, bool? masked_grad=None) -> Tensor")
  static at::Tensor call(const at::Tensor & self, ::std::optional<at::ScalarType> dtype, ::std::optional<bool> masked_grad);
  static at::Tensor redispatch(c10::DispatchKeySet dispatchKeySet, const at::Tensor & self, ::std::optional<at::ScalarType> dtype, ::std::optional<bool> masked_grad);
};

struct TORCH_API _to_dense_out {
  using schema = at::Tensor & (const at::Tensor &, ::std::optional<at::ScalarType>, ::std::optional<bool>, at::Tensor &);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::_to_dense")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "out")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "_to_dense.out(Tensor self, ScalarType? dtype=None, bool? masked_grad=None, *, Tensor(a!) out) -> Tensor(a!)")
  static at::Tensor & call(const at::Tensor & self, ::std::optional<at::ScalarType> dtype, ::std::optional<bool> masked_grad, at::Tensor & out);
  static at::Tensor & redispatch(c10::DispatchKeySet dispatchKeySet, const at::Tensor & self, ::std::optional<at::ScalarType> dtype, ::std::optional<bool> masked_grad, at::Tensor & out);
};

}} // namespace at::_ops
