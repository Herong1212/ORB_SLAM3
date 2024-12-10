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


struct TORCH_API _nested_view_from_jagged {
  using schema = at::Tensor (const at::Tensor &, const at::Tensor &, const at::Tensor &, const ::std::optional<at::Tensor> &, int64_t, const ::std::optional<at::Tensor> &, const ::std::optional<at::Tensor> &);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::_nested_view_from_jagged")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "_nested_view_from_jagged(Tensor(a) self, Tensor offsets, Tensor dummy, Tensor? lengths=None, int ragged_idx=1, Tensor? min_seqlen=None, Tensor? max_seqlen=None) -> Tensor(a)")
  static at::Tensor call(const at::Tensor & self, const at::Tensor & offsets, const at::Tensor & dummy, const ::std::optional<at::Tensor> & lengths, int64_t ragged_idx, const ::std::optional<at::Tensor> & min_seqlen, const ::std::optional<at::Tensor> & max_seqlen);
  static at::Tensor redispatch(c10::DispatchKeySet dispatchKeySet, const at::Tensor & self, const at::Tensor & offsets, const at::Tensor & dummy, const ::std::optional<at::Tensor> & lengths, int64_t ragged_idx, const ::std::optional<at::Tensor> & min_seqlen, const ::std::optional<at::Tensor> & max_seqlen);
};

}} // namespace at::_ops