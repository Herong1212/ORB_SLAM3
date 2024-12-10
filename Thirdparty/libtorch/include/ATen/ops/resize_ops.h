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


struct TORCH_API resize_ {
  using schema = const at::Tensor & (const at::Tensor &, c10::SymIntArrayRef, ::std::optional<at::MemoryFormat>);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::resize_")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "resize_(Tensor(a!) self, SymInt[] size, *, MemoryFormat? memory_format=None) -> Tensor(a!)")
  static const at::Tensor & call(const at::Tensor & self, c10::SymIntArrayRef size, ::std::optional<at::MemoryFormat> memory_format);
  static const at::Tensor & redispatch(c10::DispatchKeySet dispatchKeySet, const at::Tensor & self, c10::SymIntArrayRef size, ::std::optional<at::MemoryFormat> memory_format);
};

struct TORCH_API resize_out {
  using schema = const at::Tensor & (const at::Tensor &, c10::SymIntArrayRef, ::std::optional<at::MemoryFormat>, const at::Tensor &);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::resize")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "out")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "resize.out(Tensor self, SymInt[] size, *, MemoryFormat? memory_format=None, Tensor(a!) out) -> Tensor(a!)")
  static const at::Tensor & call(const at::Tensor & self, c10::SymIntArrayRef size, ::std::optional<at::MemoryFormat> memory_format, const at::Tensor & out);
  static const at::Tensor & redispatch(c10::DispatchKeySet dispatchKeySet, const at::Tensor & self, c10::SymIntArrayRef size, ::std::optional<at::MemoryFormat> memory_format, const at::Tensor & out);
};

struct TORCH_API resize {
  using schema = at::Tensor (const at::Tensor &, c10::SymIntArrayRef, ::std::optional<at::MemoryFormat>);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::resize")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "resize(Tensor self, SymInt[] size, *, MemoryFormat? memory_format=None) -> Tensor")
  static at::Tensor call(const at::Tensor & self, c10::SymIntArrayRef size, ::std::optional<at::MemoryFormat> memory_format);
  static at::Tensor redispatch(c10::DispatchKeySet dispatchKeySet, const at::Tensor & self, c10::SymIntArrayRef size, ::std::optional<at::MemoryFormat> memory_format);
};

}} // namespace at::_ops
