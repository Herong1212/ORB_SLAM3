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
TORCH_API ::std::vector<at::Tensor> foreach_tensor_clamp_max_scalar_kernel_slow(at::TensorList self, const at::Scalar & scalar);
TORCH_API void _foreach_clamp_max_Scalar_out(at::TensorList self, const at::Scalar & scalar, at::TensorList out);
TORCH_API void foreach_tensor_clamp_max_scalar_kernel_slow_(at::TensorList self, const at::Scalar & scalar);
TORCH_API ::std::vector<at::Tensor> foreach_tensor_clamp_max_scalar_kernel_cuda(at::TensorList self, const at::Scalar & scalar);
TORCH_API void foreach_tensor_clamp_max_scalar_kernel_cuda_(at::TensorList self, const at::Scalar & scalar);
TORCH_API ::std::vector<at::Tensor> foreach_tensor_clamp_max_list_kernel_slow(at::TensorList self, at::TensorList other);
TORCH_API void _foreach_clamp_max_List_out(at::TensorList self, at::TensorList other, at::TensorList out);
TORCH_API void foreach_tensor_clamp_max_list_kernel_slow_(at::TensorList self, at::TensorList other);
TORCH_API ::std::vector<at::Tensor> foreach_tensor_clamp_max_list_kernel_cuda(at::TensorList self, at::TensorList other);
TORCH_API void foreach_tensor_clamp_max_list_kernel_cuda_(at::TensorList self, at::TensorList other);
TORCH_API ::std::vector<at::Tensor> foreach_tensor_clamp_max_scalarlist_kernel_slow(at::TensorList self, at::ArrayRef<at::Scalar> scalars);
TORCH_API void _foreach_clamp_max_ScalarList_out(at::TensorList self, at::ArrayRef<at::Scalar> scalars, at::TensorList out);
TORCH_API void foreach_tensor_clamp_max_scalarlist_kernel_slow_(at::TensorList self, at::ArrayRef<at::Scalar> scalars);
TORCH_API ::std::vector<at::Tensor> foreach_tensor_clamp_max_scalarlist_kernel_cuda(at::TensorList self, at::ArrayRef<at::Scalar> scalars);
TORCH_API void foreach_tensor_clamp_max_scalarlist_kernel_cuda_(at::TensorList self, at::ArrayRef<at::Scalar> scalars);
} // namespace native
} // namespace at
