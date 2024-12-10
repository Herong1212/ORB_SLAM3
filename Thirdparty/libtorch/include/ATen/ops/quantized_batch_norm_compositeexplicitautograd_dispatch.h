#pragma once
// @generated by torchgen/gen.py from DispatchKeyFunction.h

// NB: The implementing C++ file is RegisterDispatchKey.cpp

// The only #includes we need are for custom classes that have defaults in the C++ API
#include <c10/core/MemoryFormat.h>
#include <c10/core/Scalar.h>
#include <ATen/core/Reduction.h>

// Forward declarations of any types needed in the operator signatures.
// We can't directly include these classes because it will cause circular include dependencies.
// This file is included by TensorBody.h, which defines the Tensor class.
#include <ATen/core/ATen_fwd.h>

namespace at {

namespace compositeexplicitautograd {

TORCH_API at::Tensor & quantized_batch_norm_out(at::Tensor & out, const at::Tensor & input, const ::std::optional<at::Tensor> & weight, const ::std::optional<at::Tensor> & bias, const at::Tensor & mean, const at::Tensor & var, double eps, double output_scale, int64_t output_zero_point);
TORCH_API at::Tensor & quantized_batch_norm_outf(const at::Tensor & input, const ::std::optional<at::Tensor> & weight, const ::std::optional<at::Tensor> & bias, const at::Tensor & mean, const at::Tensor & var, double eps, double output_scale, int64_t output_zero_point, at::Tensor & out);

} // namespace compositeexplicitautograd
} // namespace at
