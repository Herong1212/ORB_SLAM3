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
TORCH_API ::std::tuple<at::Tensor,at::Tensor> _scaled_dot_product_flash_attention_cpu(const at::Tensor & query, const at::Tensor & key, const at::Tensor & value, double dropout_p=0.0, bool is_causal=false, const ::std::optional<at::Tensor> & attn_mask={}, ::std::optional<double> scale=::std::nullopt);
} // namespace native
} // namespace at