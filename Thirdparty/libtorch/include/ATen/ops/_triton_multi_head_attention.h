#pragma once

// @generated by torchgen/gen.py from Function.h

#include <ATen/Context.h>
#include <ATen/DeviceGuard.h>
#include <ATen/TensorUtils.h>
#include <ATen/TracerMode.h>
#include <ATen/core/Generator.h>
#include <ATen/core/Reduction.h>
#include <ATen/core/Tensor.h>
#include <c10/core/Scalar.h>
#include <c10/core/Storage.h>
#include <c10/core/TensorOptions.h>
#include <c10/util/Deprecated.h>
#include <optional>



#include <ATen/ops/_triton_multi_head_attention_ops.h>

namespace at {


// aten::_triton_multi_head_attention(Tensor query, Tensor key, Tensor value, int embed_dim, int num_head, Tensor qkv_weight, Tensor qkv_bias, Tensor proj_weight, Tensor proj_bias, Tensor? mask=None) -> Tensor
inline at::Tensor _triton_multi_head_attention(const at::Tensor & query, const at::Tensor & key, const at::Tensor & value, int64_t embed_dim, int64_t num_head, const at::Tensor & qkv_weight, const at::Tensor & qkv_bias, const at::Tensor & proj_weight, const at::Tensor & proj_bias, const ::std::optional<at::Tensor> & mask={}) {
    return at::_ops::_triton_multi_head_attention::call(query, key, value, embed_dim, num_head, qkv_weight, qkv_bias, proj_weight, proj_bias, mask);
}

// aten::_triton_multi_head_attention.out(Tensor query, Tensor key, Tensor value, int embed_dim, int num_head, Tensor qkv_weight, Tensor qkv_bias, Tensor proj_weight, Tensor proj_bias, Tensor? mask=None, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & _triton_multi_head_attention_out(at::Tensor & out, const at::Tensor & query, const at::Tensor & key, const at::Tensor & value, int64_t embed_dim, int64_t num_head, const at::Tensor & qkv_weight, const at::Tensor & qkv_bias, const at::Tensor & proj_weight, const at::Tensor & proj_bias, const ::std::optional<at::Tensor> & mask={}) {
    return at::_ops::_triton_multi_head_attention_out::call(query, key, value, embed_dim, num_head, qkv_weight, qkv_bias, proj_weight, proj_bias, mask, out);
}
// aten::_triton_multi_head_attention.out(Tensor query, Tensor key, Tensor value, int embed_dim, int num_head, Tensor qkv_weight, Tensor qkv_bias, Tensor proj_weight, Tensor proj_bias, Tensor? mask=None, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & _triton_multi_head_attention_outf(const at::Tensor & query, const at::Tensor & key, const at::Tensor & value, int64_t embed_dim, int64_t num_head, const at::Tensor & qkv_weight, const at::Tensor & qkv_bias, const at::Tensor & proj_weight, const at::Tensor & proj_bias, const ::std::optional<at::Tensor> & mask, at::Tensor & out) {
    return at::_ops::_triton_multi_head_attention_out::call(query, key, value, embed_dim, num_head, qkv_weight, qkv_bias, proj_weight, proj_bias, mask, out);
}

}
