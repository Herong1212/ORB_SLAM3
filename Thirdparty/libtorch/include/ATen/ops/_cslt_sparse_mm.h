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



#include <ATen/ops/_cslt_sparse_mm_ops.h>

namespace at {


// aten::_cslt_sparse_mm(Tensor compressed_A, Tensor dense_B, Tensor? bias=None, Tensor? alpha=None, ScalarType? out_dtype=None, bool transpose_result=False, int alg_id=0) -> Tensor
inline at::Tensor _cslt_sparse_mm(const at::Tensor & compressed_A, const at::Tensor & dense_B, const ::std::optional<at::Tensor> & bias={}, const ::std::optional<at::Tensor> & alpha={}, ::std::optional<at::ScalarType> out_dtype=::std::nullopt, bool transpose_result=false, int64_t alg_id=0) {
    return at::_ops::_cslt_sparse_mm::call(compressed_A, dense_B, bias, alpha, out_dtype, transpose_result, alg_id);
}

}