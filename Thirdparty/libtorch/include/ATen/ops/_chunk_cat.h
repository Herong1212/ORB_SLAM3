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



#include <ATen/ops/_chunk_cat_ops.h>

namespace at {


// aten::_chunk_cat(Tensor[] tensors, int dim, int num_chunks) -> Tensor
inline at::Tensor _chunk_cat(at::TensorList tensors, int64_t dim, int64_t num_chunks) {
    return at::_ops::_chunk_cat::call(tensors, dim, num_chunks);
}

// aten::_chunk_cat.out(Tensor[] tensors, int dim, int num_chunks, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & _chunk_cat_out(at::Tensor & out, at::TensorList tensors, int64_t dim, int64_t num_chunks) {
    return at::_ops::_chunk_cat_out::call(tensors, dim, num_chunks, out);
}
// aten::_chunk_cat.out(Tensor[] tensors, int dim, int num_chunks, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & _chunk_cat_outf(at::TensorList tensors, int64_t dim, int64_t num_chunks, at::Tensor & out) {
    return at::_ops::_chunk_cat_out::call(tensors, dim, num_chunks, out);
}

}