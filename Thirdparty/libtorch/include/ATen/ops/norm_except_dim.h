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



#include <ATen/ops/norm_except_dim_ops.h>

namespace at {


// aten::norm_except_dim(Tensor v, int pow=2, int dim=0) -> Tensor
inline at::Tensor norm_except_dim(const at::Tensor & v, int64_t pow=2, int64_t dim=0) {
    return at::_ops::norm_except_dim::call(v, pow, dim);
}

}
