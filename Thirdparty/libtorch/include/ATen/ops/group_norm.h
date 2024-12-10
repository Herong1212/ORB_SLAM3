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



#include <ATen/ops/group_norm_ops.h>

namespace at {


// aten::group_norm(Tensor input, int num_groups, Tensor? weight=None, Tensor? bias=None, float eps=1e-05, bool cudnn_enabled=True) -> Tensor
inline at::Tensor group_norm(const at::Tensor & input, int64_t num_groups, const ::std::optional<at::Tensor> & weight={}, const ::std::optional<at::Tensor> & bias={}, double eps=1e-05, bool cudnn_enabled=true) {
    return at::_ops::group_norm::call(input, num_groups, weight, bias, eps, cudnn_enabled);
}

}