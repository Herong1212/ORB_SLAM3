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



#include <ATen/ops/batch_norm_gather_stats_with_counts_ops.h>

namespace at {


// aten::batch_norm_gather_stats_with_counts(Tensor input, Tensor mean, Tensor invstd, Tensor? running_mean, Tensor? running_var, float momentum, float eps, Tensor counts) -> (Tensor, Tensor)
inline ::std::tuple<at::Tensor,at::Tensor> batch_norm_gather_stats_with_counts(const at::Tensor & input, const at::Tensor & mean, const at::Tensor & invstd, const ::std::optional<at::Tensor> & running_mean, const ::std::optional<at::Tensor> & running_var, double momentum, double eps, const at::Tensor & counts) {
    return at::_ops::batch_norm_gather_stats_with_counts::call(input, mean, invstd, running_mean, running_var, momentum, eps, counts);
}

// aten::batch_norm_gather_stats_with_counts.out(Tensor input, Tensor mean, Tensor invstd, Tensor? running_mean, Tensor? running_var, float momentum, float eps, Tensor counts, *, Tensor(a!) out0, Tensor(b!) out1) -> (Tensor(a!), Tensor(b!))
inline ::std::tuple<at::Tensor &,at::Tensor &> batch_norm_gather_stats_with_counts_out(at::Tensor & out0, at::Tensor & out1, const at::Tensor & input, const at::Tensor & mean, const at::Tensor & invstd, const ::std::optional<at::Tensor> & running_mean, const ::std::optional<at::Tensor> & running_var, double momentum, double eps, const at::Tensor & counts) {
    return at::_ops::batch_norm_gather_stats_with_counts_out::call(input, mean, invstd, running_mean, running_var, momentum, eps, counts, out0, out1);
}
// aten::batch_norm_gather_stats_with_counts.out(Tensor input, Tensor mean, Tensor invstd, Tensor? running_mean, Tensor? running_var, float momentum, float eps, Tensor counts, *, Tensor(a!) out0, Tensor(b!) out1) -> (Tensor(a!), Tensor(b!))
inline ::std::tuple<at::Tensor &,at::Tensor &> batch_norm_gather_stats_with_counts_outf(const at::Tensor & input, const at::Tensor & mean, const at::Tensor & invstd, const ::std::optional<at::Tensor> & running_mean, const ::std::optional<at::Tensor> & running_var, double momentum, double eps, const at::Tensor & counts, at::Tensor & out0, at::Tensor & out1) {
    return at::_ops::batch_norm_gather_stats_with_counts_out::call(input, mean, invstd, running_mean, running_var, momentum, eps, counts, out0, out1);
}

}
