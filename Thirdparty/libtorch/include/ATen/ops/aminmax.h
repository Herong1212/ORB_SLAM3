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



#include <ATen/ops/aminmax_ops.h>

namespace at {


// aten::aminmax(Tensor self, *, int? dim=None, bool keepdim=False) -> (Tensor min, Tensor max)
inline ::std::tuple<at::Tensor,at::Tensor> aminmax(const at::Tensor & self, ::std::optional<int64_t> dim=::std::nullopt, bool keepdim=false) {
    return at::_ops::aminmax::call(self, dim, keepdim);
}

// aten::aminmax.out(Tensor self, *, int? dim=None, bool keepdim=False, Tensor(a!) min, Tensor(b!) max) -> (Tensor(a!) min, Tensor(b!) max)
inline ::std::tuple<at::Tensor &,at::Tensor &> aminmax_out(at::Tensor & min, at::Tensor & max, const at::Tensor & self, ::std::optional<int64_t> dim=::std::nullopt, bool keepdim=false) {
    return at::_ops::aminmax_out::call(self, dim, keepdim, min, max);
}
// aten::aminmax.out(Tensor self, *, int? dim=None, bool keepdim=False, Tensor(a!) min, Tensor(b!) max) -> (Tensor(a!) min, Tensor(b!) max)
inline ::std::tuple<at::Tensor &,at::Tensor &> aminmax_outf(const at::Tensor & self, ::std::optional<int64_t> dim, bool keepdim, at::Tensor & min, at::Tensor & max) {
    return at::_ops::aminmax_out::call(self, dim, keepdim, min, max);
}

}