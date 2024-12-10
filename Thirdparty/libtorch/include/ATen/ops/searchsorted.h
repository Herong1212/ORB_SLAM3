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



#include <ATen/ops/searchsorted_ops.h>

namespace at {


// aten::searchsorted.Tensor(Tensor sorted_sequence, Tensor self, *, bool out_int32=False, bool right=False, str? side=None, Tensor? sorter=None) -> Tensor
inline at::Tensor searchsorted(const at::Tensor & sorted_sequence, const at::Tensor & self, bool out_int32=false, bool right=false, ::std::optional<c10::string_view> side=::std::nullopt, const ::std::optional<at::Tensor> & sorter={}) {
    return at::_ops::searchsorted_Tensor::call(sorted_sequence, self, out_int32, right, side, sorter);
}

// aten::searchsorted.Tensor_out(Tensor sorted_sequence, Tensor self, *, bool out_int32=False, bool right=False, str? side=None, Tensor? sorter=None, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & searchsorted_out(at::Tensor & out, const at::Tensor & sorted_sequence, const at::Tensor & self, bool out_int32=false, bool right=false, ::std::optional<c10::string_view> side=::std::nullopt, const ::std::optional<at::Tensor> & sorter={}) {
    return at::_ops::searchsorted_Tensor_out::call(sorted_sequence, self, out_int32, right, side, sorter, out);
}
// aten::searchsorted.Tensor_out(Tensor sorted_sequence, Tensor self, *, bool out_int32=False, bool right=False, str? side=None, Tensor? sorter=None, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & searchsorted_outf(const at::Tensor & sorted_sequence, const at::Tensor & self, bool out_int32, bool right, ::std::optional<c10::string_view> side, const ::std::optional<at::Tensor> & sorter, at::Tensor & out) {
    return at::_ops::searchsorted_Tensor_out::call(sorted_sequence, self, out_int32, right, side, sorter, out);
}

// aten::searchsorted.Scalar(Tensor sorted_sequence, Scalar self, *, bool out_int32=False, bool right=False, str? side=None, Tensor? sorter=None) -> Tensor
inline at::Tensor searchsorted(const at::Tensor & sorted_sequence, const at::Scalar & self, bool out_int32=false, bool right=false, ::std::optional<c10::string_view> side=::std::nullopt, const ::std::optional<at::Tensor> & sorter={}) {
    return at::_ops::searchsorted_Scalar::call(sorted_sequence, self, out_int32, right, side, sorter);
}

// aten::searchsorted.Scalar_out(Tensor sorted_sequence, Scalar self, *, bool out_int32=False, bool right=False, str? side=None, Tensor? sorter=None, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & searchsorted_out(at::Tensor & out, const at::Tensor & sorted_sequence, const at::Scalar & self, bool out_int32=false, bool right=false, ::std::optional<c10::string_view> side=::std::nullopt, const ::std::optional<at::Tensor> & sorter={}) {
    return at::_ops::searchsorted_Scalar_out::call(sorted_sequence, self, out_int32, right, side, sorter, out);
}
// aten::searchsorted.Scalar_out(Tensor sorted_sequence, Scalar self, *, bool out_int32=False, bool right=False, str? side=None, Tensor? sorter=None, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & searchsorted_outf(const at::Tensor & sorted_sequence, const at::Scalar & self, bool out_int32, bool right, ::std::optional<c10::string_view> side, const ::std::optional<at::Tensor> & sorter, at::Tensor & out) {
    return at::_ops::searchsorted_Scalar_out::call(sorted_sequence, self, out_int32, right, side, sorter, out);
}

}
