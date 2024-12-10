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



#include <ATen/ops/div_ops.h>

namespace at {


// aten::div.Tensor(Tensor self, Tensor other) -> Tensor
inline at::Tensor div(const at::Tensor & self, const at::Tensor & other) {
    return at::_ops::div_Tensor::call(self, other);
}

// aten::div.out(Tensor self, Tensor other, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & div_out(at::Tensor & out, const at::Tensor & self, const at::Tensor & other) {
    return at::_ops::div_out::call(self, other, out);
}
// aten::div.out(Tensor self, Tensor other, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & div_outf(const at::Tensor & self, const at::Tensor & other, at::Tensor & out) {
    return at::_ops::div_out::call(self, other, out);
}

// aten::div.Tensor_mode(Tensor self, Tensor other, *, str? rounding_mode) -> Tensor
inline at::Tensor div(const at::Tensor & self, const at::Tensor & other, ::std::optional<c10::string_view> rounding_mode) {
    return at::_ops::div_Tensor_mode::call(self, other, rounding_mode);
}

// aten::div.out_mode(Tensor self, Tensor other, *, str? rounding_mode, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & div_out(at::Tensor & out, const at::Tensor & self, const at::Tensor & other, ::std::optional<c10::string_view> rounding_mode) {
    return at::_ops::div_out_mode::call(self, other, rounding_mode, out);
}
// aten::div.out_mode(Tensor self, Tensor other, *, str? rounding_mode, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & div_outf(const at::Tensor & self, const at::Tensor & other, ::std::optional<c10::string_view> rounding_mode, at::Tensor & out) {
    return at::_ops::div_out_mode::call(self, other, rounding_mode, out);
}

// aten::div.Scalar(Tensor self, Scalar other) -> Tensor
inline at::Tensor div(const at::Tensor & self, const at::Scalar & other) {
    return at::_ops::div_Scalar::call(self, other);
}

// aten::div.Scalar_mode(Tensor self, Scalar other, *, str? rounding_mode) -> Tensor
inline at::Tensor div(const at::Tensor & self, const at::Scalar & other, ::std::optional<c10::string_view> rounding_mode) {
    return at::_ops::div_Scalar_mode::call(self, other, rounding_mode);
}

// aten::div.Scalar_out(Tensor self, Scalar other, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & div_out(at::Tensor & out, const at::Tensor & self, const at::Scalar & other) {
    return at::_ops::div_Scalar_out::call(self, other, out);
}
// aten::div.Scalar_out(Tensor self, Scalar other, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & div_outf(const at::Tensor & self, const at::Scalar & other, at::Tensor & out) {
    return at::_ops::div_Scalar_out::call(self, other, out);
}

// aten::div.Scalar_mode_out(Tensor self, Scalar other, *, str? rounding_mode, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & div_out(at::Tensor & out, const at::Tensor & self, const at::Scalar & other, ::std::optional<c10::string_view> rounding_mode) {
    return at::_ops::div_Scalar_mode_out::call(self, other, rounding_mode, out);
}
// aten::div.Scalar_mode_out(Tensor self, Scalar other, *, str? rounding_mode, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & div_outf(const at::Tensor & self, const at::Scalar & other, ::std::optional<c10::string_view> rounding_mode, at::Tensor & out) {
    return at::_ops::div_Scalar_mode_out::call(self, other, rounding_mode, out);
}

}
