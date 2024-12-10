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



#include <ATen/ops/linalg_pinv_ops.h>

namespace at {


// aten::linalg_pinv.atol_rtol_tensor(Tensor self, *, Tensor? atol=None, Tensor? rtol=None, bool hermitian=False) -> Tensor
inline at::Tensor linalg_pinv(const at::Tensor & self, const ::std::optional<at::Tensor> & atol={}, const ::std::optional<at::Tensor> & rtol={}, bool hermitian=false) {
    return at::_ops::linalg_pinv_atol_rtol_tensor::call(self, atol, rtol, hermitian);
}

// aten::linalg_pinv.atol_rtol_tensor_out(Tensor self, *, Tensor? atol=None, Tensor? rtol=None, bool hermitian=False, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & linalg_pinv_out(at::Tensor & out, const at::Tensor & self, const ::std::optional<at::Tensor> & atol={}, const ::std::optional<at::Tensor> & rtol={}, bool hermitian=false) {
    return at::_ops::linalg_pinv_atol_rtol_tensor_out::call(self, atol, rtol, hermitian, out);
}
// aten::linalg_pinv.atol_rtol_tensor_out(Tensor self, *, Tensor? atol=None, Tensor? rtol=None, bool hermitian=False, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & linalg_pinv_outf(const at::Tensor & self, const ::std::optional<at::Tensor> & atol, const ::std::optional<at::Tensor> & rtol, bool hermitian, at::Tensor & out) {
    return at::_ops::linalg_pinv_atol_rtol_tensor_out::call(self, atol, rtol, hermitian, out);
}

// aten::linalg_pinv.atol_rtol_float(Tensor self, *, float? atol=None, float? rtol=None, bool hermitian=False) -> Tensor
inline at::Tensor linalg_pinv(const at::Tensor & self, ::std::optional<double> atol, ::std::optional<double> rtol, bool hermitian=false) {
    return at::_ops::linalg_pinv_atol_rtol_float::call(self, atol, rtol, hermitian);
}

// aten::linalg_pinv.atol_rtol_float_out(Tensor self, *, float? atol=None, float? rtol=None, bool hermitian=False, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & linalg_pinv_out(at::Tensor & out, const at::Tensor & self, ::std::optional<double> atol, ::std::optional<double> rtol, bool hermitian=false) {
    return at::_ops::linalg_pinv_atol_rtol_float_out::call(self, atol, rtol, hermitian, out);
}
// aten::linalg_pinv.atol_rtol_float_out(Tensor self, *, float? atol=None, float? rtol=None, bool hermitian=False, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & linalg_pinv_outf(const at::Tensor & self, ::std::optional<double> atol, ::std::optional<double> rtol, bool hermitian, at::Tensor & out) {
    return at::_ops::linalg_pinv_atol_rtol_float_out::call(self, atol, rtol, hermitian, out);
}

// aten::linalg_pinv(Tensor self, float rcond, bool hermitian=False) -> Tensor
inline at::Tensor linalg_pinv(const at::Tensor & self, double rcond, bool hermitian=false) {
    return at::_ops::linalg_pinv::call(self, rcond, hermitian);
}

// aten::linalg_pinv.rcond_tensor(Tensor self, Tensor rcond, bool hermitian=False) -> Tensor
inline at::Tensor linalg_pinv(const at::Tensor & self, const at::Tensor & rcond, bool hermitian=false) {
    return at::_ops::linalg_pinv_rcond_tensor::call(self, rcond, hermitian);
}

// aten::linalg_pinv.out(Tensor self, float rcond, bool hermitian=False, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & linalg_pinv_out(at::Tensor & out, const at::Tensor & self, double rcond, bool hermitian=false) {
    return at::_ops::linalg_pinv_out::call(self, rcond, hermitian, out);
}
// aten::linalg_pinv.out(Tensor self, float rcond, bool hermitian=False, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & linalg_pinv_outf(const at::Tensor & self, double rcond, bool hermitian, at::Tensor & out) {
    return at::_ops::linalg_pinv_out::call(self, rcond, hermitian, out);
}

// aten::linalg_pinv.out_rcond_tensor(Tensor self, Tensor rcond, bool hermitian=False, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & linalg_pinv_out(at::Tensor & out, const at::Tensor & self, const at::Tensor & rcond, bool hermitian=false) {
    return at::_ops::linalg_pinv_out_rcond_tensor::call(self, rcond, hermitian, out);
}
// aten::linalg_pinv.out_rcond_tensor(Tensor self, Tensor rcond, bool hermitian=False, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & linalg_pinv_outf(const at::Tensor & self, const at::Tensor & rcond, bool hermitian, at::Tensor & out) {
    return at::_ops::linalg_pinv_out_rcond_tensor::call(self, rcond, hermitian, out);
}

}