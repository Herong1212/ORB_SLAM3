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



#include <ATen/ops/_linalg_slogdet_ops.h>

namespace at {


// aten::_linalg_slogdet(Tensor A) -> (Tensor sign, Tensor logabsdet, Tensor LU, Tensor pivots)
inline ::std::tuple<at::Tensor,at::Tensor,at::Tensor,at::Tensor> _linalg_slogdet(const at::Tensor & A) {
    return at::_ops::_linalg_slogdet::call(A);
}

// aten::_linalg_slogdet.sign(Tensor A, *, Tensor(a!) sign, Tensor(b!) logabsdet, Tensor(c!) LU, Tensor(d!) pivots) -> (Tensor(a!) sign, Tensor(b!) logabsdet, Tensor(c!) LU, Tensor(d!) pivots)
inline ::std::tuple<at::Tensor &,at::Tensor &,at::Tensor &,at::Tensor &> _linalg_slogdet_out(at::Tensor & sign, at::Tensor & logabsdet, at::Tensor & LU, at::Tensor & pivots, const at::Tensor & A) {
    return at::_ops::_linalg_slogdet_sign::call(A, sign, logabsdet, LU, pivots);
}
// aten::_linalg_slogdet.sign(Tensor A, *, Tensor(a!) sign, Tensor(b!) logabsdet, Tensor(c!) LU, Tensor(d!) pivots) -> (Tensor(a!) sign, Tensor(b!) logabsdet, Tensor(c!) LU, Tensor(d!) pivots)
inline ::std::tuple<at::Tensor &,at::Tensor &,at::Tensor &,at::Tensor &> _linalg_slogdet_outf(const at::Tensor & A, at::Tensor & sign, at::Tensor & logabsdet, at::Tensor & LU, at::Tensor & pivots) {
    return at::_ops::_linalg_slogdet_sign::call(A, sign, logabsdet, LU, pivots);
}

}
