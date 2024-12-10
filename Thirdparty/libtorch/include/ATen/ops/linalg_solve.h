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



#include <ATen/ops/linalg_solve_ops.h>

namespace at {


// aten::linalg_solve(Tensor A, Tensor B, *, bool left=True) -> Tensor
inline at::Tensor linalg_solve(const at::Tensor & A, const at::Tensor & B, bool left=true) {
    return at::_ops::linalg_solve::call(A, B, left);
}

// aten::linalg_solve.out(Tensor A, Tensor B, *, bool left=True, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & linalg_solve_out(at::Tensor & out, const at::Tensor & A, const at::Tensor & B, bool left=true) {
    return at::_ops::linalg_solve_out::call(A, B, left, out);
}
// aten::linalg_solve.out(Tensor A, Tensor B, *, bool left=True, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & linalg_solve_outf(const at::Tensor & A, const at::Tensor & B, bool left, at::Tensor & out) {
    return at::_ops::linalg_solve_out::call(A, B, left, out);
}

}
