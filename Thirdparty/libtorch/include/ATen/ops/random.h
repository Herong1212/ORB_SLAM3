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



#include <ATen/ops/random_ops.h>

namespace at {


// aten::random.from_out(Tensor self, int from, int? to, *, Generator? generator=None, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & random_out(at::Tensor & out, const at::Tensor & self, int64_t from, ::std::optional<int64_t> to, ::std::optional<at::Generator> generator=::std::nullopt) {
    return at::_ops::random_from_out::call(self, from, to, generator, out);
}
// aten::random.from_out(Tensor self, int from, int? to, *, Generator? generator=None, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & random_outf(const at::Tensor & self, int64_t from, ::std::optional<int64_t> to, ::std::optional<at::Generator> generator, at::Tensor & out) {
    return at::_ops::random_from_out::call(self, from, to, generator, out);
}

// aten::random.from(Tensor self, int from, int? to, *, Generator? generator=None) -> Tensor
inline at::Tensor random(const at::Tensor & self, int64_t from, ::std::optional<int64_t> to, ::std::optional<at::Generator> generator=::std::nullopt) {
    return at::_ops::random_from::call(self, from, to, generator);
}

// aten::random.to_out(Tensor self, int to, *, Generator? generator=None, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & random_out(at::Tensor & out, const at::Tensor & self, int64_t to, ::std::optional<at::Generator> generator=::std::nullopt) {
    return at::_ops::random_to_out::call(self, to, generator, out);
}
// aten::random.to_out(Tensor self, int to, *, Generator? generator=None, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & random_outf(const at::Tensor & self, int64_t to, ::std::optional<at::Generator> generator, at::Tensor & out) {
    return at::_ops::random_to_out::call(self, to, generator, out);
}

// aten::random.to(Tensor self, int to, *, Generator? generator=None) -> Tensor
inline at::Tensor random(const at::Tensor & self, int64_t to, ::std::optional<at::Generator> generator=::std::nullopt) {
    return at::_ops::random_to::call(self, to, generator);
}

// aten::random.out(Tensor self, *, Generator? generator=None, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & random_out(at::Tensor & out, const at::Tensor & self, ::std::optional<at::Generator> generator=::std::nullopt) {
    return at::_ops::random_out::call(self, generator, out);
}
// aten::random.out(Tensor self, *, Generator? generator=None, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & random_outf(const at::Tensor & self, ::std::optional<at::Generator> generator, at::Tensor & out) {
    return at::_ops::random_out::call(self, generator, out);
}

// aten::random(Tensor self, *, Generator? generator=None) -> Tensor
inline at::Tensor random(const at::Tensor & self, ::std::optional<at::Generator> generator=::std::nullopt) {
    return at::_ops::random::call(self, generator);
}

}
