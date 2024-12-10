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



#include <ATen/ops/channel_shuffle_ops.h>

namespace at {


// aten::channel_shuffle(Tensor self, SymInt groups) -> Tensor
inline at::Tensor channel_shuffle(const at::Tensor & self, int64_t groups) {
    return at::_ops::channel_shuffle::call(self, groups);
}
namespace symint {
  template <typename T, typename = std::enable_if_t<std::is_same<T, int64_t>::value>>
  at::Tensor channel_shuffle(const at::Tensor & self, int64_t groups) {
    return at::_ops::channel_shuffle::call(self, groups);
  }
}

// aten::channel_shuffle(Tensor self, SymInt groups) -> Tensor
inline at::Tensor channel_shuffle_symint(const at::Tensor & self, c10::SymInt groups) {
    return at::_ops::channel_shuffle::call(self, groups);
}
namespace symint {
  template <typename T, typename = std::enable_if_t<std::is_same<T, c10::SymInt>::value>>
  at::Tensor channel_shuffle(const at::Tensor & self, c10::SymInt groups) {
    return at::_ops::channel_shuffle::call(self, groups);
  }
}

// aten::channel_shuffle.out(Tensor self, SymInt groups, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & channel_shuffle_out(at::Tensor & out, const at::Tensor & self, int64_t groups) {
    return at::_ops::channel_shuffle_out::call(self, groups, out);
}
namespace symint {
  template <typename T, typename = std::enable_if_t<std::is_same<T, int64_t>::value>>
  at::Tensor & channel_shuffle_out(at::Tensor & out, const at::Tensor & self, int64_t groups) {
    return at::_ops::channel_shuffle_out::call(self, groups, out);
  }
}

// aten::channel_shuffle.out(Tensor self, SymInt groups, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & channel_shuffle_outf(const at::Tensor & self, int64_t groups, at::Tensor & out) {
    return at::_ops::channel_shuffle_out::call(self, groups, out);
}
namespace symint {
  template <typename T, typename = std::enable_if_t<std::is_same<T, int64_t>::value>>
  at::Tensor & channel_shuffle_outf(const at::Tensor & self, int64_t groups, at::Tensor & out) {
    return at::_ops::channel_shuffle_out::call(self, groups, out);
  }
}

// aten::channel_shuffle.out(Tensor self, SymInt groups, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & channel_shuffle_symint_out(at::Tensor & out, const at::Tensor & self, c10::SymInt groups) {
    return at::_ops::channel_shuffle_out::call(self, groups, out);
}
namespace symint {
  template <typename T, typename = std::enable_if_t<std::is_same<T, c10::SymInt>::value>>
  at::Tensor & channel_shuffle_out(at::Tensor & out, const at::Tensor & self, c10::SymInt groups) {
    return at::_ops::channel_shuffle_out::call(self, groups, out);
  }
}

// aten::channel_shuffle.out(Tensor self, SymInt groups, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & channel_shuffle_symint_outf(const at::Tensor & self, c10::SymInt groups, at::Tensor & out) {
    return at::_ops::channel_shuffle_out::call(self, groups, out);
}
namespace symint {
  template <typename T, typename = std::enable_if_t<std::is_same<T, c10::SymInt>::value>>
  at::Tensor & channel_shuffle_outf(const at::Tensor & self, c10::SymInt groups, at::Tensor & out) {
    return at::_ops::channel_shuffle_out::call(self, groups, out);
  }
}

}