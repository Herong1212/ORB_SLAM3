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



#include <ATen/ops/repeat_ops.h>

namespace at {


namespace symint {
  template <typename T, typename = std::enable_if_t<std::is_same<T, int64_t>::value>>
  at::Tensor repeat(const at::Tensor & self, at::IntArrayRef repeats) {
    return at::_ops::repeat::call(self, c10::fromIntArrayRefSlow(repeats));
  }
}

namespace symint {
  template <typename T, typename = std::enable_if_t<std::is_same<T, c10::SymInt>::value>>
  at::Tensor repeat(const at::Tensor & self, c10::SymIntArrayRef repeats) {
    return at::_ops::repeat::call(self, repeats);
  }
}

// aten::repeat.out(Tensor self, SymInt[] repeats, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & repeat_out(at::Tensor & out, const at::Tensor & self, at::IntArrayRef repeats) {
    return at::_ops::repeat_out::call(self, c10::fromIntArrayRefSlow(repeats), out);
}
namespace symint {
  template <typename T, typename = std::enable_if_t<std::is_same<T, int64_t>::value>>
  at::Tensor & repeat_out(at::Tensor & out, const at::Tensor & self, at::IntArrayRef repeats) {
    return at::_ops::repeat_out::call(self, c10::fromIntArrayRefSlow(repeats), out);
  }
}

// aten::repeat.out(Tensor self, SymInt[] repeats, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & repeat_outf(const at::Tensor & self, at::IntArrayRef repeats, at::Tensor & out) {
    return at::_ops::repeat_out::call(self, c10::fromIntArrayRefSlow(repeats), out);
}
namespace symint {
  template <typename T, typename = std::enable_if_t<std::is_same<T, int64_t>::value>>
  at::Tensor & repeat_outf(const at::Tensor & self, at::IntArrayRef repeats, at::Tensor & out) {
    return at::_ops::repeat_out::call(self, c10::fromIntArrayRefSlow(repeats), out);
  }
}

// aten::repeat.out(Tensor self, SymInt[] repeats, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & repeat_symint_out(at::Tensor & out, const at::Tensor & self, c10::SymIntArrayRef repeats) {
    return at::_ops::repeat_out::call(self, repeats, out);
}
namespace symint {
  template <typename T, typename = std::enable_if_t<std::is_same<T, c10::SymInt>::value>>
  at::Tensor & repeat_out(at::Tensor & out, const at::Tensor & self, c10::SymIntArrayRef repeats) {
    return at::_ops::repeat_out::call(self, repeats, out);
  }
}

// aten::repeat.out(Tensor self, SymInt[] repeats, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & repeat_symint_outf(const at::Tensor & self, c10::SymIntArrayRef repeats, at::Tensor & out) {
    return at::_ops::repeat_out::call(self, repeats, out);
}
namespace symint {
  template <typename T, typename = std::enable_if_t<std::is_same<T, c10::SymInt>::value>>
  at::Tensor & repeat_outf(const at::Tensor & self, c10::SymIntArrayRef repeats, at::Tensor & out) {
    return at::_ops::repeat_out::call(self, repeats, out);
  }
}

}
