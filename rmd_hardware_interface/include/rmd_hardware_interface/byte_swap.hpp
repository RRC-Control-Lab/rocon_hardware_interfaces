#ifndef RMD_HARDWARE_INTERFACE__BYTE_SWAP_HPP_
#define RMD_HARDWARE_INTERFACE__BYTE_SWAP_HPP_

#include  <bit>

#if !__cpp_lib_byteswap
namespace std
{
// This will be added in C++23 but we backport it here
template<typename T>
constexpr T byteswap(T n) noexcept
{
  static_assert(
    std::has_unique_object_representations_v<T>,
    "T may not have padding bits");
  std::array<std::byte, sizeof(T)> & as_arr =
    reinterpret_cast<std::array<std::byte, sizeof(T)> &>(n);
  std::reverse(as_arr.begin(), as_arr.end());
  return n;
}
}  // namespace std
#endif  // !__cpp_lib_byteswap

template<std::endian endianness, typename T>
T maybe_byteswap(T val)
{
  if constexpr (std::endian::native == endianness) {
    return val;
  } else {
    return std::byteswap(val);
  }
}

template<typename T>
T read_le(const unsigned char * buf)
{
  return maybe_byteswap<std::endian::little>(*reinterpret_cast<const T *>(buf));
}

template<typename T>
void write_le(const T & val, const unsigned char * buf)
{
  *reinterpret_cast<T *>(const_cast<unsigned char *>(buf)) =
    maybe_byteswap<std::endian::little>(val);
}

#endif  // RMD_HARDWARE_INTERFACE__BYTE_SWAP_HPP_
