// ByteStream.h — byte codecs for trivially copyable values: the unit of exchange
// for anything serialized (e.g. save games). Values use the host's native layout,
// so both ends of an exchange must agree on it.
#pragma once

#include <cstddef>
#include <cstring>
#include <type_traits>
#include <utility>
#include <vector>

class ByteWriter {
public:
    template <typename T>
    void write(const T& value) {
        static_assert(std::is_trivially_copyable_v<T>,
                      "ByteStream only carries trivially copyable values");
        std::size_t offset{m_data.size()};
        m_data.resize(offset + sizeof(T));
        std::memcpy(m_data.data() + offset, &value, sizeof(T));
    }

    std::vector<std::byte> take() { return std::move(m_data); }

private:
    std::vector<std::byte> m_data{};
};

class ByteReader {
public:
    explicit ByteReader(const std::vector<std::byte>& data) : m_data{data} {}

    // False once the stream runs out: the buffer is truncated or malformed and the
    // caller must drop it. Every read is checked, so bad input can only cause a
    // drop, never an overrun.
    template <typename T>
    bool read(T& out) {
        static_assert(std::is_trivially_copyable_v<T>,
                      "ByteStream only carries trivially copyable values");
        if (m_offset + sizeof(T) > m_data.size()) {
            return false;
        }
        std::memcpy(&out, m_data.data() + m_offset, sizeof(T));
        m_offset += sizeof(T);
        return true;
    }

private:
    const std::vector<std::byte>& m_data;
    std::size_t m_offset{0};
};
