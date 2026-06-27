#pragma once

#include <any>
#include <typeindex>
#include <unordered_map>

class IPointerStorage {
    std::unordered_map<std::type_index, std::any> m_pointers;

public:
    template<typename T>
    void set_pointer(T* ptr) {
        m_pointers[std::type_index(typeid(T))] = ptr;
    }

    template<typename T>
    T* get_pointer() {
        auto it = m_pointers.find(std::type_index(typeid(T)));
        if (it == m_pointers.end()) return nullptr;
        return std::any_cast<T*>(it->second);
    }

    template<typename T>
    bool has_pointer() const {
        auto it = m_pointers.find(std::type_index(typeid(T)));
        return it != m_pointers.end() && it->second.has_value();
    }

    template<typename T>
    void remove_pointer() {
        m_pointers.erase(std::type_index(typeid(T)));
    }

    virtual ~IPointerStorage() = default;
};
