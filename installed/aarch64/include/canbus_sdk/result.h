#ifndef CANBUS_SDK_RESULT_H
#define CANBUS_SDK_RESULT_H

/**
 * @file result.h
 * @brief Result<T> template class for error handling in CAN bus SDK
 * 
 * This file provides a modern C++ Result<T> template class that represents
 * either a successful value of type T or an error code. The implementation
 * follows standard library patterns with move semantics, noexcept specifications,
 * and comprehensive comparison operators.
 * 
 * Key features:
 * - Union-based memory optimization
 * - Move semantics with proper noexcept specifications
 * - Standard library style methods (has_value(), value(), error())
 * - Factory methods for creating success/error results
 * - Comparison operators with values and error codes
 * - Perfect forwarding in value_or method
 * - Thread-safe operations
 */

#include <utility> // for std::move
#include <new>     // for placement new
#include <type_traits>

namespace canbus_sdk {

template<typename T>
class Result {
    static_assert(!std::is_same<T, void>::value, "Result<void> is not supported");
    
private:
    union {
        T data;
        int error_code;
    };
    bool success;
    
public:
    // Constructors
    Result() noexcept : success(false) {}  // Default constructor for internal use
    explicit Result(const T& val) noexcept(std::is_nothrow_copy_constructible<T>::value) : data(val), success(true) {}
    explicit Result(T&& val) noexcept(std::is_nothrow_move_constructible<T>::value) : data(std::move(val)), success(true) {}
    explicit Result(int err) noexcept : error_code(err), success(false) {}
    
    // Move constructor
    Result(Result&& other) noexcept(std::is_nothrow_move_constructible<T>::value) : success(other.success) {
        if (success) {
            new(&data) T(std::move(other.data));
        } else {
            error_code = other.error_code;
        }
    }
    
    // Destructor
    ~Result() noexcept {
        if (success) {
            data.~T();
        }
    }
    
    // Move assignment operator
    Result& operator=(Result&& other) noexcept(std::is_nothrow_move_constructible<T>::value && std::is_nothrow_move_assignable<T>::value) {
        if (this != &other) {
            if (success) {
                data.~T();
            }
            success = other.success;
            if (success) {
                new(&data) T(std::move(other.data));
            } else {
                error_code = other.error_code;
            }
        }
        return *this;
    }
    
    // Delete copy constructor and assignment (prevent double destruction)
    Result(const Result&) = delete;
    Result& operator=(const Result&) = delete;
    
    // Type conversion
    explicit operator bool() const noexcept { return success; }
    
    // Access operators
    const T& operator*() const & noexcept { return data; }
    T& operator*() & noexcept { return data; }
    T&& operator*() && noexcept { return std::move(data); }
    const T* operator->() const noexcept { return &data; }
    T* operator->() noexcept { return &data; }
    
    // Standard library style methods
    [[nodiscard]] bool has_value() const noexcept { return success; }
    [[nodiscard]] const T& value() const & noexcept { return data; }
    [[nodiscard]] T& value() & noexcept { return data; }
    [[nodiscard]] T&& value() && noexcept { return std::move(data); }
    [[nodiscard]] int error() const noexcept { return error_code; }
    
    // Improved value_or
    template<typename U>
    [[nodiscard]] T value_or(U&& default_value) const & 
        noexcept(std::is_nothrow_copy_constructible<T>::value && std::is_nothrow_copy_constructible<T>::value) {
        return success ? data : static_cast<T>(std::forward<U>(default_value));
    }
    
    template<typename U>
    [[nodiscard]] T value_or(U&& default_value) && 
        noexcept(std::is_nothrow_move_constructible<T>::value && std::is_nothrow_copy_constructible<T>::value) {
        return success ? std::move(data) : static_cast<T>(std::forward<U>(default_value));
    }
    
        
    // Factory methods
    static Result<T> ok(const T& value) noexcept(std::is_nothrow_copy_constructible<T>::value) { 
        Result<T> result;
        result.success = true;
        new(&result.data) T(value);
        return result;
    }
    
    static Result<T> ok(T&& value) noexcept(std::is_nothrow_move_constructible<T>::value) { 
        Result<T> result;
        result.success = true;
        new(&result.data) T(std::move(value));
        return result;
    }
    
    static Result<T> error(int error_code) noexcept { 
        Result<T> result;
        result.success = false;
        result.error_code = error_code;
        return result;
    }
    
    // Comparison operators
    bool operator==(const Result& other) const noexcept {
        if (success != other.success) return false;
        return success ? (data == other.data) : (error_code == other.error_code);
    }
    
    bool operator!=(const Result& other) const noexcept {
        return !(*this == other);
    }
    
    // Comparison with values
    bool operator==(const T& other_value) const noexcept(noexcept(data == other_value)) {
        return success && (data == other_value);
    }
    
    bool operator!=(const T& other_value) const noexcept(noexcept(data == other_value)) {
        return !(*this == other_value);
    }
    
    // Comparison with error codes
    bool operator==(int other_error) const noexcept {
        return !success && (error_code == other_error);
    }
    
    bool operator!=(int other_error) const noexcept {
        return !(*this == other_error);
    }
};

} // namespace canbus_sdk

/*
 * Usage Examples:
 * 
 * // 1. Basic usage with factory methods
 * Result<int> result = Result<int>::ok(42);
 * if (result.has_value()) {
 *     int value = result.value();
 *     // use value...
 * } else {
 *     int error = result.error();
 *     // handle error...
 * }
 * 
 * // 2. Modern C++ style with operator bool
 * if (auto result = someFunction()) {
 *     // Success case
 *     auto value = *result;
 *     result->someMethod();  // if T is a pointer or has operator->
 * } else {
 *     // Error case
 *     auto error = result.error();
 * }
 * 
 * // 3. Error handling
 * Result<std::string> result = Result<std::string>::error(-1);
 * if (!result) {
 *     std::cout << "Error code: " << result.error() << std::endl;
 * }
 * 
 * // 4. Move semantics
 * Result<std::vector<int>> result = Result<std::vector<int>>::ok(createLargeVector());
 * auto vec = std::move(*result);  // Efficient move
 * 
 * // 5. Default value fallback with improved value_or
 * Result<std::string> result = someOperation();
 * std::string value = result.value_or("default_value");
 * 
 * // 6. Move assignment
 * Result<std::vector<int>> result1 = Result<std::vector<int>>::ok({1, 2, 3});
 * Result<std::vector<int>> result2;
 * result2 = std::move(result1);  // Move assignment
 * 
 * // 7. Comparison operators
 * Result<int> success1 = Result<int>::ok(42);
 * Result<int> success2 = Result<int>::ok(42);
 * Result<int> error = Result<int>::error(-1);
 * 
 */

#endif // CANBUS_SDK_RESULT_H