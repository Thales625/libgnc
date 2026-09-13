namespace gnc::utils {
    template <typename T>
    constexpr T PI = static_cast<T>(3.14159265359);

    template <typename T>
    constexpr T sign(const T& val) {
        return static_cast<T>((val > 0) - (val < 0));
    }
}