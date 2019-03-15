
#ifndef UGV_UTILITIES_HPP
#define UGV_UTILITIES_HPP

// Get sign of number
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// Centimers conversion from SI units
constexpr long double operator"" _cm (long double centimeters) {
    return centimeters * 0.01;
}
constexpr long double operator"" _cm (unsigned long long centimeters) {
    return centimeters * 0.01;
}

// Meters conversion from SI units
constexpr long double operator"" _m (long double meters) {
    return meters;
}
constexpr long double operator"" _m (unsigned long long meters) {
    return meters;
}

// // Convert degrees to SI
// constexpr long double operator"" _deg (long double degrees) {
//     return degrees * 0.0174533;
// }
// constexpr long double operator"" _deg (unsigned long long degrees) {
//     return degrees * 0.0174533;
// }

// Notation for SI density
constexpr long double operator"" _kg_per_m3 (long double density) {
    return density;
}
constexpr long double operator"" _kg_per_m3 (unsigned long long density) {
    return density;
}


#endif
