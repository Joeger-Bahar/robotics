#pragma once

#include <iostream>
#include <cmath>
#include <ratio>

namespace units
{
template<typename MassDim, typename LengthDim, typename TimeDim, typename AngleDim>
class Quantity
{
private:
    double value;

public:
    constexpr Quantity() : value(0.0) {}
    constexpr Quantity(double val) : value(val) {}
    constexpr Quantity(long double val) : value(static_cast<double>(val)) {}

    // The intrinsic operations for a quantity with a unit is addition and subtraction
    constexpr Quantity const& operator+=(const Quantity& rhs)
    {
        value += rhs.value;
        return *this;
    }
    constexpr Quantity const& operator-=(const Quantity& rhs)
    {
        value -= rhs.value;
        return *this;
    }

    // Returns the value of the quantity in multiples of the specified unit
    constexpr double convert(const Quantity& rhs) const
    {
        return value / rhs.value;
    }

    // returns the raw value of the quantity (should not be used)
    constexpr double getValue() const
    {
        return value;
    }
};


// Predefined (physical unit) quantity types:
// ------------------------------------------
#define QUANTITY_TYPE(_Mdim, _Ldim, _Tdim, _Adim, name) \
    typedef Quantity<std::ratio<_Mdim>, std::ratio<_Ldim>, std::ratio<_Tdim>, std::ratio<_Adim>> name;

// Replacement of "double" type
QUANTITY_TYPE(0, 0, 0, 0, Number);

// Physical quantity types
QUANTITY_TYPE(0, 1, 0, 0, Length);
QUANTITY_TYPE(0, 0, 1, 0, Time);
QUANTITY_TYPE(0, 1, -1, 0, Speed);
QUANTITY_TYPE(0, 1, -2, 0, Acceleration);

// Angle type:
QUANTITY_TYPE(0, 0, 0, 1, Angle);

// Standard arithmetic operators:
// ------------------------------
template <typename M, typename L, typename T, typename A>
constexpr Quantity<M, L, T, A> 
    operator+(const Quantity<M, L, T, A>& lhs, const Quantity<M, L, T, A>& rhs)
{
    return Quantity<M, L, T, A>(lhs.getValue() + rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr Quantity<M, L, T, A> 
    operator-(const Quantity<M, L, T, A>& lhs, const Quantity<M, L, T, A>& rhs)
{
    return Quantity<M, L, T, A>(lhs.getValue() - rhs.getValue());
}
template <typename M1, typename L1, typename T1, typename A1, 
          typename M2, typename L2, typename T2, typename A2>
constexpr Quantity<std::ratio_add<M1, M2>, std::ratio_add<L1, L2>, 
                    std::ratio_add<T1, T2>, std::ratio_add<A1, A2>> 
    operator*(const Quantity<M1, L1, T1, A1>& lhs, const Quantity<M2, L2, T2, A2>& rhs)
{
    return Quantity<std::ratio_add<M1, M2>, std::ratio_add<L1, L2>, 
                     std::ratio_add<T1, T2>, std::ratio_add<A1, A2>>
                    (lhs.getValue()*rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr Quantity<M, L, T, A> 
    operator*(const double& lhs, const Quantity<M, L, T, A>& rhs)
{
    return Quantity<M, L, T, A>(lhs*rhs.getValue());
}
template <typename M1, typename L1, typename T1, typename A1, 
          typename M2, typename L2, typename T2, typename A2>
constexpr Quantity<std::ratio_subtract<M1, M2>, std::ratio_subtract<L1, L2>,
                    std::ratio_subtract<T1, T2>, std::ratio_subtract<A1, A2>> 
    operator/(const Quantity<M1, L1, T1, A1>& lhs, const Quantity<M2, L2, T2, A2>& rhs)
{
    return Quantity<std::ratio_subtract<M1, M2>, std::ratio_subtract<L1, L2>, 
                     std::ratio_subtract<T1, T2>, std::ratio_subtract<A1, A2>>
                    (lhs.getValue() / rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr Quantity<std::ratio_subtract<std::ratio<0>, M>, std::ratio_subtract<std::ratio<0>, L>,
                    std::ratio_subtract<std::ratio<0>, T>, std::ratio_subtract<std::ratio<0>, A>> 
    operator/(double x, const Quantity<M, L, T, A>& rhs)
{
    return Quantity<std::ratio_subtract<std::ratio<0>, M>, std::ratio_subtract<std::ratio<0>, L>, 
                     std::ratio_subtract<std::ratio<0>, T>, std::ratio_subtract<std::ratio<0>, A>>
                    (x / rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr Quantity<M, L, T, A> 
    operator/(const Quantity<M, L, T, A>& rhs, double x)
{
    return Quantity<M, L, T, A>(rhs.getValue() / x);
}


// Comparison operators for quantities:
// ------------------------------------
template <typename M, typename L, typename T, typename A>
constexpr bool operator==(const Quantity<M, L, T, A>& lhs, const Quantity<M, L, T, A>& rhs)
{
    return (lhs.getValue() == rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator!=(const Quantity<M, L, T, A>& lhs, const Quantity<M, L, T, A>& rhs)
{
    return (lhs.getValue() != rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator<=(const Quantity<M, L, T, A>& lhs, const Quantity<M, L, T, A>& rhs)
{
    return (lhs.getValue() <= rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator>=(const Quantity<M, L, T, A>& lhs, const Quantity<M, L, T, A>& rhs)
{
    return (lhs.getValue() >= rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator< (const Quantity<M, L, T, A>& lhs, const Quantity<M, L, T, A>& rhs)
{
    return (lhs.getValue()<rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator> (const Quantity<M, L, T, A>& lhs, const Quantity<M, L, T, A>& rhs)
{
    return (lhs.getValue()>rhs.getValue());
}


// Predefined units:
// -----------------

// Predefined length-derived units
constexpr Length meter(1.0);                   // SI base unit
constexpr Length decimetre = meter / 10;
constexpr Length centimetre = meter / 100;
constexpr Length millimetre = meter / 1000;
constexpr Length kilometre = 1000 * meter;
constexpr Length inch = 2.54 * centimetre;
constexpr Length foot = 12 * inch;
constexpr Length yard = 3 * foot;
constexpr Length mile = 5280 * foot;

// Predefined time-derived units:
constexpr Time second(1.0);                        // SI base unit
constexpr Time minute = 60 * second;
constexpr Time hour = 60 * minute;
constexpr Time day = 24 * hour;

// Predefined mixed units:
constexpr Acceleration G = 9.80665 *  meter / (second*second);


// Physical unit literals:
// -----------------------

// literals for length units
constexpr Length operator"" _mm(long double x) { return static_cast<double>(x)*millimetre; }
constexpr Length operator"" _cm(long double x) { return static_cast<double>(x)*centimetre; }
constexpr Length operator"" _m(long double x) { return static_cast<double>(x)*meter; }
constexpr Length operator"" _km(long double x) { return static_cast<double>(x)*kilometre; }
constexpr Length operator"" _mi(long double x) { return static_cast<double>(x)*mile; }
constexpr Length operator"" _yd(long double x) { return static_cast<double>(x)*yard; }
constexpr Length operator"" _ft(long double x) { return static_cast<double>(x)*foot; }
constexpr Length operator"" _in(long double x) { return static_cast<double>(x)*inch; }
constexpr Length operator"" _mm(unsigned long long int x) { return static_cast<double>(x)*millimetre; }
constexpr Length operator"" _cm(unsigned long long int  x) { return static_cast<double>(x)*centimetre; }
constexpr Length operator"" _m(unsigned long long int  x) { return static_cast<double>(x)*meter; }
constexpr Length operator"" _km(unsigned long long int  x) { return static_cast<double>(x)*kilometre; }
constexpr Length operator"" _mi(unsigned long long int  x) { return static_cast<double>(x)*mile; }
constexpr Length operator"" _yd(unsigned long long int  x) { return static_cast<double>(x)*yard; }
constexpr Length operator"" _ft(unsigned long long int  x) { return static_cast<double>(x)*foot; }
constexpr Length operator"" _in(unsigned long long int  x) { return static_cast<double>(x)*inch; }

// literals for speed units
constexpr Speed operator"" _mps(long double x) { return Speed(x); };
constexpr Speed operator"" _miph(long double x) { return static_cast<double>(x)*mile / hour; };
constexpr Speed operator"" _kmph(long double x) { return static_cast<double>(x)*kilometre / hour; };
constexpr Speed operator"" _mps(unsigned long long int x) 
                                { return Speed(static_cast<long double>(x)); };
constexpr Speed operator"" _miph(unsigned long long int x) 
                                 { return static_cast<double>(x)*mile / hour; };
constexpr Speed operator"" _kmph(unsigned long long int x) 
                                 { return static_cast<double>(x)*kilometre / hour; };

// literals for time units
constexpr Time operator"" _s(long double x) { return Time(x); };
constexpr Time operator"" _min(long double x) { return static_cast<double>(x)*minute; };
constexpr Time operator"" _h(long double x) { return static_cast<double>(x)*hour; };
constexpr Time operator"" _day(long double x) { return static_cast<double>(x)*day; };
constexpr Time operator"" _s(unsigned long long int x) { return Time(static_cast<double>(x)); };
constexpr Time operator"" _min(unsigned long long int x) { return static_cast<double>(x)*minute; };
constexpr Time operator"" _h(unsigned long long int x) { return static_cast<double>(x)*hour; };
constexpr Time operator"" _day(unsigned long long int x) { return static_cast<double>(x)*day; };


// literals for acceleration units
constexpr Acceleration operator"" _mps2(long double x) { return Acceleration(x); };
constexpr Acceleration operator"" _mps2(unsigned long long int x) 
                                        { return Acceleration(static_cast<double>(x)); };
constexpr Acceleration operator"" _G(long double x) { return static_cast<double>(x)*G; };
constexpr Acceleration operator"" _G(unsigned long long int x) { return static_cast<double>(x)*G; }


// Angular unit literals:
// ----------------------
constexpr long double operator"" _pi(long double x) 
    { return static_cast<double>(x) * 3.1415926535897932384626433832795; }
constexpr long double operator"" _pi(unsigned long long int x) 
    { return static_cast<double>(x) * 3.1415926535897932384626433832795; }

// Predefined angle units:
constexpr Angle radian(1.0);
constexpr Angle degree = static_cast<double>(2_pi / 360.0) * radian;

// literals for angle units
constexpr Angle operator"" _rad(long double x) { return Angle(x); };
constexpr Angle operator"" _rad(unsigned long long int x) { return Angle(static_cast<double>(x)); };
constexpr Angle operator"" _deg(long double x) { return static_cast<double>(x)*degree; };
constexpr Angle operator"" _deg(unsigned long long int x) { return static_cast<double>(x)*degree; };

// Conversion macro, which utilizes the string literals
#define ConvertTo(_x, _y) (_x).Convert(1.0_##_y)



// Typesafe mathematical operations:
// ---------------------------------
template <typename M, typename L, typename T, typename A>
constexpr Quantity<std::ratio_divide<M, std::ratio<2>>, std::ratio_divide<L, std::ratio<2>>, 
                    std::ratio_divide<T, std::ratio<2>>, std::ratio_divide<A, std::ratio<2>>> 
    Qsqrt(const Quantity<M, L, T, A>& num)
{
    return Quantity<std::ratio_divide<M, std::ratio<2>>, std::ratio_divide<L, std::ratio<2>>, 
                     std::ratio_divide<T, std::ratio<2>>, std::ratio_divide<A, std::ratio<2>>>
                    (sqrt(num.getValue()));
}

// Typesafe trigonometric operations
inline double sin(const Angle &num)
{
    return sin(num.getValue());
}
inline double cos(const Angle &num)
{
    return cos(num.getValue());
}
inline double tan(const Angle &num)
{
    return tan(num.getValue());
}
} // namespace units