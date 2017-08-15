#ifndef MATH_VECTOR_H
#define MATH_VECTOR_H

#include <array>
#include <algorithm>

typedef double scalar_t;
typedef std::array<scalar_t, 1> vector1_t;
typedef std::array<scalar_t, 2> vector2_t;
typedef std::array<scalar_t, 3> vector3_t;

template <typename InternalScalar, typename InternalVector>
class MathVector {
public:
  typedef InternalScalar Scalar;

  MathVector() {};
  MathVector(InternalVector v_) : v(v_) {};

  friend MathVector operator*(InternalScalar s, MathVector v) {
    InternalVector out;
    std::transform(v.v.begin(), v.v.end(), out.begin(),
                   [s](InternalScalar v_i) { return s*v_i; });
    return MathVector(out);
  }
  friend MathVector operator*(MathVector v, InternalScalar s) {
    InternalVector out;
    std::transform(v.v.begin(), v.v.end(), out.begin(),
                   [s](InternalScalar v_i) { return s*v_i; });
    return MathVector(out);
  }

  friend MathVector operator+(MathVector v1, MathVector v2) {
    InternalVector out;
    std::transform(v1.v.begin(), v1.v.end(), v2.v.begin(), out.begin(),
                   [](InternalScalar v1_i, InternalScalar v2_i) { return v1_i+v2_i; });
    return MathVector(out);
  }

  friend MathVector operator-(MathVector v1, MathVector v2) {
    InternalVector out;
    std::transform(v1.v.begin(), v1.v.end(), v2.v.begin(), out.begin(),
                   [](InternalScalar v1_i, InternalScalar v2_i) { return v1_i-v2_i; });
    return MathVector(out);
  }

  Scalar& operator[](size_t i) {
    return v[i];
  }

  InternalScalar norm() {
    InternalVector help;
    std::transform(v.begin(), v.end(), help.begin(),
                   [](InternalScalar v_i) { return v_i*v_i; });

    //std::reduce(out.begin(), out.end(), 0, std::plus<>());
    InternalScalar sum(0);
    std::for_each(help.begin(), help.end(), [&sum](InternalScalar u){sum += u;});
    return sqrt(sum);
  }

private:
  InternalVector v;
};

#endif
