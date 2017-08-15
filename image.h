#ifndef IMAGE_H
#define IMAGE_H

#define png_infopp_NULL (png_infopp)NULL
#define int_p_NULL (int*)NULL

#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>
#include <boost/gil/extension/io/jpeg_dynamic_io.hpp>

#include <boost/utility.hpp>                // for boost::tie
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/copy.hpp>

#include <array>

typedef boost::gil::rgb8_view_t  view_t;
typedef boost::gil::rgb8c_view_t cview_t;
typedef boost::gil::rgb8_pixel_t pixel_t;
typedef boost::gil::rgb8_image_t image_t;

int VDIM = 3;
int CHANNEL_DIST = boost::gil::channel_traits<boost::gil::channel_type<pixel_t>::type>::max_value() - boost::gil::channel_traits<boost::gil::channel_type<pixel_t>::type>::min_value();


/*
template <typename Vector, typename Scalar>
Vector scalar_mult(Scalar s, Vector v) {
  Vector out;
  std::transform(v.begin(), v.end(), out.begin(),
                 [s](Scalar v_i) { return s*v_i; });
  return out;
}

template <typename Vector, typename Scalar>
Vector addition(Vector v1, Vector v2) {
  Vector out;
  std::transform(v1.begin(), v1.end(), v2.begin(), out.begin(),
                 [](Scalar v1_i, Scalar v2_i) { return v1_i+v2_i; });
  return out;
}

template <typename Vector, typename Scalar>
Vector subtraction(Vector v1, Vector v2) {
  Vector out;
  std::transform(v1.begin(), v1.end(), v2.begin(), out.begin(),
                 [](Scalar v1_i, Scalar v2_i) { return v1_i-v2_i; });
  return out;
}

template <typename Vector, typename Scalar>
Scalar norm(Vector v) {
  Vector help;
  std::transform(v.begin(), v.end(), help.begin(),
                 [](Scalar v_i) { return v_i*v_i; });

  //std::reduce(out.begin(), out.end(), 0, std::plus<>());
  Scalar sum(0);
  std::for_each(help.begin(), help.end(), [&sum](Scalar u){sum += u;});
  return sqrt(sum);
}*/


#endif
