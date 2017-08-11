#include <cstdlib>
#include <cmath>
#include <random>
#include <functional>
#include <queue>
#include <vector>
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <limits>
#include <numeric>
#include <string>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "graph.h"
#include "SLIC/SLIC.h"


boost::gil::point2<int> size;
cview_t src;
view_t dst;

namespace b = boost;
namespace bg = b::gil;

void error_and_bye(std::string error_string, int error_code) {
  std::cerr << error_string << std::endl;
  exit(error_code);
}

int main(int argc, char* const argv[])
{
  std::string image_file_type = "jpg";
  std::string image_file_name;
  std::string image_file_path(argv[argc-1]);

  std::string crop(image_file_path);
  int pos = image_file_path.find_last_of('.');
  if(pos != std::string::npos) {
    image_file_type = image_file_path.substr(pos+1);
    crop = image_file_path.substr(0, pos);
  }
  pos = image_file_path.find_last_of("/\\");
  if(pos != std::string::npos)
    image_file_name = crop.substr(pos+1);
  else
    image_file_name = crop;

	bg::rgb8_image_t src_img;
	std::cout << "get image..." << std::endl;
	if(image_file_type == "jpg" || image_file_type == "jpeg")
		bg::jpeg_read_image(image_file_path, src_img);
	else if(image_file_type == "png")
		bg::png_read_image(image_file_path, src_img);
	else
		error_and_bye("unsupported image file type", -2);
	bg::rgb8_image_t dst_img(src_img); // copy
	src = const_view(src_img);

	dst = view(dst_img);// global picture size

	size.x = src.width();
	size.y = src.height();
	//printf("%b", src(0,0));
	Grid grid(size.x*size.y);
	// unsigned int (32 bits) to hold a pixel in ARGB format as follows:
	// from left to right,
	// the first 8 bits are for the alpha channel (and are ignored)
	// the next 8 bits are for the red channel
	// the next 8 bits are for the green channel
	// the last 8 bits are for the blue channel
	std::cout << "converting data" << std::endl;
	uint32_t* slic_pixel_data = new uint32_t[size.x*size.y];// 64 bit problems?
	for (int x = 0; x < size.x; ++x) {
		for (int y = 0; y < size.y; ++y) {
			uint32_t data = 0;
			uint32_t red = bg::get_color(src(x,y), bg::red_t());
			uint32_t green = bg::get_color(src(x,y), bg::green_t());
			uint32_t blue = bg::get_color(src(x,y), bg::blue_t());

			data |= 0xff;
			data <<= 8;
			data |= red;
			data <<= 8;
			data |= green;
			data <<= 8;
			data |= blue;
			slic_pixel_data[xy_to_index(x, y)] = data;
		}

	}

	std::cout << "starting slic..." << std::endl;
	int slic_number_of_superpixels = 5;//Desired number of superpixels.
	double slic_compactness = 30;//Compactness factor. use a value ranging from 10 to 40 depending on your needs. Default is 10
	int* slic_labels = new int[size.x*size.y];
	int slic_numlabels(0);
	SLIC slic;
	slic.PerformSLICO_ForGivenK(slic_pixel_data, size.x, size.y, slic_labels, slic_numlabels, slic_number_of_superpixels, slic_compactness);

	slic.DrawContoursAroundSegments(slic_pixel_data, slic_labels, size.x, size.y, 0x00ff0000);
	printf("%x", (uint32_t)(0x000000ab<<8));
	for (int x = 0; x < size.x; ++x) {
		for (int y = 0; y < size.y; ++y) {
			uint32_t data = slic_pixel_data[xy_to_index(x, y)];


			bg::get_color(dst(x,y), bg::blue_t()) = 0xff & data;
			data >>= 8;
			bg::get_color(dst(x,y), bg::green_t()) = 0xff & data;
			data >>= 8;
			bg::get_color(dst(x,y), bg::red_t()) = 0xff & data;
			data >>= 8;
		}
	}
	if(image_file_type == "jpg" || image_file_type == "jpeg")
		bg::jpeg_write_view(image_file_name+"_slic_contours"+"."+image_file_type, bg::const_view(dst_img));
	else if(image_file_type == "png")
		bg::png_write_view(image_file_name+"_slic_contours"+"."+image_file_type, bg::const_view(dst_img));


	std::cout << "printing result..." << std::endl;
	std::ofstream segment_file;
	segment_file.open ("slic_labels.txt");
	int print_width = floor(log10(slic_numlabels)) + 2;
	for (int x = 0; x < size.x; ++x) {
		for (int y = 0; y < size.y; ++y) {
			segment_file << std::setw(print_width) << std::left << std::setfill(static_cast<char>(' ')) << slic_labels[xy_to_index(x, y)];
		}
		segment_file << std::endl;
	}
}
