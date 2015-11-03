/*
 * Copyright Regents of the University of Minnesota, 2015.  This software is released under the following license: http://opensource.org/licenses/GPL-2.0
 * Source code originally developed at the University of Minnesota Interactive Visualization Lab (http://ivlab.cs.umn.edu).
 *
 * Code author(s):
 * 		Dan Orban (dtorban)
 */

#include <texture/SOILVolume.h>
#include <SOIL.h>

namespace Spatialize {

SOILVolume::SOILVolume(const std::vector<std::string>& filepath) {
	int width, height, depth;

	depth = filepath.size();

	for (int f = 0; f < filepath.size(); f++)
	{
		int w, h;
		unsigned char* image;
		if (f == 0)
		{
			image = SOIL_load_image(filepath[f].c_str(), &width, &height, 0, SOIL_LOAD_RGBA);
			w = width;
			h = height;
		}
		else
		{
			unsigned char* image = SOIL_load_image(filepath[f].c_str(), &w, &h, 0, SOIL_LOAD_RGBA);
		}

		if (image != NULL && w == width && h == height)
		{
			if (f == 0)
			{
				create(width, height, depth);
			}

			for (int i = 0; i < width*height*4; i++)
			{
				getDepthIndexValue(f, i) = image[i];
			}

			delete[] image;
		}
		else {
			break;
		}
	}
}

SOILVolume::~SOILVolume() {
}

} /* namespace Spatialize */
