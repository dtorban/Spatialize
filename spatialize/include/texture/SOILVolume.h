/*
 * Copyright Regents of the University of Minnesota, 2015.  This software is released under the following license: http://opensource.org/licenses/GPL-2.0
 * Source code originally developed at the University of Minnesota Interactive Visualization Lab (http://ivlab.cs.umn.edu).
 *
 * Code author(s):
 * 		Dan Orban (dtorban)
 */

#ifndef SOILVOLUME_H_
#define SOILVOLUME_H_

#include "Texture.h"
#include <string>
#include <vector>

namespace Spatialize {

class SOILVolume : public Texture {
public:
	SOILVolume(const std::vector<std::string>& filepath);
	virtual ~SOILVolume();
};

} /* namespace Spatialize */

#endif /* SOILVOLUME_H_ */
