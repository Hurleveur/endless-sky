/* Terrain.h
Copyright (c) 2022 by Hurleveur

Endless Sky is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.

Endless Sky is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
*/

#ifndef TERRAIN_H_
#define TERRAIN_H_

#include "DataNode.h"

#include <map>
#include <string>

class Ship;

class Terrain {
public:
	void Load(const DataNode &node);

	double Get(const std::string type) const;

	double GetDefault(const Ship &ship) const;


private:
	std::map<const std::string, double> types;
	std::string name;
};

#endif
