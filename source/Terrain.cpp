/* Terrain.cpp
Copyright (c) 2022 by Hurleveur

Endless Sky is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.

Endless Sky is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
*/

#include "GameData.h"
#include "Ship.h"
#include "Terrain.h"

using namespace std;

const std::string Terrain::normal = "normal";

void Terrain::Load(const DataNode &node)
{
	name = node.Token(1);
	for(const DataNode &child : node)
	{
		// Get the key and value (if any).
		const string &tag = child.Token(0);
		if(child.Size() > 1)
			types.emplace(tag, child.Value(1));
		else
			child.PrintTrace("Skipping unrecognized attribute:");
	}
}



double Terrain::Get(const string type) const
{
	return types.find(type)->second;
}



const string &Terrain::Get(double value) const
{
	for(const auto &type : types)
		if(type.second == value)
			return type.first;
	return normal;
}



double Terrain::GetDefault(const Ship &ship) const
{
	if(name == "corridors")
		return max(1., ship.Attributes().Get("bunks") / 10.) +
			max(1., ship.Attributes().Get("cargo") / 25.);
	else if(name == "design layout")
		return ship.Attributes().Category() == "Transport" ? 2. :
			(ship.Attributes().Category() == "Light Freighter" || 
			ship.Attributes().Category() == "Heavy Freighter") ? 1. : 1.5;
	else
		return 1.;
}



int Terrain::CombatWidth(const Ship &ship)
{
	double corridor = ship.Attributes().Get("corridors");
	double layout = ship.Attributes().Get("design layout");
	return (corridor != 0. ? corridor : GameData::Terrains().Get("corridors")->GetDefault(ship)) *
		(layout != 0. ? layout : GameData::Terrains().Get("design layout")->GetDefault(ship));
}
