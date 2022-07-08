/* CaptureOdds.cpp
Copyright (c) 2014 by Michael Zahniser

Endless Sky is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.

Endless Sky is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
*/

#include "CaptureOdds.h"

#include "GameData.h"
#include "Government.h"
#include "Outfit.h"
#include "Ship.h"
#include "Terrain.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

using namespace std;

namespace {
	double CalculateOutfitPower(const Ship &ship, const Ship &fightingPlace, bool isDefender, double value, const Outfit *weapon)
	{
		for(const auto &terrain : GameData::Terrains())
		{
			double terrainVuln = weapon->Get(terrain.first + " vulnerability");
			terrainVuln = terrainVuln ? terrainVuln : terrain.second.GetDefault(fightingPlace);
			if(terrainVuln)
				value *= ship.Attributes().Get(terrain.first) * terrainVuln;
		}
		// These ones depend on security systems installed.
		static const vector<std::string> names = {"combat environmental suit", "security alcove"};
		for(unsigned i = 0; i < names.size(); ++i)
		{
			double vulnerability = weapon->Get(names[i]);
			double shipAttribute = fightingPlace.Attributes().Get(names[i]);
			value *= vulnerability * shipAttribute;
		}
		return max(0.1, value);
	}
}



// Constructor.
CaptureOdds::CaptureOdds(const Ship &attacker, const Ship &defender)
{
	powerA = Power(attacker, defender, false);
	powerD = Power(defender, attacker, true);
	Calculate();
}



// Refreshes the lookup tables when a consumable item is used.
void CaptureOdds::RefreshOdds(const Ship &attacker, const Ship &defender, bool isDefender)
{
	if(isDefender)
		powerD = Power(defender, attacker, true);
	else
		powerA = Power(attacker, defender, false);
	Calculate();
}



// Get the odds of the attacker winning if the two ships have the given
// number of crew members remaining.
double CaptureOdds::Odds(int attackingCrew, int defendingCrew) const
{
	// If the defender has no crew remaining, odds are 100%.
	if(!defendingCrew)
		return 1.;

	// Make sure the input is within range, with the special constraint that the
	// attacker can never succeed if they don't have two crew left (one to pilot
	// each of the ships).
	int index = Index(attackingCrew, defendingCrew);
	if(attackingCrew < 2 || index < 0)
		return 0.;

	return capture[index];
}



// Get the expected number of casualties for the attacker in the remainder of
// the battle if the two ships have the given number of crew remaining.
double CaptureOdds::AttackerCasualties(int attackingCrew, int defendingCrew) const
{
	// If the attacker has fewer than two crew, they cannot attack. If the
	// defender has no crew, they cannot defend (so casualties will be zero).
	int index = Index(attackingCrew, defendingCrew);
	if(attackingCrew < 2 || !defendingCrew || index < 0)
		return 0.;

	return casualtiesA[index];
}



// Get the expected number of casualties for the defender in the remainder of
// the battle if the two ships have the given number of crew remaining.
double CaptureOdds::DefenderCasualties(int attackingCrew, int defendingCrew) const
{
	// If the attacker has fewer than two crew, they cannot attack. If the
	// defender has no crew, they cannot defend (so casualties will be zero).
	int index = Index(attackingCrew, defendingCrew);
	if(attackingCrew < 2 || !defendingCrew || index < 0)
		return 0.;

	return casualtiesD[index];
}



// Get the total power (inherent crew power plus bonuses from hand to hand
// weapons) for the attacker when they have the given number of crew remaining.
double CaptureOdds::AttackerPower(int attackingCrew) const
{
	if(static_cast<unsigned>(attackingCrew - 1) >= powerA.size())
		return 0.;

	return powerA[attackingCrew - 1];
}



// Get the total power (inherent crew power plus bonuses from hand to hand
// weapons) for the defender when they have the given number of crew remaining.
double CaptureOdds::DefenderPower(int defendingCrew) const
{
	if(static_cast<unsigned>(defendingCrew - 1) >= powerD.size())
		return 0.;

	return powerD[defendingCrew - 1];
}



// We know the stronger weapon is the last used one.
const Outfit *CaptureOdds::LastUsedWeapon(const Ship &ship, bool isDefender, const char *attribute) const
{
	const string stat = (isDefender ? "capture defense" : "capture attack");
	const string attributes = attribute;
	const Outfit *best = nullptr;
	for(const auto &it : ship.Outfits())
		if(it.second > 0 && ((attributes.empty() && !it.first->Get("consumable") && !it.first->Get("defense"))
				|| it.first->Get(attributes)) && (!best || it.first->Get(stat) > best->Get(stat)))
			best = it.first;
	return best;
}



// Generate the lookup tables.
void CaptureOdds::Calculate()
{
	if(powerD.empty() || powerA.empty())
		return;

	// The first row represents the case where the attacker has only one crew left.
	// In that case, the defending ship can never be successfully captured.
	capture.resize(powerD.size(), 0.);
	casualtiesA.resize(powerD.size(), 0.);
	casualtiesD.resize(powerD.size(), 0.);
	unsigned up = 0;
	for(unsigned a = 2; a <= powerA.size(); ++a)
	{
		double ap = powerA[a - 1];
		// Special case: odds for defender having only one person,
		// because 0 people is outside the end of the table.
		double odds = ap / (ap + powerD[0]);
		capture.push_back(odds + (1. - odds) * capture[up]);
		casualtiesA.push_back((1. - odds) * (casualtiesA[up] + 1.));
		casualtiesD.push_back(odds + (1. - odds) * casualtiesD[up]);
		++up;

		// Loop through each number of crew the defender might have.
		for(unsigned d = 2; d <= powerD.size(); ++d)
		{
			// This is  basic 2D dynamic program, where each value is based on
			// the odds of success and the values for one fewer crew members
			// for the defender or the attacker depending on who wins.
			odds = ap / (ap + powerD[d - 1]);
			capture.push_back(odds * capture.back() + (1. - odds) * capture[up]);
			casualtiesA.push_back(odds * casualtiesA.back() + (1. - odds) * (casualtiesA[up] + 1.));
			casualtiesD.push_back(odds * (casualtiesD.back() + 1.) + (1. - odds) * casualtiesD[up]);
			++up;
		}
	}
}



// Map the given crew complements to an index in the lookup tables. There is no
// row in the table for 0 crew on either ship.
int CaptureOdds::Index(int attackingCrew, int defendingCrew) const
{
	if(static_cast<unsigned>(attackingCrew - 1) > powerA.size())
		return -1;
	if(static_cast<unsigned>(defendingCrew - 1) > powerD.size())
		return -1;

	return (attackingCrew - 1) * powerD.size() + (defendingCrew - 1);
}



// Generate a vector with the total power of the given ship's crew when any
// number of them are left, either for attacking or for defending.
vector<double> CaptureOdds::Power(const Ship &ship, const Ship &other, bool isDefender)
{
	vector<double> consumable;
	vector<double> defense;
	vector<double> power;
	if(!ship.Crew())
		return power;

	// Check for any outfits that assist with attacking or defending:
	const string attribute = (isDefender ? "capture defense" : "capture attack");
	const double crewPower = (isDefender ?
		ship.GetGovernment()->CrewDefense() : ship.GetGovernment()->CrewAttack());

	// Each crew member can wield one weapon. They use the most powerful ones
	// that can be wielded by the remaining crew.
	for(const auto &it : ship.Outfits())
	{
		double value = it.first->Get(attribute);
		if(value > 0. && it.second > 0)
		{
			value = CalculateOutfitPower(ship, other, value, isDefender, it.first);
			if(it.first->Attributes().Get("consumable"))
				consumable.insert(consumable.end(), it.second, value);
			else if(it.first->Attributes().Get("defense"))
				defense.insert(defense.end(), it.second, value);
			else
				power.insert(power.end(), it.second, value);
		}
	}
	// Use the best weapons first.
	sort(consumable.begin(), consumable.end(), greater<double>());
	sort(defense.begin(), defense.end(), greater<double>());
	sort(power.begin(), power.end(), greater<double>());

	// Resize the vector to have exactly one entry per crew member.
	consumable.resize(ship.Crew(), 0.);
	defense.resize(ship.Crew(), 0.);
	power.resize(ship.Crew(), 0.);

	// Calculate partial sums. That is, power[N - 1] should be your total crew
	// power when you have N crew left.
	power.front() += crewPower + defense.front() + consumable.front();
	for(unsigned i = 1; i < power.size(); ++i)
		power[i] += power[i - 1] + crewPower + defense[i - 1] + consumable[i - 1];

	return power;
}