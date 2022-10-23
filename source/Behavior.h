/* Behavior.h
Copyright (c) 2014 by Michael Zahniser

Endless Sky is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.

Endless Sky is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

#include "Point.h"

#include <memory>

class AI;
class Body;
class Command;
class Ship;


// This class includes all AI personalities expected behaviors.
class Behavior{
public:
	Behavior(AI *aiPointer);
	
	
	// Special personality behaviors.
	void DoAppeasing(const std::shared_ptr<Ship> &ship, double *threshold) const;
	void DoSwarming(Ship &ship, Command &command, std::shared_ptr<Ship> &target);
	void DoSurveillance(Ship &ship, Command &command, std::shared_ptr<Ship> &target) const;
	void DoMining(Ship &ship, Command &command);
	bool DoHarvesting(Ship &ship, Command &command);
	bool DoCloak(Ship &ship, Command &command);
	// Prevent ships from stacking on each other when many are moving in sync.
	void DoScatter(Ship &ship, Command &command);

	// Methods of moving from the current position to a desired position / orientation.
	static bool MoveToPlanet(Ship &ship, Command &command);
	static bool MoveTo(Ship &ship, Command &command, const Point &targetPosition,
		const Point &targetVelocity, double radius, double slow);
	static bool Stop(Ship &ship, Command &command, double maxSpeed = 0., const Point direction = Point());
	static void PrepareForHyperspace(Ship &ship, Command &command);
	static void CircleAround(Ship &ship, Command &command, const Body &target);
	static void Swarm(Ship &ship, Command &command, const Body &target);
	static void Refuel(Ship &ship, Command &command);
	static void KeepStation(Ship &ship, Command &command, const Body &target);
	static void Attack(Ship &ship, Command &command, const Ship &target);
	static void MoveToAttack(Ship &ship, Command &command, const Body &target);
	static void PickUp(Ship &ship, Command &command, const Body &target);

	static void Deploy(const Ship &ship, bool includingDamaged);


private:
	AI *ai;
};

#endif