/* Behavior.cpp
Copyright (c) 2014 by Michael Zahniser
Copyright (c) 2022 by Hurleveur

Endless Sky is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.

Endless Sky is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include "Behavior.h"

#include "Angle.h"
#include "AI.h"
#include "Body.h"
#include "Command.h"
#include "Government.h"
#include "Flotsam.h"
#include "Messages.h"
#include "Minable.h"
#include "pi.h"
#include "Point.h"
#include "Random.h"
#include "Ship.h"
#include "ShipEvent.h"
#include "System.h"

#include <algorithm>
#include <cmath>

using namespace std;

Behavior::Behavior(AI *aiPointer) : ai(aiPointer)
{
}



// "Appeasing" ships will dump cargo after being injured, if they are being targeted.
void Behavior::DoAppeasing(const shared_ptr<Ship> &ship, double *threshold) const
{
	double health = .5 * ship->Shields() + ship->Hull();
	if(1. - health <= *threshold)
		return;

	const auto enemies = ai->GetShipsList(*ship, true);
	if(none_of(enemies.begin(), enemies.end(), [&ship](const Ship *foe) noexcept -> bool
			{ return !foe->IsDisabled() && foe->GetTargetShip() == ship; }))
		return;

	int toDump = 11 + (1. - health) * .5 * ship->Cargo().Size();
	for(auto &&commodity : ship->Cargo().Commodities())
		if(commodity.second && toDump > 0)
		{
			int dumped = min(commodity.second, toDump);
			ship->Jettison(commodity.first, dumped, true);
			toDump -= dumped;
		}

	Messages::Add(ship->GetGovernment()->GetName() + " " + ship->Noun() + " \"" + ship->Name()
		+ "\": Please, just take my cargo and leave me alone.", Messages::Importance::Low);

	*threshold = (1. - health) + .1;
}



// Find a target ship to flock around at high speed.
void Behavior::DoSwarming(Ship &ship, Command &command, shared_ptr<Ship> &target)
{
	// Find a new ship to target on average every 10 seconds, or if the current target
	// is no longer eligible. If landing, release the old target so others can swarm it.
	if(ship.IsLanding() || !target || !AI::CanSwarm(ship, *target) || !Random::Int(600))
	{
		if(target)
		{
			// Allow another swarming ship to consider the target.
			auto sit = ai->swarmCount.find(target.get());
			if(sit != ai->swarmCount.end() && sit->second > 0)
				--sit->second;
			// Release the current target.
			target.reset();
			ship.SetTargetShip(target);
		}
		// If here just because we are about to land, do not seek a new target.
		if(ship.IsLanding())
			return;

		int lowestCount = 7;
		// Consider swarming around non-hostile ships in the same system.
		const auto others = ai->GetShipsList(ship, false);
		for(auto *other : others)
			if(!other->GetPersonality().IsSwarming())
			{
				// Prefer to swarm ships that are not already being heavily swarmed.
				int count = ai->swarmCount[other] + Random::Int(4);
				if(count < lowestCount)
				{
					target = other->shared_from_this();
					lowestCount = count;
				}
			}
		ship.SetTargetShip(target);
		if(target)
			++ai->swarmCount[target.get()];
	}
	// If a friendly ship to flock with was not found, return to an available planet.
	if(target)
		Swarm(ship, command, *target);
	else if(ship.Zoom() == 1.)
		Refuel(ship, command);
}



void Behavior::DoSurveillance(Ship &ship, Command &command, shared_ptr<Ship> &target) const
{
	// Since DoSurveillance is called after target-seeking and firing, if this
	// ship has a target, that target is guaranteed to be targetable.
	if(target && (target->GetSystem() != ship.GetSystem() || target->IsEnteringHyperspace()))
	{
		target.reset();
		ship.SetTargetShip(target);
	}
	// If you have a hostile target, pursuing and destroying it has priority.
	if(target && ship.GetGovernment()->IsEnemy(target->GetGovernment()))
	{
		// Automatic aiming and firing already occurred.
		ai->MoveIndependent(ship, command);
		return;
	}

	// Choose a surveillance behavior.
	if(ship.GetTargetSystem())
	{
		// Unload surveillance drones in this system before leaving.
		PrepareForHyperspace(ship, command);
		command |= Command::JUMP;
		if(ship.HasBays())
		{
			command |= Command::DEPLOY;
			Deploy(ship, false);
		}
	}
	else if(ship.GetTargetStellar())
	{
		// Approach the planet and "land" on it (i.e. scan it).
		MoveToPlanet(ship, command);
		double atmosphereScan = ship.Attributes().Get("atmosphere scan");
		double distance = ship.Position().Distance(ship.GetTargetStellar()->Position());
		if(distance < atmosphereScan && !Random::Int(100))
			ship.SetTargetStellar(nullptr);
		else
			command |= Command::LAND;
	}
	else if(target)
	{
		// Approach and scan the targeted, friendly ship's cargo or outfits.
		bool cargoScan = ship.Attributes().Get("cargo scan power");
		bool outfitScan = ship.Attributes().Get("outfit scan power");
		// If the pointer to the target ship exists, it is targetable and in-system.
		bool mustScanCargo = cargoScan && !ai->Has(ship, target, ShipEvent::SCAN_CARGO);
		bool mustScanOutfits = outfitScan && !ai->Has(ship, target, ShipEvent::SCAN_OUTFITS);
		if(!mustScanCargo && !mustScanOutfits)
			ship.SetTargetShip(shared_ptr<Ship>());
		else
		{
			CircleAround(ship, command, *target);
			command |= Command::SCAN;
		}
	}
	else
	{
		const System *system = ship.GetSystem();
		const Government *gov = ship.GetGovernment();

		// Consider scanning any non-hostile ship in this system that you haven't yet personally scanned.
		vector<Ship *> targetShips;
		bool cargoScan = ship.Attributes().Get("cargo scan power");
		bool outfitScan = ship.Attributes().Get("outfit scan power");
		if(cargoScan || outfitScan)
			for(const auto &grit : ai->governmentRosters)
			{
				if(gov == grit.first || gov->IsEnemy(grit.first))
					continue;
				for(const auto &it : grit.second)
				{
					auto ptr = it->shared_from_this();
					if((!cargoScan || ai->Has(ship, ptr, ShipEvent::SCAN_CARGO))
							&& (!outfitScan || ai->Has(ship, ptr, ShipEvent::SCAN_OUTFITS)))
						continue;

					if(it->IsTargetable())
						targetShips.emplace_back(it);
				}
			}

		// Consider scanning any planetary object in the system, if able.
		vector<const StellarObject *> targetPlanets;
		double atmosphereScan = ship.Attributes().Get("atmosphere scan");
		if(atmosphereScan)
			for(const StellarObject &object : system->Objects())
				if(object.HasSprite() && !object.IsStar() && !object.IsStation())
					targetPlanets.push_back(&object);

		// If this ship can jump away, consider traveling to a nearby system.
		vector<const System *> targetSystems;
		// TODO: These ships cannot travel through wormholes?
		if(ship.JumpsRemaining(false))
		{
			const auto &links = ship.Attributes().Get("jump drive") ? system->JumpNeighbors(ship.JumpRange()) : system->Links();
			targetSystems.insert(targetSystems.end(), links.begin(), links.end());
		}

		unsigned total = targetShips.size() + targetPlanets.size() + targetSystems.size();
		if(!total)
		{
			// If there is nothing for this ship to scan, have it hold still
			// instead of drifting away from the system center.
			Stop(ship, command);
			return;
		}

		unsigned index = Random::Int(total);
		if(index < targetShips.size())
			ship.SetTargetShip(targetShips[index]->shared_from_this());
		else
		{
			index -= targetShips.size();
			if(index < targetPlanets.size())
				ship.SetTargetStellar(targetPlanets[index]);
			else
				ship.SetTargetSystem(targetSystems[index - targetPlanets.size()]);
		}
	}
}



void Behavior::DoMining(Ship &ship, Command &command)
{
	// This function is only called for ships that are in the player's system.
	// Update the radius that the ship is searching for asteroids at.
	bool isNew = !ai->miningAngle.count(&ship);
	Angle &angle = ai->miningAngle[&ship];
	if(isNew)
	{
		angle = Angle::Random();
		ai->miningRadius[&ship] = ship.GetSystem()->AsteroidBeltRadius();
	}
	angle += Angle::Random(1.) - Angle::Random(1.);
	double radius = ai->miningRadius[&ship] * pow(2., angle.Unit().X());

	shared_ptr<Minable> target = ship.GetTargetAsteroid();
	if(!target || target->Velocity().Length() > ship.MaxVelocity())
	{
		for(const shared_ptr<Minable> &minable : ai->minables)
		{
			Point offset = minable->Position() - ship.Position();
			// Target only nearby minables that are within 45deg of the current heading
			// and not moving faster than the ship can catch.
			if(offset.Length() < 800. && offset.Unit().Dot(ship.Facing().Unit()) > .7
					&& minable->Velocity().Dot(offset.Unit()) < ship.MaxVelocity())
			{
				target = minable;
				ship.SetTargetAsteroid(target);
				break;
			}
		}
	}
	if(target)
	{
		// If the asteroid has moved well out of reach, stop tracking it.
		if(target->Position().Distance(ship.Position()) > 1600.)
			ship.SetTargetAsteroid(nullptr);
		else
		{
			MoveToAttack(ship, command, *target);
			ai->AutoFire(ship, ai->firingCommands, *target);
			return;
		}
	}

	Point heading = Angle(30.).Rotate(ship.Position().Unit() * radius) - ship.Position();
	command.SetTurn(ai->TurnToward(ship, heading));
	if(ship.Velocity().Dot(heading.Unit()) < .7 * ship.MaxVelocity())
		command |= Command::FORWARD;
}



bool Behavior::DoHarvesting(Ship &ship, Command &command)
{
	// If the ship has no target to pick up, do nothing.
	shared_ptr<Flotsam> target = ship.GetTargetFlotsam();
	if(target && ship.Cargo().Free() < target->UnitSize())
	{
		target.reset();
		ship.SetTargetFlotsam(target);
	}
	if(!target)
	{
		// Only check for new targets every 10 frames, on average.
		if(Random::Int(10))
			return false;

		// Don't chase anything that will take more than 10 seconds to reach.
		double bestTime = 600.;
		for(const shared_ptr<Flotsam> &it : ai->flotsam)
		{
			if(ship.Cargo().Free() < it->UnitSize())
				continue;
			// Only pick up flotsam that is nearby and that you are facing toward.
			Point p = it->Position() - ship.Position();
			double range = p.Length();
			if(range > 800. || (range > 100. && p.Unit().Dot(ship.Facing().Unit()) < .9))
				continue;

			// Estimate how long it would take to intercept this flotsam.
			Point v = it->Velocity() - ship.Velocity();
			double vMax = ship.MaxVelocity();
			double time = ai->RendezvousTime(p, v, vMax);
			if(std::isnan(time))
				continue;

			double degreesToTurn = TO_DEG * acos(min(1., max(-1., p.Unit().Dot(ship.Facing().Unit()))));
			time += degreesToTurn / ship.TurnRate();
			if(time < bestTime)
			{
				bestTime = time;
				target = it;
			}
		}
		if(!target)
			return false;

		ship.SetTargetFlotsam(target);
	}
	// Deploy any carried ships to improve maneuverability.
	if(ship.HasBays())
	{
		command |= Command::DEPLOY;
		Deploy(ship, false);
	}

	PickUp(ship, command, *target);
	return true;
}



// Check if this ship should cloak. Returns true if this ship decided to run away while cloaking.
bool Behavior::DoCloak(Ship &ship, Command &command)
{
	if(ship.Attributes().Get("cloak"))
	{
		// Never cloak if it will cause you to be stranded.
		const Outfit &attributes = ship.Attributes();
		double fuelCost = attributes.Get("cloaking fuel") + attributes.Get("fuel consumption")
			- attributes.Get("fuel generation");
		if(attributes.Get("cloaking fuel") && !attributes.Get("ramscoop"))
		{
			double fuel = ship.Fuel() * attributes.Get("fuel capacity");
			int steps = ceil((1. - ship.Cloaking()) / attributes.Get("cloak"));
			// Only cloak if you will be able to fully cloak and also maintain it
			// for as long as it will take you to reach full cloak.
			fuel -= fuelCost * (1 + 2 * steps);
			if(fuel < ship.JumpFuel())
				return false;
		}

		// If your parent has chosen to cloak, cloak and rendezvous with them.
		const shared_ptr<const Ship> &parent = ship.GetParent();
		if(parent && parent->Commands().Has(Command::CLOAK) && parent->GetSystem() == ship.GetSystem()
				&& !parent->GetGovernment()->IsEnemy(ship.GetGovernment()))
		{
			command |= Command::CLOAK;
			KeepStation(ship, command, *parent);
			return true;
		}

		// Otherwise, always cloak if you are in imminent danger.
		static const double MAX_RANGE = 10000.;
		double range = MAX_RANGE;
		const Ship *nearestEnemy = nullptr;
		// Find the nearest targetable, in-system enemy that could attack this ship.
		const auto enemies = ai->GetShipsList(ship, true);
		for(const auto &foe : enemies)
			if(!foe->IsDisabled())
			{
				double distance = ship.Position().Distance(foe->Position());
				if(distance < range)
				{
					range = distance;
					nearestEnemy = foe;
				}
			}

		// If this ship has started cloaking, it must get at least 40% repaired
		// or 40% farther away before it begins decloaking again.
		double hysteresis = ship.Commands().Has(Command::CLOAK) ? .4 : 0.;
		// If cloaking costs nothing, and no one has asked you for help, cloak at will.
		bool cloakFreely = (fuelCost <= 0.) && !ship.GetShipToAssist();
		// If this ship is injured / repairing, it should cloak while under threat.
		bool cloakToRepair = (ship.Health() < ai->RETREAT_HEALTH + hysteresis)
				&& (attributes.Get("shield generation") || attributes.Get("hull repair rate"));
		if(cloakToRepair && (cloakFreely || range < 2000. * (1. + hysteresis)))
		{
			command |= Command::CLOAK;
			// Move away from the nearest enemy.
			if(nearestEnemy)
			{
				Point safety;
				// TODO: This could use an "Avoid" method, to account for other in-system hazards.
				// Simple approximation: move equally away from both the system center and the
				// nearest enemy, until the constrainment boundary is reached.
				if(ship.GetPersonality().IsUnconstrained() || !ai->fenceCount.count(&ship))
					safety = 2 * ship.Position().Unit() - nearestEnemy->Position().Unit();
				else
					safety = -ship.Position().Unit();

				safety *= ship.MaxVelocity();
				MoveTo(ship, command, ship.Position() + safety, safety, 1., .8);
				return true;
			}
		}
		// Choose to cloak if there are no enemies nearby and cloaking is sensible.
		if(range == MAX_RANGE && cloakFreely && !ship.GetTargetShip())
			command |= Command::CLOAK;
	}
	return false;
}



void Behavior::DoScatter(Ship &ship, Command &command)
{
	if(!command.Has(Command::FORWARD))
		return;

	double turnRate = ship.TurnRate();
	double acceleration = ship.Acceleration();
	// TODO: If there are many ships, use CollisionSet::Circle or another
	// suitable method to limit which ships are checked.
	for(const shared_ptr<Ship> &other : ai->ships)
	{
		// Do not scatter away from yourself, or ships in other systems.
		if(other.get() == &ship || other->GetSystem() != ship.GetSystem())
			continue;

		// Check for any ships that have nearly the same movement profile as
		// this ship and are in nearly the same location.
		Point offset = other->Position() - ship.Position();
		if(offset.LengthSquared() > 400.)
			continue;
		if(fabs(other->TurnRate() / turnRate - 1.) > .05)
			continue;
		if(fabs(other->Acceleration() / acceleration - 1.) > .05)
			continue;

		// Move away from this ship. What side of me is it on?
		command.SetTurn(offset.Cross(ship.Facing().Unit()) > 0. ? 1. : -1.);
		return;
	}
}


void Behavior::CircleAround(Ship &ship, Command &command, const Body &target)
{
	Point direction = target.Position() - ship.Position();
	command.SetTurn(AI::TurnToward(ship, direction));

	double length = direction.Length();
	if(length > 200. && ship.Facing().Unit().Dot(direction) >= 0.)
	{
		command |= Command::FORWARD;

		// If the ship is far away enough the ship should use the afterburner.
		if(length > 750. && AI::ShouldUseAfterburner(ship))
			command |= Command::AFTERBURNER;
	}
}



void Behavior::Swarm(Ship &ship, Command &command, const Body &target)
{
	Point direction = target.Position() - ship.Position();
	double maxSpeed = ship.MaxVelocity();
	double rendezvousTime = AI::RendezvousTime(direction, target.Velocity(), maxSpeed);
	if(std::isnan(rendezvousTime) || rendezvousTime > 600.)
		rendezvousTime = 600.;
	direction += rendezvousTime * target.Velocity();
	MoveTo(ship, command, target.Position() + direction, .5 * maxSpeed * direction.Unit(), 50., 2.);
}



// Prefer your parent's target planet for refueling, but if it and your current
// target planet can't fuel you, try to find one that can.
void Behavior::Refuel(Ship &ship, Command &command)
{
	const StellarObject *parentTarget = (ship.GetParent() ? ship.GetParent()->GetTargetStellar() : nullptr);
	if(AI::CanRefuel(ship, parentTarget))
		ship.SetTargetStellar(parentTarget);
	else if(!AI::CanRefuel(ship, ship.GetTargetStellar()))
		ship.SetTargetStellar(AI::GetRefuelLocation(ship));

	if(ship.GetTargetStellar())
	{
		MoveToPlanet(ship, command);
		command |= Command::LAND;
	}
}



void Behavior::KeepStation(Ship &ship, Command &command, const Body &target)
{
	// Constants:
	static const double MAX_TIME = 600.;
	static const double LEAD_TIME = 500.;
	static const double POSITION_DEADBAND = 200.;
	static const double VELOCITY_DEADBAND = 1.5;
	static const double TIME_DEADBAND = 120.;
	static const double THRUST_DEADBAND = .5;

	// Current properties of the two ships:
	double maxV = ship.MaxVelocity();
	double accel = ship.Acceleration();
	double turn = ship.TurnRate();
	double mass = ship.Mass();
	Point unit = ship.Facing().Unit();
	double currentAngle = ship.Facing().Degrees();
	// This is where we want to be relative to where we are now:
	Point velocityDelta = target.Velocity() - ship.Velocity();
	Point positionDelta = target.Position() + LEAD_TIME * velocityDelta - ship.Position();
	double positionSize = positionDelta.Length();
	double positionWeight = positionSize / (positionSize + POSITION_DEADBAND);
	// This is how fast we want to be going relative to how fast we're going now:
	velocityDelta -= unit * VELOCITY_DEADBAND;
	double velocitySize = velocityDelta.Length();
	double velocityWeight = velocitySize / (velocitySize + VELOCITY_DEADBAND);

	// Time it will take (roughly) to move to the target ship:
	double positionTime = AI::RendezvousTime(positionDelta, target.Velocity(), maxV);
	if(std::isnan(positionTime) || positionTime > MAX_TIME)
		positionTime = MAX_TIME;
	Point rendezvous = positionDelta + target.Velocity() * positionTime;
	double positionAngle = Angle(rendezvous).Degrees();
	positionTime += AI::AngleDiff(currentAngle, positionAngle) / turn;
	positionTime += (rendezvous.Unit() * maxV - ship.Velocity()).Length() / accel;
	// If you are very close, stop trying to adjust:
	positionTime *= positionWeight * positionWeight;

	// Time it will take (roughly) to adjust your velocity to match the target:
	double velocityTime = velocityDelta.Length() / accel;
	double velocityAngle = Angle(velocityDelta).Degrees();
	velocityTime += AI::AngleDiff(currentAngle, velocityAngle) / turn;
	// If you are very close, stop trying to adjust:
	velocityTime *= velocityWeight * velocityWeight;

	// Focus on matching position or velocity depending on which will take longer.
	double totalTime = positionTime + velocityTime + TIME_DEADBAND;
	positionWeight = positionTime / totalTime;
	velocityWeight = velocityTime / totalTime;
	double facingWeight = TIME_DEADBAND / totalTime;

	// Determine the angle we want to face, interpolating smoothly between three options.
	Point facingGoal = rendezvous.Unit() * positionWeight
		+ velocityDelta.Unit() * velocityWeight
		+ target.Facing().Unit() * facingWeight;
	double targetAngle = Angle(facingGoal).Degrees() - currentAngle;
	if(abs(targetAngle) > 180.)
		targetAngle += (targetAngle < 0. ? 360. : -360.);
	// Avoid "turn jitter" when position & velocity are well-matched.
	bool changedDirection = (signbit(ship.Commands().Turn()) != signbit(targetAngle));
	double targetTurn = abs(targetAngle / turn);
	double lastTurn = abs(ship.Commands().Turn());
	if(lastTurn && (changedDirection || (lastTurn < 1. && targetTurn > lastTurn)))
	{
		// Keep the desired turn direction, but damp the per-frame turn rate increase.
		double dampedTurn = (changedDirection ? 0. : lastTurn) + min(.025, targetTurn);
		command.SetTurn(copysign(dampedTurn, targetAngle));
	}
	else if(targetTurn < 1.)
		command.SetTurn(copysign(targetTurn, targetAngle));
	else
		command.SetTurn(targetAngle);

	// Determine whether to apply thrust.
	Point drag = ship.Velocity() * (ship.Attributes().Get("drag") / mass);
	if(ship.Attributes().Get("reverse thrust"))
	{
		// Don't take drag into account when reverse thrusting, because this
		// estimate of how it will be applied can be quite inaccurate.
		Point a = (unit * (-ship.Attributes().Get("reverse thrust") / mass)).Unit();
		double direction = positionWeight * positionDelta.Dot(a) / POSITION_DEADBAND
			+ velocityWeight * velocityDelta.Dot(a) / VELOCITY_DEADBAND;
		if(direction > THRUST_DEADBAND)
		{
			command |= Command::BACK;
			return;
		}
	}
	Point a = (unit * accel - drag).Unit();
	double direction = positionWeight * positionDelta.Dot(a) / POSITION_DEADBAND
		+ velocityWeight * velocityDelta.Dot(a) / VELOCITY_DEADBAND;
	if(direction > THRUST_DEADBAND)
		command |= Command::FORWARD;
}



void Behavior::Attack(Ship &ship, Command &command, const Ship &target)
{
	// First, figure out what your shortest-range weapon is.
	double shortestRange = 4000.;
	bool isArmed = false;
	bool hasAmmo = false;
	double minSafeDistance = 0.;
	for(const Hardpoint &hardpoint : ship.Weapons())
	{
		const Weapon *weapon = hardpoint.GetOutfit();
		if(weapon && !hardpoint.IsAntiMissile())
		{
			isArmed = true;
			bool hasThisAmmo = (!weapon->Ammo() || ship.OutfitCount(weapon->Ammo()));
			hasAmmo |= hasThisAmmo;

			// Exploding weaponry that can damage this ship requires special
			// consideration (while we have the ammo to use the weapon).
			if(hasThisAmmo && weapon->BlastRadius() && !weapon->IsSafe())
				minSafeDistance = max(weapon->BlastRadius() + weapon->TriggerRadius(), minSafeDistance);

			// The missile boat AI should be applied at 1000 pixels range if
			// all weapons are homing or turrets, and at 2000 if not.
			double multiplier = (hardpoint.IsHoming() || hardpoint.IsTurret()) ? 1. : .5;
			shortestRange = min(multiplier * weapon->Range(), shortestRange);
		}
	}
	// If this ship was using the missile boat AI to run away and bombard its
	// target from a distance, have it stop running once it is out of ammo. This
	// is not realistic, but it's a whole lot less annoying for the player when
	// they are trying to hunt down and kill the last missile boat in a fleet.
	if(isArmed && !hasAmmo)
		shortestRange = 0.;

	// Deploy any fighters you are carrying.
	if(!ship.IsYours() && ship.HasBays())
	{
		command |= Command::DEPLOY;
		Deploy(ship, false);
	}

	// If this ship has only long-range weapons, or some weapons have a
	// blast radius, it should keep some distance instead of closing in.
	Point d = (target.Position() + target.Velocity()) - (ship.Position() + ship.Velocity());
	if((minSafeDistance > 0. || shortestRange > 1000.)
			&& d.Length() < max(1.25 * minSafeDistance, .5 * shortestRange))
	{
		// If this ship can use reverse thrusters, consider doing so.
		double reverseSpeed = ship.MaxReverseVelocity();
		if(reverseSpeed && (reverseSpeed >= min(target.MaxVelocity(), ship.MaxVelocity())
				|| target.Velocity().Dot(-d.Unit()) <= reverseSpeed))
		{
			command.SetTurn(AI::TurnToward(ship, d));
			if(ship.Facing().Unit().Dot(d) >= 0.)
				command |= Command::BACK;
		}
		else
		{
			command.SetTurn(AI::TurnToward(ship, -d));
			if(ship.Facing().Unit().Dot(d) <= 0.)
				command |= Command::FORWARD;
		}
		return;
	}

	MoveToAttack(ship, command, target);
}



void Behavior::MoveToAttack(Ship &ship, Command &command, const Body &target)
{
	Point d = target.Position() - ship.Position();

	// First of all, aim in the direction that will hit this target.
	command.SetTurn(AI::TurnToward(ship, AI::TargetAim(ship, target)));

	// Calculate this ship's "turning radius"; that is, the smallest circle it
	// can make while at full speed.
	double stepsInFullTurn = 360. / ship.TurnRate();
	double circumference = stepsInFullTurn * ship.Velocity().Length();
	double diameter = max(200., circumference / PI);

	// If the ship has reverse thrusters and the target is behind it, we can
	// use them to reach the target more quickly.
	if(ship.Facing().Unit().Dot(d.Unit()) < -.75 && ship.Attributes().Get("reverse thrust"))
		command |= Command::BACK;
	// This isn't perfect, but it works well enough.
	else if((ship.Facing().Unit().Dot(d) >= 0. && d.Length() > diameter)
			|| (ship.Velocity().Dot(d) < 0. && ship.Facing().Unit().Dot(d.Unit()) >= .9))
		command |= Command::FORWARD;

	// Use an equipped afterburner if possible.
	if(command.Has(Command::FORWARD) && d.Length() < 1000. && AI::ShouldUseAfterburner(ship))
		command |= Command::AFTERBURNER;
}



void Behavior::PickUp(Ship &ship, Command &command, const Body &target)
{
	// Figure out the target's velocity relative to the ship.
	Point p = target.Position() - ship.Position();
	Point v = target.Velocity() - ship.Velocity();
	double vMax = ship.MaxVelocity();

	// Estimate where the target will be by the time we reach it.
	double time = AI::RendezvousTime(p, v, vMax);
	if(std::isnan(time))
		time = p.Length() / vMax;
	double degreesToTurn = TO_DEG * acos(min(1., max(-1., p.Unit().Dot(ship.Facing().Unit()))));
	time += degreesToTurn / ship.TurnRate();
	p += v * time;

	// Move toward the target.
	command.SetTurn(AI::TurnToward(ship, p));
	double dp = p.Unit().Dot(ship.Facing().Unit());
	if(dp > .7)
		command |= Command::FORWARD;

	// Use the afterburner if it will not cause you to miss your target.
	double squareDistance = p.LengthSquared();
	if(command.Has(Command::FORWARD) && AI::ShouldUseAfterburner(ship))
		if(dp > max(.9, min(.9999, 1. - squareDistance / 10000000.)))
			command |= Command::AFTERBURNER;
}



bool Behavior::MoveToPlanet(Ship &ship, Command &command)
{
	if(!ship.GetTargetStellar())
		return false;

	const Point &target = ship.GetTargetStellar()->Position();
	return MoveTo(ship, command, target, Point(), ship.GetTargetStellar()->Radius(), 1.);
}



// Instead of moving to a point with a fixed location, move to a moving point (Ship = position + velocity)
bool Behavior::MoveTo(Ship &ship, Command &command, const Point &targetPosition,
	const Point &targetVelocity, double radius, double slow)
{
	const Point &position = ship.Position();
	const Point &velocity = ship.Velocity();
	const Angle &angle = ship.Facing();
	Point dp = targetPosition - position;
	Point dv = targetVelocity - velocity;

	double speed = dv.Length();

	bool isClose = (dp.Length() < radius);
	if(isClose && speed < slow)
		return true;

	bool shouldReverse = false;
	dp = targetPosition - AI::StoppingPoint(ship, targetVelocity, shouldReverse);

	bool isFacing = (dp.Unit().Dot(angle.Unit()) > .95);
	if(!isClose || (!isFacing && !shouldReverse))
		command.SetTurn(AI::TurnToward(ship, dp));
	if(isFacing)
		command |= Command::FORWARD;
	else if(shouldReverse)
	{
		command.SetTurn(AI::TurnToward(ship, velocity));
		command |= Command::BACK;
	}

	return false;
}



bool Behavior::Stop(Ship &ship, Command &command, double maxSpeed, const Point direction)
{
	const Point &velocity = ship.Velocity();
	const Angle &angle = ship.Facing();

	double speed = velocity.Length();

	// If asked for a complete stop, the ship needs to be going much slower.
	if(speed <= (maxSpeed ? maxSpeed : .001))
		return true;
	if(!maxSpeed)
		command |= Command::STOP;

	// If you're moving slow enough that one frame of acceleration could bring
	// you to a stop, make sure you're pointed perfectly in the right direction.
	// This is a fudge factor for how straight you must be facing: it increases
	// from 0.8 when it will take many frames to stop, to nearly 1 when it will
	// take less than 1 frame to stop.
	double stopTime = speed / ship.Acceleration();
	double limit = .8 + .2 / (1. + stopTime * stopTime * stopTime * .001);

	// If you have a reverse thruster, figure out whether using it is faster
	// than turning around and using your main thruster.
	if(ship.Attributes().Get("reverse thrust"))
	{
		// Figure out your stopping time using your main engine:
		double degreesToTurn = TO_DEG * acos(min(1., max(-1., -velocity.Unit().Dot(angle.Unit()))));
		double forwardTime = degreesToTurn / ship.TurnRate();
		forwardTime += stopTime;

		// Figure out your reverse thruster stopping time:
		double reverseAcceleration = ship.Attributes().Get("reverse thrust") / ship.Mass();
		double reverseTime = (180. - degreesToTurn) / ship.TurnRate();
		reverseTime += speed / reverseAcceleration;

		// If you want to end up facing a specific direction, add the extra turning time.
		if(direction)
		{
			// Time to turn from facing backwards to target:
			double degreesFromBackwards = TO_DEG * acos(min(1., max(-1., direction.Unit().Dot(-velocity.Unit()))));
			double turnFromBackwardsTime = degreesFromBackwards / ship.TurnRate();
			forwardTime += turnFromBackwardsTime;

			// Time to turn from facing forwards to target:
			double degreesFromForward = TO_DEG * acos(min(1., max(-1., direction.Unit().Dot(angle.Unit()))));
			double turnFromForwardTime = degreesFromForward / ship.TurnRate();
			reverseTime += turnFromForwardTime;
		}

		if(reverseTime < forwardTime)
		{
			command.SetTurn(AI::TurnToward(ship, velocity));
			if(velocity.Unit().Dot(angle.Unit()) > limit)
				command |= Command::BACK;
			return false;
		}
	}

	command.SetTurn(AI::TurnBackward(ship));
	if(velocity.Unit().Dot(angle.Unit()) < -limit)
		command |= Command::FORWARD;

	return false;
}



void Behavior::PrepareForHyperspace(Ship &ship, Command &command)
{
	bool hasHyperdrive = ship.Attributes().Get("hyperdrive");
	double scramThreshold = ship.Attributes().Get("scram drive");
	bool hasJumpDrive = ship.Attributes().Get("jump drive");
	if(!hasHyperdrive && !hasJumpDrive)
		return;

	bool isJump = (ship.GetCheapestJumpType(ship.GetTargetSystem()).first == Ship::JumpType::JumpDrive);

	Point direction = ship.GetTargetSystem()->Position() - ship.GetSystem()->Position();
	if(!isJump && scramThreshold)
	{
		direction = direction.Unit();
		Point normal(-direction.Y(), direction.X());

		double deviation = ship.Velocity().Dot(normal);
		if(fabs(deviation) > scramThreshold)
		{
			// Need to maneuver; not ready to jump
			if((ship.Facing().Unit().Dot(normal) < 0) == (deviation < 0))
				// Thrusting from this angle is counterproductive
				direction = -deviation * normal;
			else
			{
				command |= Command::FORWARD;

				// How much correction will be applied to deviation by thrusting
				// as I turn back toward the jump direction.
				double turnRateRadians = ship.TurnRate() * TO_RAD;
				double cos = ship.Facing().Unit().Dot(direction);
				// integral(t*sin(r*x), angle/r, 0) = t/r * (1 - cos(angle)), so:
				double correctionWhileTurning = fabs(1 - cos) * ship.Acceleration() / turnRateRadians;
				// (Note that this will always underestimate because thrust happens before turn)

				if(fabs(deviation) - correctionWhileTurning > scramThreshold)
					// Want to thrust from an even sharper angle
					direction = -deviation * normal;
			}
		}
		command.SetTurn(AI::TurnToward(ship, direction));
	}
	// If we're a jump drive, just stop.
	else if(isJump)
		Stop(ship, command, ship.Attributes().Get("jump speed"));
	// Else stop in the fastest way to end facing in the right direction
	else if(Stop(ship, command, ship.Attributes().Get("jump speed"), direction))
		command.SetTurn(AI::TurnToward(ship, direction));
}



void Behavior::Deploy(const Ship &ship, bool includingDamaged)
{
	for(const Ship::Bay &bay : ship.Bays())
		if(bay.ship && (includingDamaged || bay.ship->Health() > .75) &&
				(!bay.ship->IsYours() || bay.ship->HasDeployOrder()))
			bay.ship->SetCommands(Command::DEPLOY);
}
