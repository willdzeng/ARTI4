#include <dad_local_planner/DynamicsConstraints.h>

DynamicsConstraints::DynamicsConstraints()
{
	maxForwardAcceleration = 1;
	maxForwardVelocity = 1;

	maxBackwardAcceleration = -2;
	maxBackwardVelocity = -0.5;

	maxClockwiseAlpha = 1;
	maxClockwiseOmega = 1;

	maxCounterClockwiseAlpha = 1;
	maxCounterClockwiseOmega = 1;

}
DynamicsConstraints::~DynamicsConstraints()
{

}