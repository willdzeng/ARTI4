#pragma once

class DynamicsConstraints
{
public:
	double maxForwardAcceleration;
	double maxForwardVelocity;

	double maxBackwardAcceleration;
	double maxBackwardVelocity;

	double maxClockwiseAlpha;
	double maxClockwiseOmega;

	double maxCounterClockwiseAlpha;
	double maxCounterClockwiseOmega;

	DynamicsConstraints();

	~DynamicsConstraints();



	/* data */
};