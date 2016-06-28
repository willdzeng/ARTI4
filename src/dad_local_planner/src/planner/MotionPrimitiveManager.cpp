#include <dad_local_planner/MotionPrimitiveManager.h>

using namespace dad_local_planner;

MotionPrimitiveManager::MotionPrimitiveManager(const std::string fileName)
{
	dynamicsConstraints_ = std::shared_ptr<DynamicsConstraints> (new DynamicsConstraints());

	mpDatabasePtr_ = std::shared_ptr< std::vector<MotionPrimitive> >(new std::vector<MotionPrimitive>());

	loadMotionPrimitives(fileName);
	std::cout << " Loaded motion primitive database " << std::endl;

	setMaxDistanceOneMP();
	// needs to be setup as parameters
}

void MotionPrimitiveManager::getFeasibleMotionPrimitiveIndexSet(std::vector<int>& mpIndexSet ,
        const double currentVelocity,
        const double currentOmega)
{
	
	// for (int i = 0; i < mpSize_;i++){
	// 	mpIndexSet.push_back(i);
	// }
	
	mpIndexSet.clear();
	double amax = dynamicsConstraints_->maxForwardAcceleration;
	double amin = dynamicsConstraints_->maxBackwardAcceleration;
	double vmax = dynamicsConstraints_->maxForwardVelocity;
	double vmin = dynamicsConstraints_->maxBackwardVelocity;
	double vi = currentVelocity; // initial velocty
	int validCount = 0;

	for (int i = 0; i < mpSize_; i++) {
		double ds = mpDatabasePtr_->at(i).ds;
		double amin_t = amin;
		double amax_t = amax;
		double vfmin = 0; // minimum final velcity
		double vfmax = 0; // maximum final velcity
		double kMax = mpDatabasePtr_->at(i).kMax;


		// calculate forward maximum velocity
		double vfmax2 = 2 * amax * ds + vi * vi;
		double vfmin2 = 2 * amin * ds + vi * vi;

		if ( vfmin2 < 0 ) {
			// amin_t = -1 * vi * vi / ( 2 * ds );
			vfmin = 0;
		} else {
			vfmin = std::sqrt(vfmin2);
		}

		if ( vfmax2 < 0 ) {
			// amax_t = -1 * vi * vi / ( 2 * ds );
			vfmax = 0;
		} else {
			vfmax = std::sqrt(vfmax2);
		}

		if ( vfmin > vmax ) {
			printf("MP manager: Bad MP: Impossible to decelerate from a high velocity down to the allowed maximum forward velocity\n");
			goto not_valid;
		}

		if ( vfmax < vmin ) {
			printf("MP manager: Bad MP: Impossible to accelerate from a negative velocity up to the allowed maximum reverese velocity\n");
			goto not_valid;
		}

		if ( vfmin < vmin ) {vfmin = vmin;}
		if ( vfmax > vmax ) {vfmax = vmax;}

		// calculate if curvature matches
		if (trajSpace_.isValid(vfmax, kMax) == false ) {
			// printf("MP manager: bad mp: maximum velocity outside the trajectory space, vmax is %f, k is %f\n",vfmax,kMax);
			goto not_valid;
		}

		if (trajSpace_.isValid(vfmin, kMax) == false) {
			// printf("MP manager: bad mp: minmum velocity outside the trajectory space, vmax is %f, k is %f\n",vfmax,kMax);
			goto not_valid;
		}

		// the mp is valid
		mpIndexSet.push_back(i);
		mpInfoVector_[i].vc = vi;
		mpInfoVector_[i].vmin = vfmin;
		mpInfoVector_[i].vmax = vfmax;
		mpInfoVector_[i].valid = true;
		validCount++;


not_valid:
		mpInfoVector_[i].valid = false;

	}

	if( validCount == 0){
		printf("MP manager: no valid mp found\n");
	}
}

const MotionPrimitiveInfo MotionPrimitiveManager::getMotionPrimitiveInfo(const int index) const
{
	return mpInfoVector_[index];
}

const MotionPrimitive& MotionPrimitiveManager::getMotionPrimitive(const int index) const
{
	return (*mpDatabasePtr_)[index];
}


void MotionPrimitiveManager::setDurationOneMP(const double durationOneMP)
{
	DURATION_ONE_MP = durationOneMP;
	printf("MP Manager: Setting mp duration %f\n", DURATION_ONE_MP);
}

double MotionPrimitiveManager::getDurationOneMP() const
{
	return DURATION_ONE_MP;
}
double MotionPrimitiveManager::getMaxDistanceOneMP() const
{
	return MAX_DISTANCE_ONE_MP;
}


void MotionPrimitiveManager::setMaxDistanceOneMP()
{
	assert(mpDatabasePtr_->size() > 0);

	auto it = std::max_element( mpDatabasePtr_->begin(),
	                            mpDatabasePtr_->end(),
	                            [](const MotionPrimitive & lhs, const MotionPrimitive & rhs)
	{return std::abs(lhs.ds) < std::abs(rhs.ds);});

	MAX_DISTANCE_ONE_MP = std::abs(it->ds);

	std::cout << " MP Manager: MAX_DISTANCE_ONE_MP is " << MAX_DISTANCE_ONE_MP << std::endl;

}


void MotionPrimitiveManager::setDynamicsConstraints(std::shared_ptr<DynamicsConstraints> dynamicsConstraints)
{
	if ( dynamicsConstraints_->maxForwardAcceleration < -0.00001 ) {
		printf("MP Manager: can't set maximum forward acceleration less than zero");
		assert(0);
	}

	if ( dynamicsConstraints_->maxBackwardAcceleration > 0.00001) {
		printf("MP Manager: can't set maximum reverse acceleration greater than zero");
		assert(0);
	}

	if ( dynamicsConstraints_->maxForwardVelocity < -0.00001) {
		printf("MP Manager: can't set maximum forward velocity less than zero");
		assert(0);
	}

	if ( dynamicsConstraints_->maxBackwardVelocity > 0.00001) {
		printf("MP Manager: can't set maximum reverse velocity greater than zero");
		assert(0);
	}


	dynamicsConstraints_ = dynamicsConstraints;
	printf("MP Manager: Set Dynamic Constrains:\n");
	printf("                                   maxForwardAcceleration:   %f\n", dynamicsConstraints_->maxForwardAcceleration);
	printf("                                   maxForwardVelocity:       %f\n", dynamicsConstraints_->maxForwardVelocity);
	printf("                                   maxBackwardAcceleration:   %f\n", dynamicsConstraints_->maxBackwardAcceleration);
	printf("                                   maxBackwardVelocity:       %f\n", dynamicsConstraints_->maxBackwardVelocity);
	printf("                                   maxClockwiseAlpha:        %f\n", dynamicsConstraints_->maxClockwiseAlpha);
	printf("                                   maxClockwiseOmega:        %f\n", dynamicsConstraints_->maxClockwiseOmega);
	printf("                                   maxCounterClockwiseAlpha: %f\n", dynamicsConstraints_->maxCounterClockwiseAlpha);
	printf("                                   maxCounterClockwiseOmega: %f\n", dynamicsConstraints_->maxCounterClockwiseOmega);
	// set trajectory space constrain
	trajSpace_.setVelocityConstraint(dynamicsConstraints_->maxBackwardVelocity,dynamicsConstraints_->maxForwardVelocity);
}

// Shouldn't set up Curvature Constrain Directly, should have compute this by angular velocity stuff, this is a hack now;
void MotionPrimitiveManager::setCurvatureConstraint(const double kmin, const double kmax)
{
	printf("MP manager: get curvature constrain kmin: %f kmax: %f\n",kmin,kmax);
	trajSpace_.setCurvatureConstraint(kmin,kmax);
}

void MotionPrimitiveManager::setVelocityConstraint(const double vmin, const double vmax)
{
	printf("MP manager: get velocity constrain vmin: %f vmax: %f\n",vmin,vmax);
	trajSpace_.setVelocityConstraint(vmin,vmax);
}

void MotionPrimitiveManager::setTrajectorySpaceResolution(const int resolution)
{
	printf("MP manager: get resolution: %d\n",resolution);
	trajSpace_.setResolution(resolution,resolution);
}

void MotionPrimitiveManager::reinitialize()
{
	trajSpace_.initialize();
}

bool MotionPrimitiveManager::loadMotionPrimitives(const std::string fileName)
{
	std::ifstream f ( fileName.c_str() );

	if ( ! f.is_open() )
	{
		std::cout << " Error opening file : " << fileName.c_str() << std::endl;
		exit(-1);
	}
	int M;
	f >> M;
	for ( int n = 0 ; n < M ; n++ )
	{
		int mpid;
		double dx;
		double dy;
		double dth;
		double ds;
		double kMax;
		int N;
		f >> mpid;
		f >> dx;
		f >> dy;
		f >> dth;
		f >> ds;
		f >> kMax;
		f >> N;
		printf("MP id is %d n is %d\n", mpid, n);
		assert(mpid == n && "Order of MP is wrong. Do not rearrange it."); // implicit assumption that position in database and id are equal

		MotionPrimitive mp ( dx, dy, dth, ds, kMax, mpid );
		for ( int p = 0 ; p < N ; p++ )
		{
			double x;
			double y;
			double th;
			double s;
			double k;
			f >> x;
			f >> y;
			f >> th;
			f >> s;
			f >> k;
			mp.append ( x, y, th, s, k );
		}
		//std::cout << " ########## " << std::endl;
		//std::cout << mp << std::endl;
		//std::cout << " ********** " << std::endl;
		mpDatabasePtr_->push_back ( mp );
	}
	mpSize_ = mpDatabasePtr_->size();

	for (int i = 0; i < mpSize_ ; i++) {
		MotionPrimitiveInfo dump;
		mpInfoVector_.push_back(dump);
	}

	std::cout << " Motion primitives are loaded " << std::endl;
	return true;
}


// void MotionPrimitiveManager::get