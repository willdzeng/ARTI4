#include <dad_local_planner/TrajectorySpace.h>

namespace dad_local_planner {

TrajectorySpace::TrajectorySpace() {
	// initialization
	vmin_ = -1;
	vmax_ = 1;
	kmin_ = -5;
	kmax_ = 5;
	vsize_ = 200;
	ksize_ = 200;
	// resize(vmin_, vmax_, kmin_, kmax_, vsize_, ksize_);
	initialize();
}

TrajectorySpace::~TrajectorySpace() {

}

void TrajectorySpace::initialize()
{
	if (vmin_ > vmax_ || kmin_ > kmax_ || vsize_ < 1 || ksize_ < 1) {
		printf("TrajectorySpace: wrong input please check, vmin:%f vmax:%f kmin:%f kmax:%f vsize:%d ksize:%d\n",
		       vmin_, vmax_, kmin_, kmax_, vsize_, ksize_);
		assert(0);
	}
	
	vsize_ = std::floor(vsize_ / 2) * 2 + 1;
	ksize_ = std::floor(ksize_ / 2) * 2 + 1;

	vresolution_ = (vmax_ - vmin_) / (vsize_ - 1);
	kresolution_ = (kmax_ - kmin_) / (ksize_ - 1);
	space_.resize(boost::extents[vsize_][ksize_]);
	std::fill( space_.origin(), space_.origin() + space_.size(), false);
	triangularSpace();
	printf("TrajectorySpace: Resized the space by vmin:%f vmax:%f kmin:%f kmax:%f vsize:%d ksize:%d\n",
		       vmin_, vmax_, kmin_, kmax_, vsize_, ksize_);
}

void TrajectorySpace::xyToij(double x, double y, int& i, int& j) const
{
	double v, k;
	v = x - vmin_;
	k = y - kmin_;
	// printf("v resolution is %f k res	olution is %f\n", vresolution_, kresolution_);
	i = std::round(v / vresolution_);
	j = std::round(k / kresolution_);

	if (i < 0) {
		i = 0;
		printf("i<0\n");
	}
	if ( j < 0 ) {
		j = 0;
		printf("j<0\n");
	}
	if ( i > vsize_ ) {
		i = vsize_;
		printf("i>vsize\n");
	}
	if ( j > ksize_ ) {
		j = ksize_;
		printf("i>ksize\n");
	}

}

void TrajectorySpace::ijToxy(int i, int j , double& x, double& y) const
{
	double v, k;
	v = i * vresolution_;
	k = j * kresolution_;
	x = v + vmin_;
	y = k + kmin_;
}

bool TrajectorySpace::isValid(double v, double k)
{
	int i, j;
	xyToij(v, k, i, j);
	return space_[i][j];
}

void TrajectorySpace::triangularSpace() {
	// double k1,b1,k2,b2,k3,b3,k4,b4;
	std::vector<double> k;
	std::vector<double> b;
	k.push_back( -kmax_ / vmax_ );
	k.push_back( -kmax_ / vmin_);
	k.push_back( -kmin_ / vmin_);
	k.push_back( -kmin_ / vmax_);
	b.push_back( kmax_ );
	b.push_back( kmax_ );
	b.push_back( kmin_ );
	b.push_back( kmin_ );
	for (int i = 0; i < k.size(); i++) {
		printf("k is %f  ", k[i]);
		printf("b is %f\n", b[i]);
	}

	for (int i = 0; i < vsize_; i ++) {
		for (int j = 0; j < ksize_; j ++) {
			double x, y;
			ijToxy(i, j, x, y);
			bool valid = true;
			for (int p = 0; p < k.size(); p++) {
				if (p < 2) {
					if ( y - k[p]*x - b[p] > 0.000001) {
						valid = false;
					}
				} else {
					if ( y - k[p]*x - b[p] < 0.000001) {
						valid = false;
					}
				}
			}
			space_[i][j] = valid;
			// printf("i: %d j: %d x: %f y: %f valid: %d\n",i,j,x,y,valid);
		}
	}
}

void TrajectorySpace::setCurvatureConstraint(const double kmin, const double kmax)
{
	kmax_ = kmax;
	kmin_ = kmin;
	printf("TrajectorySpace: Setting curvature constraint kmin: %f kmax: %f\n",kmin_,kmax_);
	// resize(vmin_, vmax_, kmin_, kmax_, vsize_, ksize_);
}

void TrajectorySpace::setVelocityConstraint(const double vmin, const double vmax)
{
	vmin_ = vmin;
	vmax_ = vmax;
	printf("TrajectorySpace: Setting velocity constraint vmin: %f vmax: %f\n",vmin_,vmax_);
	// resize(vmin_, vmax_, kmin_, kmax_, vsize_, ksize_);
}

void TrajectorySpace::setResolution(const double vsize, const double ksize)
{
	vsize_ = vsize;
	ksize_ = ksize;
	printf("TrajectorySpace: Setting resolution vsize: %d ksize: %d\n",vsize_,ksize_);
}

}// end of namespace
