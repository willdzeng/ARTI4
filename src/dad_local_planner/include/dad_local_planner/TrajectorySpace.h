#include <cmath>
#include <boost/multi_array.hpp>
#include <fstream>


namespace dad_local_planner {

class TrajectorySpace {
public:

	TrajectorySpace();

	~TrajectorySpace();

	void initialize();

	void xyToij(double x, double y, int& i, int& j) const;

	void ijToxy(int i, int j , double& x, double& y) const;

	bool isValid(double v, double k);

	void triangularSpace();

	void setCurvatureConstraint(const double kmin, const double kmax);
	
	void setVelocityConstraint(const double vmin, const double vmax);

	void setResolution(const double vsize, const double ksize);

private:
	boost::multi_array<bool, 2> space_;
	double vmin_;
	double vmax_;
	double kmin_;
	double kmax_;
	double vresolution_;
	double kresolution_;
	int vsize_;
	int ksize_;


};// end of trajectory class

}