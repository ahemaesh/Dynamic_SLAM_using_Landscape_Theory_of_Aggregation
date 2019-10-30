#include <ceres/ceres.h>
#include <ceres/rotation.h>


// Define a struct to hold the distance error. This error specifies that the solution vector
// must be close to the control (initial) vector.
struct DistanceError{

	// Constructor
	DistanceError(double *P) : P_(P) {}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const Q, T* residuals) const {
		residuals[0] = T(2)*(Q[0] - T(P_[0]));
		residuals[1] = T(2)*(Q[1] - T(P_[1]));
		residuals[2] = T(2)*(Q[2] - T(P_[2]));
		return true;
	}

	// Control Mesh
	double *P_;

};


struct ScaleError{

	// Constructor
	ScaleError(double *cube, double *cube_ref) : cube_(cube), cube_ref_(cube_ref) {}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const scale_factor, T* residuals) const {
		residuals[0] = T(cube_[0]*scale_factor[0]) - T(cube_ref_[0]);
		residuals[1] = T(cube_[1]*scale_factor[0]) - T(cube_ref_[1]);
		residuals[2] = T(cube_[2]*scale_factor[0]) - T(cube_ref_[2]);
		return true;
	}

	// Control Mesh
	double *cube_;
	double *cube_ref_;

};

struct TransError{

	// Constructor
	TransError(double *cube, double *cube_ref) : cube_(cube), cube_ref_(cube_ref) {}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const trans, T* residuals) const {
		residuals[0] = T(cube_[0]) + trans[0] - T(cube_ref_[0]);
		residuals[1] = T(cube_[1]) + trans[1] - T(cube_ref_[1]);
		residuals[2] = T(cube_[2]) + trans[2] - T(cube_ref_[2]);
		return true;
	}

	// Control Mesh
	double *cube_;
	double *cube_ref_;

};

struct RotTransError{

	// Constructor
	RotTransError(double *cube, double *cube_ref) : cube_(cube), cube_ref_(cube_ref) {}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const rot, const T* const trans, T* residuals) const {

		T temp_point[3];
		temp_point[0]=T(cube_[0]);
		temp_point[1]=T(cube_[1]);
		temp_point[2]=T(cube_[2]);

		ceres::AngleAxisRotatePoint(rot,temp_point,temp_point);

		temp_point[0]+=trans[0];
		temp_point[1]+=trans[1];
		temp_point[2]+=trans[2];

		residuals[0] = temp_point[0] - T(cube_ref_[0]);
		residuals[1] = temp_point[1] - T(cube_ref_[1]);
		residuals[2] = temp_point[2] - T(cube_ref_[2]);
		return true;
	}

	// Control Mesh
	double *cube_;
	double *cube_ref_;

};

struct BAError{

	// Constructor
	BAError(double *K, double *projections, double weight) : K_(K), projections_(projections), weight_(weight) {}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const rot, const T* const trans, const T* const X, T* residuals) const {

		T temp_point[3];
		ceres::AngleAxisRotatePoint(rot,X,temp_point);

		temp_point[0]+=trans[0];
		temp_point[1]+=trans[1];
		temp_point[2]+=trans[2];

		temp_point[0]=K_[0]*temp_point[0]+K_[1]*temp_point[1]+K_[2]*temp_point[2];
		temp_point[1]=K_[3]*temp_point[0]+K_[4]*temp_point[1]+K_[5]*temp_point[2];
		temp_point[2]=K_[6]*temp_point[0]+K_[7]*temp_point[1]+K_[8]*temp_point[2];


		T projected_[2];
		projected_[0] = temp_point[0]/temp_point[2];
		projected_[1] = temp_point[1]/temp_point[2];

		residuals[0] = (projected_[0] - T(projections_[0]))*T(weight_);
		residuals[1] = (projected_[1] - T(projections_[1]))*T(weight_);
		return true;
	}

	// Control Mesh
	double *K_;
	double *projections_;
	double weight_;

};

struct ClassficationError{

	// Constructor
	ClassficationError(double pij, int indexi, int indexj) : pij(pij), indexi(indexi), indexj(indexj){}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const u,  T* residuals) const {
        residuals[0] = T(pij)*T((exp(u[indexi])/(exp(u[indexi])+T(1.0))) - (exp(u[indexj])/(exp(u[indexj])+T(1.0))))/T(2.0);
		return true;
	}

	double pij;
	int indexi;
	int indexj;
};


struct MagError{

	// Constructor
	MagError(double scale, int indexi) : scale(scale), indexi(indexi) {}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const u,  T* residuals) const {
		residuals[0] = T(T(scale)*u[indexi]);
		return true;
	}

	double scale;
	int indexi;
};


struct AtleastOneError{

    // Constructor
    AtleastOneError(double scale, int tsize) : scale(scale), tsize(tsize) {}

    // The operator method. Evaluates the cost function and computes the jacobians.
    template <typename T>
    bool operator() (const T* const u,  T* residuals) const {
        T sum = T(0.0000000);
        for (int i = 0; i < tsize; ++i)
        {
            sum += T((exp(u[i])/(exp(u[i])+T(1.0)))/2.0+0.5);
        }

        residuals[0] = T(T(scale) * T(1.0)/sum);

        return true;
    }

    double scale;
    int tsize;
};
