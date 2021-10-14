#include "IBVS_orientation_control.h"

/*
Image based visual servo class for SnakeRaven Orientation control
author: Andrew Razjigaev 2021
- Get_Constants 				- clearly gets camera matrix etc for constants f, pu, pv etc.
- orientation_image_jacobian 	- Computes image jacobian orientation partition
- Compute_Control_law			- Given the pixel position and desired velocity vector, compute camera velocity
- Null_Control					- Outputs zero velocity and error when called
*/

IBVS_orientation_control::IBVS_orientation_control() {

	//Set Initial Default Constants based on data from camera.yaml
	//Determine pixel size in mm: sensor size (mm) / number of pixels
	pu = 0.9/384; //mm
	pv = 0.9/384; //mm

	//Focal length in pixels
	fx = 252.306368 * pu; //mm
	fy = 253.060935 * pv; //mm

	//Principal Point in pixels
	u0 = 207.301458;
	v0 = 205.655824;

	//Set velocity to be zero initially:
	Wc = MatrixXd::Zero(3,1);

	//Error is all zero:
	Error = MatrixXd::Zero(2 * Max_features, 1);

	//Set Lambda gain:
	lambda = 1;//2;
	Vmax = 0.05; //Max speed to velocity 0.0325

}

void IBVS_orientation_control::Get_Constants(const cv::Mat K, float height, float width, float sensor_height, float sensor_width){

	if (K.rows==0)
	{
		//Scenario when Camera info never came properly...
		//Use default Constants:
		cout << "USING DEFAULT CONSTANTS in function Get_Constants!" << endl;
	}
	else{
		//Proper process:

		//Determine pixel size in mm: sensor size (mm) / number of pixels
		pu = sensor_width/width;
		pv = sensor_height/height;

		//Camera Intrinsics matrix K structure:
		//[fx/pu   0   u0
		//   0  fy/pv  v0
		//   0     0    1]

		//Focal length in mm
		fx = K.at<double>(0,0)*pu;
		fy = K.at<double>(1,1)*pv; 
		//Principal Point in pixels
		u0 = K.at<double>(0,2);
		v0 = K.at<double>(1,2);

	}
}

void IBVS_orientation_control::Null_Control(){
	//Set to no velocity:
	Error = MatrixXd::Zero(2 * Max_features, 1);
	Wc = MatrixXd::Zero(3,1);
}

MatrixXd IBVS_orientation_control::orientation_image_jacobian(float x, float y){
	//Orientation Jacobian
	Matrix<double, 2, 3> Jw_xy;
	Jw_xy << x*y, -(1 + pow(x,2.0)), y,
			1 + pow(y,2.0), -x*y, -x;
	return Jw_xy;
}

void IBVS_orientation_control::Compute_Control_law(std::vector<double> posU, std::vector<double> posV, std::vector<double> deltaU, std::vector<double> deltaV){

	//Go through each feature error
	if (deltaU.empty())
	{
		cout << "No control Input required!" << endl;
		Null_Control();
	}
	else{
		//Ensure we are not tracking more than max number of features:
		curr_features = deltaU.size();
		if (curr_features > Max_features)
		{
			curr_features = Max_features;
		}

		//Set the error vector as zero:
		Error = MatrixXd::Zero(2 * Max_features, 1);

		//Go through each point feature and compute what is needed in the control law
		for (int i = 0; i < curr_features; i++)
		{
			//Solve normalised X and Y on image plane:
			x = (posU[i] - u0)*pu/fx; //mm
			y = (posV[i] - v0)*pv/fy; //mm

			//Solve x_dot y_dot velocity:
			x_dot = deltaU[i]*pu/fx;
			y_dot = deltaV[i]*pv/fy;

			//Put feature velocity into error vector:
			Error(2 * i)     = x_dot;
			Error(2 * i + 1) = y_dot;

			//Compute Image Jacobian for that point feature:
			Jw.block(2 * i, 0, 2, 3) = orientation_image_jacobian(x,y);
		}

		//Compute The output angular velocity for the current set of features:
		MatrixXd Ls = Jw.block(0, 0, 2 * curr_features, 3);
		MatrixXd deltaS = Error.block(0, 0, 2 * curr_features, 1);

		//Moore Penrose Pseudo Inverse
		//Wc = lambda * Ls^-1 deltaS
		//Wc = lambda * (Ls.transpose()*Ls).inverse()*(Ls.transpose()*deltaS);

		//More Sophisticated Eigen approach:
		Wc = lambda * Ls.completeOrthogonalDecomposition().solve(deltaS);

		//Restrict velocity magnitude:
		for (int i = 0; i < 3; i++)
		{
			if (abs(Wc(i))>Vmax)
			{
				Wc(i) = Vmax*(Wc(i)/abs(Wc(i)));
			}
		}
	}

}
