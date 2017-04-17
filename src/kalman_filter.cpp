#include "kalman_filter.h"
#include <iostream>
#include "tools.h"


using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
   /*
    //Kalman Filter variables
    VectorXd x;	// object state
    MatrixXd P;	// object covariance matrix
    VectorXd u;	// external motion
    MatrixXd F; // state transition matrix
    MatrixXd H;	// measurement matrix; H is the matrix that projects your belief about the object's current state into the measurement space of the sensor. For lidar, this is a fancy way of saying that we discard velocity information from the state variable since the lidar sensor only measures position:   
    MatrixXd R;	// measurement covariance matrix; We’ll call the covariance of this uncertainty (i.e. of the sensor noise) Rk. The distribution has a mean equal to the reading we observed, which we’ll call zk→.
    MatrixXd I; // Identity matrix
    MatrixXd Q;	// process covariance matrix // We can model the uncertainty associated with the “world” (i.e. things we aren’t keeping track of) by adding some new uncertainty after every prediction step. Every state in our original estimate could have moved to a range of states. Because we like Gaussian blobs so much, we’ll say that each point in x̂ k−1 is moved to somewhere inside a Gaussian blob with covariance Qk. Another way to say this is that we are treating the untracked influences as noise with covariance Qk.
    
    z is the measurement vector. For a lidar sensor, the z vector contains the position−x and position−y measurements.
    
    
    */
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd z_pred = H_ * x_;

//    cout << "z_pred: " << z_pred << endl;
//    cout << "R_: " << R_ << endl;
//    cout << "R_laser_: " << R_laser_ << endl;
//    cout << "R_radar_: " << R_radar_ << endl;
//

    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    Tools tools;
//    cout << "R_: " << R_ << endl;

    
    //recover state parameters
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
    
//
//    //pre-compute a set of terms to avoid repeated calculation
    float h0 = sqrt(px*px+py*py);
    
    float h1 = atan2(py,px);
    if ((h0) < 0.0001){ h0=0.0001;}
    
    float h2 = ((px*vx+py*vy) / h0);
    cout << "h0" << h0 << endl;
    cout << "h1" << h1 << endl;
    cout << "h2" << h2 << endl;
    float pi = 3.14159265359;
    if (h1 < -pi){
        h1=h1+2*pi;
        cout << "added 2*pi to angle" << endl;

        
    }
    if (h1 > pi) {
        h1 = h1-2*pi;
        cout << "subtracted 2*pi from angle" << endl;

    }
//
    VectorXd z_pred = VectorXd(3);
//    z_pred(0) =sqrt(px*px+py*py);

    z_pred << h0, h1, h2;
//    cout << "z_pred_radar_all" << z_pred << endl;
//    cout << "z_radar" << z << endl;
//
////
//    cout << "z_pred_radar: " << z_pred << endl;
//
    MatrixXd Hj_ = tools.CalculateJacobian(x_);

    
    VectorXd y = z - z_pred;
    MatrixXd Hjt = Hj_.transpose();
    MatrixXd S = Hj_ * P_ * Hjt + R_;

    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Hjt;
    MatrixXd K = PHt * Si;
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * Hj_) * P_;
    

  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
