#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = false;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);



    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2//TODO manipulate it for stability
    std_a_ = 30;

    // Process noise standard deviation yaw acceleration in rad/s^2  TODO manipulate it for stability
    std_yawdd_ = 30;

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;






  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.


  Hint: one or more values initialized above might be wildly off...
  */
    is_initialized_ = false;
    //state dimension
    n_x_ = 5;
    //augmentation dimension
    n_aug_ = 7;

    lambda_ = 3 - n_aug_;

    time_us_ = 0;
    //predicted sigma points
    double NIS_laser,NIS_radar_ = 0.0;

    //initialize sigma point prediction matrix
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    Xsig_pred_ <<
              5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
            0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
    P_ << 1,0,0,0,0,
          0,1,0,0,0,
           0, 0,1,0,0,
            0,0,0,1,0,
            0,0,0,0,1;

    //set weights
    //set weights
    weights_ = VectorXd(2*n_aug_+1);

    double weigth_0 = lambda_/(lambda_ +n_aug_);
    weights_(0) = weigth_0;
    for(int i = 1; i < n_aug_ + lambda_; i ++){
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_[i] = weight;
    }


}
UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */


  if(!is_initialized_){

      //initialize x and P

            //IF RADAR initialize to radar
         if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
             /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
             cout << "RADAR Data: " << endl;

             double rho = meas_package.raw_measurements_[0]; // range
             double phi = meas_package.raw_measurements_[1]; // bearing
             double rho_dot = meas_package.raw_measurements_[2]; // velocity of rho

             // Coordinates convertion from polar to cartesian
             double x = rho * cos(phi);
             double y = rho * sin(phi);
             double vx = rho_dot * cos(phi);
             double vy = rho_dot * sin(phi);
             x_ << x, y, 0,
                     0,
                     0;
             cout << "END RADAR Data: " << endl;
         }
            //ELSE LIDAR
                 cout << "LASER Data: " << endl;

            x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1],2.2049,
                    0.5015,
                    0.3528;


                    cout << "END LASER Data: "<<x_ << endl;
      time_us_ = meas_package.timestamp_;
      cout << "TIMESTAMP: "<< time_us_ << endl;


      is_initialized_ = true;
      return;


  }



    float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    Prediction(dt);

    if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
        //do radar update
        UpdateRadar(meas_package);
    }else{
        //dolaser update
    }





}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  cout << "Predicting...." << endl;
  //generate sigma points
    MatrixXd A = P_.llt().matrixL();


    //create augmented mean vector
    VectorXd x_aug = VectorXd(7);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);

    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    //create mean state sugmented vector
    x_aug.head(5) = x_;
    x_aug(5) =0;
    x_aug(6) = 0;

    //create covariance matrix P
    P_.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_ * std_a_;
    P_aug(6,6) = std_yawdd_ * std_yawdd_;

    //create square root MTX
    MatrixXd L = P_aug.llt().matrixL();

    //augmented sigma points
    Xsig_aug.col(0) = x_aug;
    for (int i = 0;i<n_aug_; i++){
        Xsig_aug.col(i+1) = x_aug + sqrt(lambda_ +n_aug_) * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ +n_aug_) * L.col(i);

    }

    //predict sigma points
    for(int i = 0 ; i < 2*n_aug_+1; i++){
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double  nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);


        //predict state values
        double px_p,py_p;

        //avoid devision by zero
        if(fabs(yawd) > 0.001){
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );

        }else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;

        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;

        //write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;


        //mean and covariance estimation


        x_.fill(0.0);

        //predicted state mean
        for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
            x_ = x_+ weights_(i) * Xsig_pred_.col(i);
        }
        P_.fill(0.0);
        //predicted covariance matrix
        for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

            // state difference
            VectorXd x_diff = Xsig_pred_.col(i) - x_;
            //angle normalization
            while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
            while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

            P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
        }


        cout << "P" << P_ << endl;
        cout << "x_" << x_ << endl;

    }


  //


}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
//
//    int n_z_ = 2;
//    //create example vector for incoming lidar measurement
//    VectorXd z = VectorXd(n_z_);
//    z << meas_package(0), meas_package(1);
//
//    //kalman gain
//    // matrix for cross correlation
//    MatrixXd Tc = MatrixXd(n_x_, n_z_);
//    Tc.fill(0.0);
//
//    MatrixXd K = MatrixXd(n_x_, n_z_);
//    K.fill(0.0);
//
//    MatrixXd S = MatrixXd(n_x_, n_z_);
//    S.fill(0.0);
//
//    K = Tc * S.inverse();



}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
    int n_z_ = 3;
    //create example vector for incoming radar measurement
    VectorXd z = VectorXd(n_z_);
    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);



    //PREDICT MEASUREMENT
    for(int i = 0; i< 2 * n_aug_+1; i++){
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);

        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        // measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
        Zsig(1,i) = atan2(p_y,p_x);                                 //phi
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }
        //predicted measurement mean
        VectorXd z_pred = VectorXd(n_z_);
        z_pred.fill(0.0);
        for (int i=0; i < 2*n_aug_+1; i++) {
            z_pred = z_pred + weights_(i) * Zsig.col(i);
        }

        //measurement covariance matrix S
        MatrixXd S = MatrixXd(n_z_,n_z_);
        S.fill(0.0);
        for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
            //residual
            VectorXd z_diff = Zsig.col(i) - z_pred;

            //angle normalization
            while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
            while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

            S = S + weights_(i) * z_diff * z_diff.transpose();
        }
        //add measurement noise covariance matrix
        MatrixXd R = MatrixXd(n_z_,n_z_);
        R <<    std_radr_*std_radr_, 0, 0,
                0, std_radphi_*std_radphi_, 0,
                0, 0,std_radrd_*std_radrd_;
        S = S + R;



    //UPDATE STATE

    // matrix for cross correlation
    MatrixXd Tc = MatrixXd(n_x_, n_z_);
    Tc.fill(0.0);

    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    double px = x_[0];
    double py = x_[1];
    double vx = x_[2];
    double vy = x_[3];
    z = meas_package.raw_measurements_;
    //residual
    VectorXd z_diff = z - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();

}
