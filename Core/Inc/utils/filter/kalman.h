/*
 * kalman.h
 * 
 * Created : 11/13/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _KALMAN_H_
#define _KALMAN_H_

#include "mat.h"

class Kalman_Filter;

struct Kalman_Vars
{
public:
        void set_F(const Mat &f) { F_ = f; }
        void set_B(const Mat &b) { B_ = b; }
        void set_H(const Mat &h) { H_ = h; }
        void set_I(uint8_t n)    { I_ = Mat::eye(n); }
        void set_P(const Mat &p) { P_ = p; }
        void set_Q(const Mat &q) { Q_ = q; }
        void set_R(const Mat &r) { R_ = r; }

        friend class Kalman_Filter;

private:
        Mat F_, B_, H_, I_, P_, Q_, R_;
};

// Trying a Simple Sensor Fusion : Gyroscope and (Accelerometer/Magnetometer)
class Kalman_Filter
{
public:
        Kalman_Filter(Kalman_Vars *kv, int (*kv_init)(uint32_t)) :
        x_(2,1), u_(1,1), z_(1,1), K_(2,1)
        {
                is_first_ = true;
                kv_ = kv;
                kv_init_ = kv_init;
        }
        Kalman_Filter(Kalman_Filter &&) = default;
        Kalman_Filter(const Kalman_Filter &) = default;
        Kalman_Filter &operator=(Kalman_Filter &&) = default;
        Kalman_Filter &operator=(const Kalman_Filter &) = default;
        ~Kalman_Filter() { }
        
        float filter(float measured, float control_input, uint32_t dt_millis) {
                // float dt = (float)dt_millis / 1000.0;
                // For first element of vector
                if (is_first_) {
                        // Initiaize all the variables at the first call
                        kv_init_(dt_millis);

                        is_first_ = false;
                        x_.at(0,0) = measured;
                        // u_.at(0,0) = control_input;
                }
                else {
                        u_.at(0,0) = control_input;
                        z_.at(0,0) = measured;

                        // Predict
                        x_ = (kv_->F_)*(x_) + (kv_->B_)*(u_);
                        kv_->P_ = (kv_->F_)*(kv_->P_)*((kv_->F_).trans()) + kv_->Q_;

                        // Update
                        K_ = kv_->P_*(kv_->H_.trans())*(((kv_->H_*(kv_->P_)*(kv_->H_.trans())) + kv_->R_).inv());
                        x_ += K_*(z_ - (kv_->H_)*(x_));
                        kv_->P_ = (kv_->I_ - K_*(kv_->H_))*(kv_->P_);
                }

                return x_.at(0,0);
        }

        void clear() {
                is_first_ = true;
        }

private:
        int (*kv_init_)(uint32_t);
        bool is_first_;
        Kalman_Vars *kv_;

        Mat x_, u_, z_, K_;
};

#endif // !_KALMAN_H_
