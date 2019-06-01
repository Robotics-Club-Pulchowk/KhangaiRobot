/*
 * pid_algrithms.h
 * 
 * Created : 11/10/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _PID_ALGORITHMS_H_
#define _PID_ALGORITHMS_H_

#include <stdint.h>

class PID_Algorithm
{
public:
        PID_Algorithm() { set_PID(0,0,0); }
        PID_Algorithm(float p, float i, float d) { set_PID(p,i,d); }
        virtual float compute(float error, uint32_t dt_millis) = 0;
        virtual void clear() = 0;

        void set_P(float p) { p_ = p; }
        void set_I(float i) { i_ = i; }
        void set_D(float d) { d_ = d; }
        void set_PID(float p, float i, float d) {
                set_P(p);
                set_I(i);
                set_D(d);
        }
        float get_P() { return p_; }
        float get_I() { return i_; }
        float get_D() { return d_; }

        void set_Limits(float max_out, float min_out) {
                max_ = max_out;
                min_ = min_out;
        }

        float get_Upper() { return max_; }
        float get_Lower() { return min_; }

        virtual ~PID_Algorithm() { }

protected:
        float p_, i_, d_;
        float max_, min_;
};

class Discrete_PID : public PID_Algorithm
{
public:
        Discrete_PID(float p, float i, float d) :
        PID_Algorithm(p, i, d) {
                clear();
        }
        Discrete_PID() {
                clear();
        }
        Discrete_PID(Discrete_PID &&) = default;
        Discrete_PID(const Discrete_PID &) = default;
        Discrete_PID &operator=(Discrete_PID &&) = default;
        Discrete_PID &operator=(const Discrete_PID &) = default;
        ~Discrete_PID() { }

        /* *** PID Algorithm Description ***
         * 1) Discrete PID control Algorithm
         * 2) Integrator Method : Forward Euler
         * 3) //! Filtered Derivative not used
         * 4) Output Limited
         * 5) Form : Parallel
         * 6) Compensator Formula : Dz = P + I*Ts/(z-1) + D*(z-1)/(Ts*z)
         * 7) In Time Domain :
         *      y(t) - y(t-1) = a*x(t) + b*x(t-1) + c*x(t-2)
         *      where,
         *              a = P + D/Ts
         *              b = -P + I*Ts - 2*D/Ts
         *              c = D/Ts
         */
        float compute(float error, uint32_t dt_millis) {
                float Ts = (float)dt_millis / 1000.0;
                
                float P = get_P();
                float I = get_I();
                // We assume that Ts is never zero
                float D_by_Ts = get_D() / Ts;

                float a = P + D_by_Ts;
                float b = -P + I*Ts - 2*D_by_Ts;
                float c = D_by_Ts;

                l_output_ += a*error + b*l_err_ + c*ll_err_;

                if (l_output_ > get_Upper()) {
                        l_output_ = get_Upper();
                }
                else if (l_output_ < get_Lower()) {
                        l_output_ = get_Lower();
                }

                ll_err_ = l_err_;
                l_err_ = error;

                return l_output_;
        }

        void clear() {
                l_output_ = 0;
                l_err_ = 0;
                ll_err_ = 0;
        }

private:
        float l_output_;
        float l_err_;
        float ll_err_;
};

#endif // !_PID_ALGORITHMS_H_
