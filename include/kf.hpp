#ifndef KF_H
#define KF_H

#include <eigen3/Eigen/Dense>

using namespace Eigen;

/* 
 * KalmanFilter class is naive (it could be improvised) implementation of Kalman Filter equations.
 * There are several resources online which are good in order to get a better understanding of what is a Kalman Filter, below ones are few:
 * 1. http://www.cs.unc.edu/~tracker/media/pdf/SIGGRAPH2001_CoursePack_08.pdf
 * 2. https://towardsdatascience.com/kalman-filter-an-algorithm-for-making-sense-from-the-insights-of-various-sensors-fused-together-ddf67597f35e
 * 3. http://bilgin.esme.org/BitsAndBytes/KalmanFilterforDummies
 * 4. https://www.youtube.com/watch?v=CaCcOwJPytQ&list=PLX2gX-ftPVXU3oUFNATxGXY90AULiqnWT
 * 5. https://www.youtube.com/watch?v=TEKPcyBwEH8&t=386s
 * 6. https://www.youtube.com/watch?v=mwn8xhgNpFY&list=PLn8PRpmsu08pzi6EMiYnR-076Mh-q3tWr
 */
class KalmanFilter
{
    using Vector = Eigen::Matrix<double, 2, 1>; //Shape 2x1 for holding state vector
    using Matrix = Eigen::Matrix<double, 2, 2>; //Shape 2x2 for holding process covariance matrix

    private:
        Vector m_mean;
        Matrix m_covariance;
        double m_acc_var;

    public:
        KalmanFilter(){}
        KalmanFilter(double init_X, double init_V, double accelVariance)
        {
            m_mean(0) = init_X;
            m_mean(1) = init_V;
            m_acc_var = accelVariance;
            m_covariance.setIdentity();
        }
        
        //This can be utilized in case of solving complex KalmanFilter scenario
        void init(double init_X, double init_V, double accelVariance)
        {
            m_mean(0) = init_X;
            m_mean(1) = init_V;
            m_acc_var = accelVariance;
            m_covariance.setIdentity();
        }

        void predict(double dt)
        {
            /* Implementing below equations
             *  x = F x
             *  P = F P Ftrans + G Gtrans a
            */
            
            Matrix F;
            F.setIdentity();

            F(0,1) = dt;

            const Vector newX = F * m_mean;

            Vector G;
            G(0) = 0.5 * dt * dt;
            G(1) = dt;

            const Matrix newP = F * m_covariance * F.transpose() + G * G.transpose() * m_acc_var;
            // std::cout<<"predicted pos"<< newX[0]<<std::endl;
            m_covariance = newP;
            m_mean = newX;
        }

        void update(double measured_val, double measured_variance)
        {
            /* 
             *   y = z - H x
             *   S = H P Ht + R
             *   K = P Ht S^-1
             *   x = x + K y
             *   P = (I - K H) * P 
            */

            Eigen::Matrix<double, 1, 2> H;
            H.setZero();

            H(0, 0) = 1;

            double y = measured_val - H * m_mean;
            // std::cout << "y:: " << y << std::endl;
            double S = H * m_covariance * H.transpose() + measured_variance;
            // std::cout<<"S:: " << S << std::endl;
            const Vector K = m_covariance * H.transpose() * 1.0 / S;
            // std::cout <<"K:: "<<K << std::endl;
            Vector newX = m_mean + K*y;
            Matrix newP = (Matrix::Identity() - K * H) * m_covariance;

            // std::cout<<"updated pos"<< newX[0]<<std::endl;

            m_mean = newX;
            m_covariance = newP;
        }

        Matrix cov() const
        {
            return m_covariance;
        }

        Vector mean() const
        {
            return m_mean;
        }

        double pos() const
        {
            return m_mean(0);
        }

        double vel() const
        {
            return m_mean(1);
        }

        /*  Simpler 1-d equations   
        void init(double initX)
        {
            posx = initX;
            cov = 1;
        }

        void predict(double a_in, double q)
        {
            double a = a_in;
            double v = 1.2;
            double newX = posx + a*posx;
            double new_cov = a * cov * a + q;

            posx = newX;
            cov = new_cov;
        }

        void update(double z, double r)
        {
            double H = 1.0; //for time being
            double y = z - H * posx;
            
            double kg = cov*H / (H*cov*H + r);
            double newX = z + kg * y;
            double new_cov = (1 - kg * H) * cov;

            posx = newX;
            cov = new_cov;
        }

        double pos() const
        {
            return posx;
        } */
};

#endif