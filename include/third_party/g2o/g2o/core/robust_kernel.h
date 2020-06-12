//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_ROBUST_KERNEL_H
#define ORB_SLAM2_KINETIC_ROBUST_KERNEL_H

#ifdef _MSC_VER
#include <memory>
#else
#include <tr1/memory>
#endif
#include <Eigen/Core>


namespace g2o {

    /**
     * \brief base for all robust cost functions
     *
     * Note in all the implementations for the other cost functions the e in the
     * funtions corresponds to the sqaured errors, i.e., the robustification
     * functions gets passed the squared error.
     *
     * e.g. the robustified least squares function is
     *
     * chi^2 = sum_{e} rho( e^T Omega e )
     */
    class  RobustKernel
    {
    public:
        RobustKernel();
        explicit RobustKernel(double delta);
        virtual ~RobustKernel() {}
        /**
         * compute the scaling factor for a error:
         * The error is e^T Omega e
         * The output rho is
         * rho[0]: The actual scaled error value
         * rho[1]: First derivative of the scaling function
         * rho[2]: Second derivative of the scaling function
         */
        virtual void robustify(double squaredError, Eigen::Vector3d& rho) const = 0;

        /**
         * set the window size of the error. A squared error above delta^2 is considered
         * as outlier in the data.
         */
        virtual void setDelta(double delta);
        double delta() const { return _delta;}

    protected:
        double _delta;
    };
    typedef std::tr1::shared_ptr<RobustKernel> RobustKernelPtr;

} // end namespace g2o

#endif //ORB_SLAM2_KINETIC_ROBUST_KERNEL_H
