//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_ROBUST_KERNEL_IMPL_H
#define ORB_SLAM2_KINETIC_ROBUST_KERNEL_IMPL_H

#include "robust_kernel.h"

namespace g2o {

    /**
     * \brief scale a robust kernel to another delta (window size)
     *
     * Scales a robust kernel to another window size. Useful, in case if
     * one implements a kernel which only is designed for a fixed window
     * size.
     */
    class  RobustKernelScaleDelta : public RobustKernel
    {
    public:
        /**
         * construct the scaled kernel ontop of another kernel which might be shared accross
         * several scaled kernels
         */
        explicit RobustKernelScaleDelta(const RobustKernelPtr& kernel, double delta = 1.);
        explicit RobustKernelScaleDelta(double delta = 1.);

        //! return the underlying kernel
        const RobustKernelPtr kernel() const { return _kernel;}
        //! use another kernel for the underlying operation
        void setKernel(const RobustKernelPtr& ptr);

        void robustify(double error, Eigen::Vector3d& rho) const;

    protected:
        RobustKernelPtr _kernel;
    };

    /**
     * \brief Huber Cost Function
     *
     * Loss function as described by Huber
     * See http://en.wikipedia.org/wiki/Huber_loss_function
     *
     * If e^(1/2) < d
     * rho(e) = e
     *
     * else
     *
     *               1/2    2
     * rho(e) = 2 d e    - d
     */
    class  RobustKernelHuber : public RobustKernel
    {
    public:
        virtual void setDelta(double delta);
        virtual void setDeltaSqr(const double &delta, const double &deltaSqr);
        virtual void robustify(double e2, Eigen::Vector3d& rho) const;

    private:
        float dsqr;
    };

    /**
    * \brief Tukey Cost Function
    *
    *
    * If e^(1/2) < d
    * rho(e) = delta2(1-(1-e/delta2)^3)
    *
    * else
    *
    * rho(e) = delta2
    */
    class  RobustKernelTukey : public RobustKernel
    {
    public:

        virtual void setDeltaSqr(const double &deltaSqr, const double &inv);
        virtual void robustify(double e2, Eigen::Vector3d& rho) const;
    private:
        float _deltaSqr;
        float _invDeltaSqr;
    };


    /**
     * \brief Pseudo Huber Cost Function
     *
     * The smooth pseudo huber cost function:
     * See http://en.wikipedia.org/wiki/Huber_loss_function
     *
     *    2       e
     * 2 d  (sqrt(-- + 1) - 1)
     *             2
     *            d
     */
    class  RobustKernelPseudoHuber : public RobustKernel
    {
    public:
        virtual void robustify(double e2, Eigen::Vector3d& rho) const;
    };

    /**
     * \brief Cauchy cost function
     *
     *  2     e
     * d  log(-- + 1)
     *         2
     *        d
     */
    class  RobustKernelCauchy : public RobustKernel
    {
    public:
        virtual void robustify(double e2, Eigen::Vector3d& rho) const;
    };

    /**
     * \brief Saturated cost function.
     *
     * The error is at most delta^2
     */
    class  RobustKernelSaturated : public RobustKernel
    {
    public:
        virtual void robustify(double e2, Eigen::Vector3d& rho) const;
    };

    /**
     * \brief Dynamic covariance scaling - DCS
     *
     * See paper Robust Map Optimization from Agarwal et al.  ICRA 2013
     *
     * delta is used as $phi$
     */
    class  RobustKernelDCS : public RobustKernel
    {
    public:
        virtual void robustify(double e2, Eigen::Vector3d& rho) const;
    };

} // end namespace g2o

#endif //ORB_SLAM2_KINETIC_ROBUST_KERNEL_IMPL_H
