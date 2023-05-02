#ifndef TE_PID_CONTROLLER_H_
#define TE_PID_CONTROLLER_H_

/// \file TE_PID_Controller.h
/// Author: Charl van de Merwe
/// Date: 12 October 2021

#include "TE_Float.h"
#include <cstdint>

namespace CV {

    /// Struct used to specify a PI controller's parameters
    struct PI_Params {
        TE::float32_t p;                  ///< Proportional gain
        TE::float32_t i;                  ///< Integral gain
        TE::float32_t iInitialval = 0.0F; ///< Integral initial value
    };

    /// Struct used to specify a PID controller's parameters
    struct PID_Params {
        TE::float32_t p;           ///< Proportional gain
        TE::float32_t i;           ///< Integral gain
        TE::float32_t iInitialval; ///< Integral initial value
        TE::float32_t d;           ///< Derivative gain
        TE::float32_t dFilterCoef; ///< derivative filter coefficient
    };

    /// This is a parallel PID controller implemented using the trapezoidal (bilinear) method. The gains specified are the continuous time
    /// gains. Anti-windup is implemented by passing in the compensation value, see the execute function comments.
    /// See https://www.scilab.org/discrete-time-pid-controller-implementation for an overview of the discrete algorithm used.
    class PID_Controller {
    public:
        /// PI controller constructor
        /// \param aPIparams    PI controller parameters
        /// \param aT_Cntrl     The control loop period in seconds
        PID_Controller(const PI_Params aPIparams, const TE::float32_t aT_Cntrl) noexcept;

        /// PID controller constructor
        /// \param aPIDparams   PID controller parameters
        /// \param aT_Cntrl     The control loop period in seconds
        PID_Controller(const PID_Params aPIDparams, const TE::float32_t aT_Cntrl) noexcept;

        /// Run the control loop iteration of the PI or PID controller
        /// \param aErr                     The input to the PID controller, the error signal
        /// \param aWindupErrCompensateVal  This value is subtracted from the error before being passed to the integral component. This
        ///                                 value needs to be calculated from an anti-windup controller. The anti-windup controller should
        ///                                 be of the form: aWindupErrCompensateVal = Ka * (Sig - SigLim), where Ka is the gain, Sig is the
        ///                                 signal before it is limited and SigLim is the limited signal. Set to 0.0F for no compensation.
        [[nodiscard]] TE::float32_t runIteration(const TE::float32_t aErr, const TE::float32_t aWindupErrCompensateVal = 0.0F) noexcept;

        /// Run the control loop iteration of the PI or PID controller, with the setpoint and feedback arguments from which the error is
        /// calculated.
        /// \param aSetpoint                The setpoint (desired plant output)
        /// \param aFeedback                The feedback from the plant output
        /// \param aWindupErrCompensateVal  This value is subtracted from the error before being passed to the integral component. This
        ///                                 value needs to be calculated from an anti-windup controller. The anti-windup controller should
        ///                                 be of the form: aWindupErrCompensateVal = Ka * (Sig - SigLim), where Ka is the gain, Sig is the
        ///                                 signal before it is limited and SigLim is the limited signal. Set to 0.0F for no compensation.
        [[nodiscard]] TE::float32_t runIterFromSetpoint(const TE::float32_t aSetpoint, const TE::float32_t aFeedback,
                                                        const TE::float32_t aWindupErrCompensateVal = 0.0F) noexcept;

        /// Reset the integral and derivative terms to zero. Reset the derivate terms to zero.
        void reset() noexcept;

        /// Reset the integral terms to the specified value. Reset the derivate terms to zero.
        /// \param aIntegralSet     Reset value of the integral term
        void reset(TE::float32_t aIntegralSet) noexcept;

        /// Calculate the error between the setpoint and the feedback.
        [[nodiscard]] TE::float32_t calcError(const TE::float32_t aSetpoint, const TE::float32_t aFeedback) const noexcept;

    private:
        const TE::float32_t mKp;                          ///< Proportional gain
        const TE::float32_t mKi;                          ///< Integral gain
        const TE::float32_t mKd {0.0F};                   ///< Derivative gain
        const TE::float32_t mDerivativeFilterCoef {0.0F}; ///< N - the derivative filter coefficient
        const TE::float32_t mT_Cntrl;                     ///< Control loop execution period
        const bool mDerivativeTerm {false};               ///< true if there is a derivative term. Otherwise this is a PI controller
        TE::float32_t mIntegral;                          ///< Integral term
        TE::float32_t mIntegralInputPrev;                 ///< previous input to the integral control. The integral input is the error
                                                          ///< minus the anti-windup compensation value
        TE::float32_t mErrPrev {0.0F};                    ///< Previous error
        TE::float32_t mDerivativePrev {0.0F};             ///< Previous derivative term
    };

    /// This class inherits from the PID_Controller class and includes an internal anti-windup controller. This class assumes that the
    /// output of this controller is limited directly. This is not always the case, sometimes the output of this controller plus some other
    /// term is limited, in which case this class shouldn't be used.
    class PID_Ctrl_InternalAntiWindup : public PID_Controller {
    public:
        /// PI controller with internal anti-windup constructor.
        /// \param aPIparams        PI controller parameters
        /// \param aT_Cntrl         The control loop period in seconds
        /// \param aBottomLimit     The bottom limit of the output of the controller
        /// \param aTopLimit        The top limit of the output of the controller
        /// \param aAntiWindupGain  This is the gain that is multiplied by the difference between the output and the limited output of this
        ///                         controller
        PID_Ctrl_InternalAntiWindup(const PI_Params aPIparams, const TE::float32_t aT_Cntrl, const TE::float32_t aBottomLimit,
                                    const TE::float32_t aTopLimit, const TE::float32_t aAntiWindupGain);

        /// PID controller with internal anti-windup constructor.
        /// \param aPIDparams       PID controller parameters
        /// \param aT_Cntrl         The control loop period in seconds
        /// \param aBottomLimit     The bottom limit of the output of the controller
        /// \param aTopLimit        The top limit of the output of the controller
        /// \param aAntiWindupGain  This is the gain that is multiplied by the difference between the output and the limited output of this
        ///                         controller
        PID_Ctrl_InternalAntiWindup(const PID_Params aPIDparams, const TE::float32_t aT_Cntrl, const TE::float32_t aBottomLimit,
                                    const TE::float32_t aTopLimit, const TE::float32_t aAntiWindupGain);

        /// Execute the PI or PID controller with anti-windup
        [[nodiscard]] TE::float32_t compute(const TE::float32_t aErr);

        /// Execute the PI or PID controller with anti-windup, from the setpoint and feedback arguments from which the error is calculated.
        [[nodiscard]] TE::float32_t computeFromSetpoint(const TE::float32_t aSetpoint, const TE::float32_t aFeedback);

    private:
        const TE::float32_t mBottomLimit;             ///< The bottom limit of the output of the controller
        const TE::float32_t mTopLimit;                ///< The top limit of the output of the controller
        const TE::float32_t mAntiWindupGain;          ///< This is the gain that is multiplied by the difference between the output and the
                                                      ///< limited output of this controller
        TE::float32_t mWindupErrCompensateVal {0.0F}; ///< This value is subtracted from the error before being passed to the integral
                                                      ///< component
    };

} // namespace CV

#endif //TE_PID_CONTROLLER_H_
