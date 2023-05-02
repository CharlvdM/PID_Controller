/// \file TE_PID_Controller.h
/// Author: Charl van de Merwe
/// Date: 12 October 2021

#include "PID_Controller.h"
#include <assert.h>

namespace {
    /// Limit the input to the lower and upper saturation values.
    [[nodiscard]] TE::float32_t saturate(const TE::float32_t aInput, const TE::float32_t aUpperLim, const TE::float32_t aLowerLim) {
        TE::float32_t limitedVal {aInput};
        if (aLowerLim > aUpperLim) {
            throw "The lower limit may not be larger than the upper limit";
        } else {
            if (aInput < aLowerLim) {
                limitedVal = aLowerLim;
            } else if (aInput > aUpperLim) {
                limitedVal = aUpperLim;
            }
        }
        return limitedVal;
    }
} // namespace

namespace CV {

    /// PI Constructor
    PID_Controller::PID_Controller(const PI_Params aPIparams, const TE::float32_t aT_Cntrl) noexcept
        : mKp(aPIparams.p), mKi(aPIparams.i), mT_Cntrl(aT_Cntrl), mIntegral(aPIparams.iInitialval),
          mIntegralInputPrev(aPIparams.iInitialval) {}

    /// PID Constructor
    PID_Controller::PID_Controller(const PID_Params aPIDparams, const TE::float32_t aT_Cntrl) noexcept
        : mKp(aPIDparams.p), mKi(aPIDparams.i), mKd(aPIDparams.d), mDerivativeFilterCoef(aPIDparams.dFilterCoef), mT_Cntrl(aT_Cntrl),
          mDerivativeTerm(true), mIntegral(aPIDparams.iInitialval), mIntegralInputPrev(aPIDparams.iInitialval) {}

    [[nodiscard]] TE::float32_t PID_Controller::calcError(const TE::float32_t aSetpoint, const TE::float32_t aFeedback) const noexcept {
        return aSetpoint - aFeedback;
    }

    [[nodiscard]] TE::float32_t PID_Controller::runIteration(const TE::float32_t aErr, const TE::float32_t aWindupErrCompensateVal) noexcept {
        const TE::float32_t proportional {mKp * aErr};

        const TE::float32_t integralInput {aErr - aWindupErrCompensateVal};
        mIntegral += mKi * (mT_Cntrl / 2.0F) * (integralInput + mIntegralInputPrev);
        mIntegralInputPrev = integralInput;

        TE::float32_t derivative {0.0F};
        if (mDerivativeTerm) {
            const TE::float32_t N = mDerivativeFilterCoef;
            const TE::float32_t N_times_Ts_div_2 {N * mT_Cntrl / 2.0F};
            derivative = (1.0F / (1.0F + N_times_Ts_div_2)) * ((1.0F - N_times_Ts_div_2) * mDerivativePrev + (mKd * N * (aErr - mErrPrev)));
            mDerivativePrev = derivative;
            mErrPrev = aErr;
        }

        return proportional + mIntegral + derivative;
    }

    [[nodiscard]] TE::float32_t PID_Controller::runIterFromSetpointNoAntiWindup(const TE::float32_t aSetpoint, const TE::float32_t aFeedback) noexcept {
        return runIteration(calcError(aSetpoint, aFeedback));
    }

    [[nodiscard]] TE::float32_t PID_Controller::runIterFromSetpoint(const TE::float32_t aSetpoint, const TE::float32_t aFeedback,
                                                                    const TE::float32_t aWindupErrCompensateVal) noexcept {
        return runIteration(calcError(aSetpoint, aFeedback), aWindupErrCompensateVal);
    }

    void PID_Controller::reset() noexcept {
        mIntegral = 0.0F;
        mIntegralInputPrev = 0.0F;
        mErrPrev = 0.0F;
        mDerivativePrev = 0.0F;
    }

    void PID_Controller::reset(TE::float32_t aIntegralSet) noexcept {
        mIntegral = aIntegralSet;
        mIntegralInputPrev = aIntegralSet;
        mErrPrev = 0.0F;
        mDerivativePrev = 0.0F;
    }

    /// PI controller with internal anti-windup constructor.
    PID_Ctrl_InternalAntiWindup::PID_Ctrl_InternalAntiWindup(const PI_Params aPIparams, const TE::float32_t aT_Cntrl,
                                                             const TE::float32_t aBottomLimit, const TE::float32_t aTopLimit,
                                                             const TE::float32_t aAntiWindupGain)
        : PID_Controller {aPIparams, aT_Cntrl}, mBottomLimit(aBottomLimit), mTopLimit(aTopLimit), mAntiWindupGain(aAntiWindupGain) {
        if (aBottomLimit > aTopLimit) {
            throw "PID: The anti-windup bottom limit is larger than the top limit.";
        }
    }

    /// PID controller with internal anti-windup constructor.
    PID_Ctrl_InternalAntiWindup::PID_Ctrl_InternalAntiWindup(const PID_Params aPIDparams, const TE::float32_t aT_Cntrl,
                                                             const TE::float32_t aBottomLimit, const TE::float32_t aTopLimit,
                                                             const TE::float32_t aAntiWindupGain)
        : PID_Controller {aPIDparams, aT_Cntrl}, mBottomLimit(aBottomLimit), mTopLimit(aTopLimit), mAntiWindupGain(aAntiWindupGain) {
        if (aBottomLimit > aTopLimit) {
            throw "PID: The anti-windup bottom limit is larger than the top limit.";
        }
    }

    [[nodiscard]] TE::float32_t PID_Ctrl_InternalAntiWindup::computeFromSetpoint(const TE::float32_t aSetpoint, const TE::float32_t aFeedback) {
        return compute(calcError(aSetpoint, aFeedback));
    }

    /// During each step the output is calculated and then limited. The difference between the output and the limited output is used to
    /// calculate the windup compensation value for the next step.
    [[nodiscard]] TE::float32_t PID_Ctrl_InternalAntiWindup::compute(const TE::float32_t aErr) {
        const TE::float32_t output {runIteration(aErr, mWindupErrCompensateVal)};
        const TE::float32_t outputLimited {saturate(output, mTopLimit, mBottomLimit)}; // throws exception if mBottomLimit > mTopLimit
        mWindupErrCompensateVal = mAntiWindupGain * (output - outputLimited);          // compensation value for the next step
        return outputLimited;
    }


    PI_Ctrl_Feedforward_ExternalAntiWindup::PI_Ctrl_Feedforward_ExternalAntiWindup(const PI_Params aPIparams, const TE::float32_t aT_Cntrl,
                                                                                   const TE::float32_t aAntiWindupGain) noexcept
        : PID_Controller(aPIparams, aT_Cntrl), mAntiWindupGain(aAntiWindupGain) {}

    [[nodiscard]] TE::float32_t PI_Ctrl_Feedforward_ExternalAntiWindup::compute(const TE::float32_t aErr, const TE::float32_t aOutputLimited,
                                                                                const TE::float32_t aFeedforward) {
        const TE::float32_t output {runIteration(aErr, mWindupErrCompensateVal) + aFeedforward};
        mWindupErrCompensateVal = mAntiWindupGain * (output - aOutputLimited); // compensation value for the next step
        return output;
    }

    PI_Ctrl_Feedforward_InternalAntiWindup::PI_Ctrl_Feedforward_InternalAntiWindup(const PI_Params aPIparams, const TE::float32_t aT_Cntrl,
                                                                                   const TE::float32_t aBottomLimit, const TE::float32_t aTopLimit,
                                                                                   const TE::float32_t aAntiWindupGain) noexcept
        : PID_Controller(aPIparams, aT_Cntrl), mBottomLimit(aBottomLimit), mTopLimit(aTopLimit), mAntiWindupGain(aAntiWindupGain) {
        if (aBottomLimit > aTopLimit) {
            throw "PID: The anti-windup bottom limit is larger than the top limit.";
        }
    }

    [[nodiscard]] TE::float32_t PI_Ctrl_Feedforward_InternalAntiWindup::compute(const TE::float32_t aErr, const TE::float32_t aFeedforward) {
        const TE::float32_t output {runIteration(aErr, mWindupErrCompensateVal) + aFeedforward};
        const TE::float32_t outputLimited {saturate(output, mTopLimit, mBottomLimit)}; // throws exception if mBottomLimit > mTopLimit
        mWindupErrCompensateVal = mAntiWindupGain * (output - outputLimited);          // compensation value for the next step
        return outputLimited;
    }

} // namespace CV
