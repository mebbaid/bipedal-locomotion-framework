/**
 * @file ApproxExactIntegrator.h
 * @authors Mohamed Elobaid
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef APPROXEXACTINTEGRATOR_H
#define APPROXEXACTINTEGRATOR_H

#include <chrono>
#include <tuple>
#include <type_traits>

#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotion/ContinuousDynamicalSystem/FixedStepIntegrator.h>
#include <BipedalLocomotion/GenericContainer/NamedTuple.h>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{
template <typename _DynamicalSystem> class ApproxExactIntegrator;
}
} // namespace BipedalLocomotion

BLF_DEFINE_INTEGRATOR_STRUCTURE(ApproxExactIntegrator, _DynamicalSystemType)

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

/**
 * ApproxExactIntegrator implements the sampled-data approximate model.
 * It implements the sampled-data model of a continous-time system as defined
 * in the paper "An Introduction to motion planning under digital control" by S. Monaco and
 * D.Normand-Cyrot
 * @tparam _DynamicalSystem a class derived from DynamicalSystem
 * @warning We assume that the operator + is defined for the objects contained in the
 * DynamicalSystem::State and DynamicalSystem::StateDerivative.
 */
template <class _DynamicalSystem>
class ApproxExactIntegrator : public FixedStepIntegrator<ApproxExactIntegrator<_DynamicalSystem>>
{
public:
    using DynamicalSystem =
        typename internal::traits<ApproxExactIntegrator<_DynamicalSystem>>::DynamicalSystem;
    using State = typename internal::traits<ApproxExactIntegrator<_DynamicalSystem>>::State;
    using StateDerivative =
        typename internal::traits<ApproxExactIntegrator<_DynamicalSystem>>::StateDerivative;
    using HigherDerivatives = std::vector<StateDerivative>; // higher order derivatives up to
                                                            // integration order
private:
    /** Temporary buffer usefully to avoid continuous memory allocation */
    StateDerivative m_computationalBufferStateDerivative;

    /** Temporary buffer usefully to avoid continuous memory allocation */
    HigherDerivatives m_computationalBufferHigherStateDerivative;

    /** Temporary buffer usefully to avoid continuous memory allocation */
    State m_computationalBufferState;

    template <std::size_t I = 0>
    inline typename std::enable_if<I == std::tuple_size<State>::value, void>::type
    addArea(const StateDerivative& dx, const HigherDerivatives& d_ix,
            const std::chrono::nanoseconds& dT, State& x)
    {
        static_assert(std::tuple_size<State>::value == std::tuple_size<StateDerivative>::value);
        //TODO ass assertion over higher derivatives dimension
    }

    template <std::size_t I = 0>
    inline typename std::enable_if<(I < std::tuple_size<State>::value), void>::type
    addArea(const StateDerivative& dx,
            const HigherDerivatives& d_ix,
            const std::chrono::nanoseconds& dT,
            State& x)
    {
        static_assert(std::tuple_size<State>::value == std::tuple_size<StateDerivative>::value);
        using std::get;

        // convert the dT in seconds and perform the integration
        get<I>(x) = (get<I>(dx) * std::chrono::duration<double>(dT).count()) + get<I>(x);
        for (int i = 0; i < d_ix.size(); i++)
        {
            int factorial = (d_ix.size() <= 1) ? 1 : d_ix.size() * (d_ix.size() - 1);
            get<I>(x)
                = (get<I>(d_ix[i])
                   * std::pow(std::chrono::duration<double>(dT).count(), static_cast<double>(i)))
                      / factorial
                  + get<I>(x);
        }
        addArea<I + 1>(dx, d_ix, dT, x);
    }

public:
    int m_order; /**< Integration order */
    /**
     * Integrate one step.
     * @param t0 initial time.
     * @param dT sampling time.
     * @param m_order integration m_order.
     * @return true in case of success, false otherwise.
     */
    bool oneStepIntegration(const std::chrono::nanoseconds& t0, const std::chrono::nanoseconds& dT);

    /**
     * Set the integration order
     * @param order integration order
     * @return true in case of success, false otherwise.
     */
    bool setOrder(const int& order)
    {
        m_order = order;
        return true;
    }
};

template <class _DynamicalSystem>
bool ApproxExactIntegrator<_DynamicalSystem>::oneStepIntegration(const std::chrono::nanoseconds& t0,
                                                                 const std::chrono::nanoseconds& dT)
{
    constexpr auto errorPrefix = "[ApproxExactIntegrator::oneStepIntegration]";
    if (this->m_dynamicalSystem == nullptr)
    {
        log()->error("{} Please specify the dynamical system.", errorPrefix);
        return false;
    }

    if (!this->m_dynamicalSystem->dynamics(t0, this->m_computationalBufferStateDerivative))
    {
        log()->error("{} Unable to compute the system dynamics.", errorPrefix);
        return false;
    }

    this->m_computationalBufferState = this->m_dynamicalSystem->getState();

    // compute higher order derivatives
    if (m_order > 1)
    {
        this->m_dynamicalSystem->setState(this->m_computationalBufferState);
        this->m_computationalBufferHigherStateDerivative.resize(m_order - 1);
        for (int i = 0; i < m_order - 1; i++)
        {
            if (!this->m_dynamicalSystem
                     ->dynamics(t0, this->m_computationalBufferHigherStateDerivative[i]))
            {
                log()->error("{} Unable to compute the system dynamics.", errorPrefix);
                return false;
            }
            if (!this->m_dynamicalSystem->setState(
                    this->m_computationalBufferHigherStateDerivative[i]))
            {
                log()->error("{} Unable to set the new state in the dynamical system.",
                             errorPrefix);
                return false;
            }
        }
    }
    this->addArea(this->m_computationalBufferStateDerivative,
                  this->m_computationalBufferHigherStateDerivative,
                  dT,
                  this->m_computationalBufferState);

    if (!this->m_dynamicalSystem->setState(this->m_computationalBufferState))
    {
        log()->error("{} Unable to set the new state in the dynamical system.", errorPrefix);
        return false;
    }

    return true;
}

} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // APPROXEXACTINTEGRATOR_H
