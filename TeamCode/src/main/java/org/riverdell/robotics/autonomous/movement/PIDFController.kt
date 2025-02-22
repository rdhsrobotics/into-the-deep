package org.riverdell.robotics.autonomous.movement

import com.acmerobotics.roadrunner.control.PIDCoefficients
import java.util.LinkedList
import kotlin.math.abs
import kotlin.math.pow

/**
 * This is a PID controller (https://en.wikipedia.org/wiki/PID_controller)
 * for your robot. Internally, it performs all the calculations for you.
 * You need to tune your values to the appropriate amounts in order
 * to properly utilize these calculations.
 *
 *
 * The equation we will use is:
 * u(t) = kP * e(t) + kI * int(0,t)[e(t')dt'] + kD * e'(t) + kF
 * where e(t) = r(t) - y(t) and r(t) is the setpoint and y(t) is the
 * measured value. If we consider e(t) the positional error, then
 * int(0,t)[e(t')dt'] is the total error and e'(t) is the velocity error.
 */
class PIDFController @JvmOverloads constructor(
    var p: Double,
    var i: Double,
    var d: Double,
    var f: (Double, Double, Double) -> Double,
    sp: Double = 0.0,
    pv: Double = 0.0,
    useAverageVelocity: Boolean = true
) {
    private var setPoint: Double
    private var measuredValue: Double
    private var minIntegral: Double
    private var maxIntegral: Double

    /**
     * @return the positional error e(t)
     */
    var positionError: Double
        private set

    /**
     * @return the velocity error e'(t)
     */
    var velocityError: Double = 0.0
        private set

    private var totalError = 0.0
    private var prevErrorVal = 0.0
    private val useAverageVelocity: Boolean

    private var errorToleranceP = 0.05
    private var errorToleranceV = 0.1

    var lastTimeStamp: Double
        private set
    var periodMillis: Double
        private set
    var averageVelocity: Double
    private val prevVels: LinkedList<Pair<Double, Double>>

    /**
     * The base constructor for the PIDF controller
     */
    constructor(kp: Double, ki: Double, kd: Double, kf: (Double, Double, Double) -> Double, useAverageVelocity: Boolean) : this(
        kp,
        ki,
        kd,
        kf,
        0.0,
        0.0,
        useAverageVelocity
    )

    constructor(pidCoefficients: com.qualcomm.robotcore.hardware.PIDCoefficients, kf: (Double, Double, Double) -> Double) : this(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d, kf)

    /**
     * This is the full constructor for the PIDF controller. Our PIDF controller
     * includes a feed-forward value which is useful for fighting friction and gravity.
     * Our errorVal represents the return of e(t) and prevErrorVal is the previous error.
     *
     * @param sp The setpoint of the pid control loop.
     * @param pv The measured value of he pid control loop. We want sp = pv, or to the degree
     * such that sp - pv, or e(t) < tolerance.
     */
    init {
        setPoint = sp
        measuredValue = pv

        minIntegral = -1.0
        maxIntegral = 1.0

        lastTimeStamp = 0.0
        periodMillis = 0.0
        prevVels = LinkedList()
        averageVelocity = 0.0
        this.useAverageVelocity = useAverageVelocity

        positionError = setPoint - measuredValue
        reset()
    }


    fun reset() {
        totalError = 0.0
        prevErrorVal = 0.0
        lastTimeStamp = 0.0
    }

    /**
     * Sets the error which is considered tolerable for use with [.atSetPoint].
     *
     * @param positionTolerance Position error which is tolerable.
     */
    fun setTolerance(positionTolerance: Double) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY)
    }

    /**
     * Sets the error which is considered tolerable for use with [.atSetPoint].
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    fun setTolerance(positionTolerance: Double, velocityTolerance: Double) {
        errorToleranceP = positionTolerance
        errorToleranceV = velocityTolerance
    }

    /**
     * Returns the current setpoint of the PIDFController.
     *
     * @return The current setpoint.
     */
    fun getSetPoint(): Double {
        return setPoint
    }

    /**
     * Sets the setpoint for the PIDFController
     *
     * @param sp The desired setpoint.
     */
    fun setSetPoint(sp: Double) {
        if (sp != setPoint) {
            setPoint = sp
            positionError = setPoint - measuredValue
            //errorVal_v = (errorVal_p - prevErrorVal) / period;
        }
    }

    /**
     * Returns true if the error is within the percentage of the total input range, determined by
     * [.setTolerance].
     *
     * @return Whether the error is within the acceptable bounds.
     */
    fun atSetPoint(): Boolean {
        return (abs(positionError) < errorToleranceP
                && abs(velocityError) < errorToleranceV)
    }

    val coefficients: Pair<DoubleArray, (Double, Double, Double) -> Double>
        /**
         * @return the PIDF coefficients
         */
        get() = Pair(doubleArrayOf(p, i, d), f)

    val tolerance: DoubleArray
        /**
         * @return the tolerances of the controller
         */
        get() = doubleArrayOf(errorToleranceP, errorToleranceV)

    /**
     * Calculates the control value, u(t).
     *
     * @param pv The current measurement of the process variable.
     * @param sp The target the controller should drive pv to.
     * @param velocity Measured velocity
     * @return the value produced by u(t).
     */
    /**
     * Calculates the next output of the PIDF controller.
     *
     * @return the next output using the current measured value via
     * [.calculate].
     */

    @JvmOverloads
    fun calculate(pv: Double = measuredValue, sp: Double = setPoint, measuredVelocity: Double? = null): Double {
        if (setPoint != sp) setSetPoint(sp)
        prevErrorVal = positionError

        val currentTimeStamp = System.nanoTime().toDouble()
        if (lastTimeStamp == 0.0) lastTimeStamp = currentTimeStamp
        periodMillis = (currentTimeStamp - lastTimeStamp) / 1E6
        lastTimeStamp = currentTimeStamp

        measuredValue = pv
        positionError = setPoint - measuredValue

        if (abs(periodMillis) > 0.1) {
            val velocity = measuredVelocity ?: ((positionError - prevErrorVal) / (periodMillis / 1E3))
            prevVels.add(Pair(velocity, periodMillis))
            if (prevVels.size > 15) { //store up to 15 previous values of velocity and its associated period
                prevVels.pop()
            }
            //quadratic recency biased average: 100 ms outdated is valued 4 times less than current
            var sum = 0.0
            var periodSum = 0.0
            var denominator = 0.0
            var recencyWeight: Double
            for (i in prevVels.indices) {
                recencyWeight = 1 / (0.0003 * periodSum.pow(2) + 1)
                sum += prevVels[i].first * recencyWeight
                denominator += recencyWeight
                periodSum += prevVels[i].second
            }
            averageVelocity = if (denominator != 0.0) sum / denominator else velocity
            velocityError = if (useAverageVelocity) averageVelocity else velocity
        }

        /* if total error is the integral from 0 to t of e(t')dt', and
        e(t) = sp - pv, then the total error, E(t), equals sp*t - pv*t.
         */
        totalError += periodMillis * (setPoint - measuredValue)
        totalError = totalError.coerceIn(minIntegral, maxIntegral)

        // returns u(t)
        return p * positionError + i * totalError + d * velocityError + f(measuredValue, setPoint, velocityError)
    }

    fun setPIDF(kp: Double, ki: Double, kd: Double, kf: (Double, Double, Double) -> Double) {
        p = kp
        i = ki
        d = kd
        f = kf
    }

    fun setIntegrationBounds(integralMin: Double, integralMax: Double) {
        minIntegral = integralMin
        maxIntegral = integralMax
    }

    fun clearTotalError() {
        totalError = 0.0
    }
}