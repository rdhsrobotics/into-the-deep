package org.riverdell.robotics.utilities.managed

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import io.liftgate.robotics.mono.states.StateHolder
import io.liftgate.robotics.mono.states.StateResult
import org.riverdell.robotics.HypnoticRobot
import java.util.UUID
import java.util.concurrent.CompletableFuture
import kotlin.math.abs
import kotlin.math.sign

class ManagedMotorGroup(
    stateHolder: StateHolder,
    var pid: PIDCoefficients, // P, I, D gains
    var kV: Double = 0.1, // velocity gain
    var kA: Double = 0.1, // acceleration gain
    var kStatic: Double = 0.1, // additive feedforward
    /**
     * Inputs: Measured position, target position, measured velocity
     */
    private var kF: (Double, Double, Double?) -> Double = { _, _, _ -> 0.0 },
    private val tolerance: Long = 3,
    private val master: DcMotorEx,
    private val slaves: List<DcMotorEx> = listOf()
)
{
    private var stuckProtection: StuckProtection? = null
    private var previousPosition: Int = 0


    private var stable = System.currentTimeMillis()
    private var pidfController = pidfBuilder()

    private fun pidfBuilder(): PIDFController =
        PIDFController(pid, kV, kA, kStatic, { measured, velocity ->
            kF(measured, pidfController.targetPosition, velocity)
        })

    /**
     * No PID updates.
     */
    private var idle = true

    private val groupId = UUID.randomUUID()
    private val state by stateHolder.state<Int>(
        write = {
            stable = System.currentTimeMillis()

            master.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            slaves.forEach { slave ->
                slave.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }

            pidfController.targetPosition = it.toDouble()
            idle = false
        },
        read = {
            master.currentPosition
        },
        complete = complete@{ current, target ->
            val isAtPoint = (abs(target - current) <= tolerance)

            if (isAtPoint) {
                return@complete true
            }

            if (enableStuckProtectionWhenTargetingOut && target - current < 0) {
                return@complete false
            }

            if (stuckProtection != null) {
                val diffs = master.velocity
                previousPosition = current

                if (diffs > stuckProtection!!.minimumRequiredPositionDifference)
                {
                    stable = System.currentTimeMillis()
                    return@complete false
                } else
                {
                    return@complete System.currentTimeMillis() - stable > stuckProtection!!.timeStuckUnderMinimumMillis
                }
            } else {
                return@complete false
            }
        }
    )

    fun currentPosition() = state.current()

    fun resetEncoders()
    {
        master.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slaves.forEach { slave ->
            slave.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        }
    }

    private var customPower: Double? = null
    fun usingCustomPower(customPower: Double)
    {
        this.customPower = customPower
    }

    fun resetCustomPower()
    {
        this.customPower = null
    }

    companion object
    {
        var keepEncoderPositions = false
    }

    init
    {
        if (!keepEncoderPositions)
        {
            resetEncoders()
        } else
        {
            keepEncoderPositions = false
        }

        master.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        slaves.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        /**
         * Continuously update the power of the motor to keep it at the
         * desired target value set in the [PIDFController].
         */
        fun generatePower(current: Int): Double?
        {
            if (idle)
            {
                return null
            }

            if (customPower != null)
            {
                return customPower
            }

            val velocity = master.velocity
            val voltage = HypnoticRobot.instance.drivetrain.voltage()

            return pidfController
                .update(
                    measuredPosition = current.toDouble(),
                    measuredVelocity = velocity
                )
                .let {
                    it * (13.0 / voltage)
                }
                .coerceIn(-1.0, 1.0)
        }

        if (slaves.isEmpty())
        {
            state.additionalPeriodic { current, _ ->
                val power = generatePower(current)
                    ?: return@additionalPeriodic

                master.power = power
            }
        } else
        {
            state.additionalPeriodic { current, _ ->
                val power = generatePower(current)
                    ?: return@additionalPeriodic

                master.power = power
                slaves.forEach { slave ->
                    slave.power = power
                }
            }
        }
    }

    fun cancel() = state.reset()

    fun supplyPowerToAll(power: Double)
    {
        cancel()

        master.power = power
        slaves.forEach { slave ->
            slave.power = power
        }
    }

    fun rebuild()
    {
        pidfController = pidfBuilder()
    }

    inner class FeedForwardDSL
    {

        fun static(value: Double)
        {
            kF = { _, _, _ -> value }
            pidfController = pidfBuilder()
        }

        fun none()
        {
            kF = { _, _, _ -> 0.0 }
            pidfController = pidfBuilder()
        }

        fun dynamic(block: (Double, Double, Double?) -> Double)
        {
            kF = block
            pidfController = pidfBuilder()
        }

    }

    fun feedForward(block: FeedForwardDSL.() -> Unit) = FeedForwardDSL().block()

    fun enableStuckProtection(stuckProtection: StuckProtection) = apply {
        this.stuckProtection = stuckProtection
    }

    var enableStuckProtectionWhenTargetingOut = false
    fun onlyEnableStuckProtectionWhenTargetingOut() = apply {
        enableStuckProtectionWhenTargetingOut = true
    }

    fun configure(block: PIDFController.() -> Unit) = apply {
        pidfController.block()
    }

    fun configureMotors(block: DcMotorEx.() -> Unit) = apply {
        master.block()
        slaves.forEach(block)
    }

    fun isTravelling() = state.inProgress()

    private var timeout = 2000L
    fun withTimeout(timeout: Long) = apply { this.timeout = timeout }

    /**
     * Overrides current state and goes to a target position.
     */
    fun goTo(target: Int): CompletableFuture<*>
    {
        exitIdle()
        kotlin.runCatching {
            state.reset()
        }
        return state.override(target, timeout = timeout)
            .exceptionally {
                it.printStackTrace()
                return@exceptionally null
            }
    }

    /**
     * Re-activates PID.
     */
    fun exitIdle()
    {
        idle = false
    }

    /**
     * Releases motor powers and stops all PID activity.
     */
    fun idle()
    {
        if (idle)
        {
            return
        }

        state.reset()
        pidfController.reset()
        idle = true

        master.power = 0.0
        slaves.forEach {
            it.power = 0.0
        }
    }
}