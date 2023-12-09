package org.riverdell.robotics.xdk.opmodes.pipeline.pid

import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.absoluteValue

class PIDController(
    private val kP: Double,
    private val kI: Double,
    private val kD: Double,
    val setPoint: Double,
    private val setPointTolerance: Int,
    private val maxTotalError: Int,
    private val telemetry: Telemetry
)
{
    private var integral = 0.0
    private var previousError = 0.0
    private var totalError = 0.0
    private var previousValue: Double? = null
    private var velocity = 0.0

    private var customErrorCalculator: ((current: Double) -> Double)? = null

    fun customErrorCalculator(block: (current: Double) -> Double) =
        apply { customErrorCalculator = block }

    fun calculate(currentValue: Double): Double
    {
        val error = customErrorCalculator?.invoke(currentValue)
            ?: (setPoint - currentValue)

        telemetry.addData("Error", error)
        telemetry.addData("Target", setPoint)

        val prevValue = (previousValue ?: currentValue) - currentValue

        integral += error
        val derivative = error - previousError

        val output = (kP * error) + (kI * integral) + (kD * derivative)
        telemetry.addData("Output", output)
        telemetry.addData("Input", currentValue)
        telemetry.update()

        previousError = error
        totalError += error

        velocity = prevValue - currentValue
        previousValue = currentValue

        return output
    }

    fun atSetPoint(currentValue: Double): Boolean
    {
        return velocity.absoluteValue < 15 && (
                setPoint - setPointTolerance <= currentValue &&
                        currentValue <= setPoint + setPointTolerance
        )
    }
}
