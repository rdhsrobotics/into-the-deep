package org.robotics.robotics.xdk.teamcode.subsystem.motionprofile.wrappers

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.robotics.robotics.xdk.teamcode.subsystem.motionprofile.AsymmetricMotionProfile
import org.robotics.robotics.xdk.teamcode.subsystem.motionprofile.ProfileConstraints
import java.util.concurrent.CompletableFuture

/**
 * A [Servo] wrapper that keeps track of motion profile states. Requires
 * the end user to run [runPeriodic].
 *
 * @author Subham
 */
class MotionProfiledServo(
    private val servo: Servo,
    private val constraints: () -> ProfileConstraints
)
{
    private var motionProfile: AsymmetricMotionProfile? = null
    private var timer: ElapsedTime? = null
    private var onComplete: CompletableFuture<Void>? = null

    fun unwrapServo() = servo
    fun setMotionProfileTarget(target: Double): CompletableFuture<Void>
    {
        val completableFuture = CompletableFuture<Void>()
        onComplete?.completeExceptionally(OverrideException())
        onComplete = completableFuture

        motionProfile = AsymmetricMotionProfile(
            servo.position,
            target,
            constraints()
        )
        timer = ElapsedTime()
        return completableFuture
    }

    /**
     * Sets the servo's position to the value given
     * for the current time in the [AsymmetricMotionProfile].
     */
    fun runPeriodic()
    {
        if (motionProfile == null)
        {
            timer = null
            return
        }

        val position = motionProfile!!.calculate(timer!!.time())
        servo.position = position.x

        if (servo.position == motionProfile!!.finalPosition)
        {
            motionProfile = null
            onComplete?.complete(null)
            onComplete = null
        }
    }

    fun cancelMotionProfile()
    {
        motionProfile = null
        onComplete?.completeExceptionally(OverrideException(true))
        onComplete = null
        timer = ElapsedTime()
    }

    fun isTargetMotionActive() = motionProfile != null

    /**
     * Overrides any existing motion profile and sets
     * the target position of the backing servo.
     */
    fun setTarget(targetPosition: Double)
    {
        cancelMotionProfile()
        servo.position = targetPosition
    }
}