package org.riverdell.robotics.utilities.managed

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import io.liftgate.robotics.mono.states.StateHolder
import org.riverdell.robotics.utilities.motionprofile.AsymmetricMotionProfile
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints
import org.riverdell.robotics.utilities.motionprofile.ProfileState
import java.util.concurrent.CompletableFuture

/**
 * A [Servo] wrapper that keeps track of motion profile states.
 *
 * @author Subham
 */
class ManagedServo(
    private val id: String,
    private val servo: ServoImplEx,
    stateHolder: StateHolder,
    var constraints: ProfileConstraints
) {
    private var motionProfile: AsymmetricMotionProfile? = null
    private var timer = ElapsedTime()
    private var behavior = ServoBehavior.MotionProfile
    private var profileState: ProfileState? = null

    private val state by stateHolder.state<Double>({
        timer = ElapsedTime()
        if (behavior == ServoBehavior.MotionProfile)
        {
            println("[${id}] Motion profile from ${servo.position} to $it")
            motionProfile = AsymmetricMotionProfile(
                servo.position,
                it,
                constraints
            )
        } else
        {
            println("[${id}] Direct from ${servo.position} to $it")
            motionProfile = null
        }
    }, { _ ->
        servo.position
    }, { _, target ->
        if (behavior == ServoBehavior.Direct) {
            println("[${id}] Direct COMPLETE to $target")
            servo.position = target
            return@state true
        } else
        {
            profileState = motionProfile?.calculate(timer.time())
            if (profileState == null)
            {
                println("[${id}] REVERTED TO DIRECT FROM MP to $target DUE TO NULL REF")
                servo.position = target
                return@state true
            }

            if (profileState!!.v == 0.0) {
                servo.position = target
                println("[${id}] MP COMPLETE to ${timer.milliseconds()}")
                return@state true
            }

            servo.position = profileState!!.x
            return@state false
        }
    })

    fun unwrapServo() = servo
    fun setTarget(
        target: Double,
        behavior: ServoBehavior = ServoBehavior.MotionProfile
    ): CompletableFuture<*> {
        this.behavior = behavior
        return state.override(target, 0L)
            .exceptionally {
                println("[${id}] FAILED TO OVERRIDE to $target")
                it.printStackTrace()
                return@exceptionally null
            }
    }

    fun cancelMotionProfile() = state.reset()
}