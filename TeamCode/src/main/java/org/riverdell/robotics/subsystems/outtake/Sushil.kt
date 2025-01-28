package org.riverdell.robotics.subsystems.outtake

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.motionProfiledServo
import org.riverdell.robotics.utilities.managed.ServoBehavior
import org.riverdell.robotics.utilities.motionprofile.Constraint
import java.util.concurrent.CompletableFuture

class Sushil(private val robot: HypnoticRobot) : AbstractSubsystem() {
    private val claw = motionProfiledServo("claw", robot.hardware.claw, Constraint.HALF.scale(10.5))

    private val wrist =
        motionProfiledServo("wrist", robot.hardware.wrist, Constraint.HALF.scale(50.5))
    private val pivotLeft =
        motionProfiledServo("left", robot.hardware.pivotLeft, Constraint.HALF.scale(100.5))
    private val pivotRight =
        motionProfiledServo("right", robot.hardware.pivotRight, Constraint.HALF.scale(100.5))

    var clawState = ClawState.Closed
    var wristState = WristState.Lateral
    var pivotState = PivotState.Initialize

    fun setClawState(state: ClawState): CompletableFuture<*> = let {
        if (clawState == state)
            return@let CompletableFuture.completedFuture(null)

        clawState = state
        return@let updateClawState()
    }

    private fun updateClawState(): CompletableFuture<*> {
        clawRotateTo(clawState.position)
        return CompletableFuture.completedFuture(null)
    }

    fun setWrist(state: WristState) = let {
        wristState = state
        return@let wristRotateTo(state.position)
    }

    fun setPivotState(state: PivotState) = let {
        if (pivotState == state)
            return@let CompletableFuture.completedFuture(null)

        pivotState = state
        return@let updatePivotState()
    }

    private fun updatePivotState(): CompletableFuture<*> {
        return pivotRotateTo(pivotState.rightPosition)
    }

    private fun clawRotateTo(position: Double) {
        claw.setTarget(
            position,
            ServoBehavior.Direct
        )
    }

    private fun wristRotateTo(position: Double) = wrist.setTarget(position, ServoBehavior.MotionProfile)
    private fun pivotRotateTo(position: Double) = CompletableFuture.allOf(
        pivotLeft.setTarget(1.0 - position, ServoBehavior.MotionProfile),
        pivotRight.setTarget(position, ServoBehavior.MotionProfile)
    )
/*    private fun wristRotateTo(position: Double): CompletableFuture<*> {
        wrist.unwrapServo().position = position
        return CompletableFuture.completedFuture(null)
    }

    private fun pivotRotateTo(position: Double): CompletableFuture<*>
    {
        pivotLeft.unwrapServo().position = 1.0 - position
        pivotRight.unwrapServo().position = position
        return CompletableFuture.completedFuture(null)
    }*/

    override fun start()
    {

    }

    override fun doInitialize()
    {

    }
}