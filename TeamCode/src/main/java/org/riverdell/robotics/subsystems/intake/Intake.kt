package org.riverdell.robotics.subsystems.intake

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.motionProfiledServo
import org.riverdell.robotics.utilities.managed.ServoBehavior
import org.riverdell.robotics.utilities.motionprofile.Constraint
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints
import java.util.concurrent.CompletableFuture

class Intake(private val robot: HypnoticRobot) : AbstractSubsystem() {
    val wrist = motionProfiledServo("intake_wr", robot.hardware.intakeWrist, ProfileConstraints(45.0, 12.0, 12.0))
    private val leftGrip =
        motionProfiledServo("intake_lg", robot.hardware.intakeClawLeft, Constraint.HALF.scale(10.5))
    private val rightGrip =
        motionProfiledServo("intake_rg", robot.hardware.intakeClawRight, Constraint.HALF.scale(10.5))

    var intakeState = IntakeState.Closed
    var wristState = WristState.Lateral

    private var dynamicPosition = 0.5
    fun currentDynamicPosition() = dynamicPosition

    fun openIntake(wideOpen: Boolean = false) = setIntake(
        if (wideOpen) IntakeState.WideOpen else IntakeState.Open
    )
    fun closeIntake() = setIntake(IntakeState.Closed)

    fun setIntake(state: IntakeState) = let {
        if (intakeState == state)
            return@let CompletableFuture.completedFuture(null)

        intakeState = state
        return@let updateIntakeState()
    }

    fun lateralWrist() = setWrist(WristState.Lateral)
    fun perpendicularWrist() = setWrist(WristState.Perpendicular)
    fun dynamicWrist(position: Double) = let {
        dynamicPosition = position
        return@let setWrist(WristState.Dynamic)
    }

    fun setWrist(state: WristState) = let {
        if (state == WristState.Dynamic) {
            return@let wrist.setTarget(dynamicPosition, ServoBehavior.Direct)
        }

        dynamicPosition = state.position
        wristState = state
        return@let updateWristState()
    }

    private fun updateIntakeState(): CompletableFuture<*> {
        return intakeRotateTo(intakeState)
    }

    private fun updateWristState(): CompletableFuture<*> {
        if (wristState == WristState.Dynamic) {
            return wrist.setTarget(dynamicPosition, ServoBehavior.Direct)
        }

        return wristRotateTo(wristState.position)
    }

    private fun wristRotateTo(position: Double) = wrist.setTarget(position, ServoBehavior.Direct)
    private fun intakeRotateTo(state: IntakeState) = CompletableFuture.allOf(
        leftGrip.setTarget(state.positionLeft, ServoBehavior.Direct),
        rightGrip.setTarget(state.positionRight, ServoBehavior.Direct)
    )

    override fun start() {

    }

    override fun doInitialize() {
        updateIntakeState()
        updateWristState()
    }
}