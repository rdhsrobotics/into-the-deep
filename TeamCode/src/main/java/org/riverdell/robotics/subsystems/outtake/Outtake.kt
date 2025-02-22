package org.riverdell.robotics.subsystems.outtake

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.motionProfiledServo
import org.riverdell.robotics.utilities.managed.ServoBehavior
import org.riverdell.robotics.utilities.motionprofile.Constraint
import java.util.concurrent.CompletableFuture

class Outtake(private val robot: HypnoticRobot) : AbstractSubsystem() {
    private val claw =
        motionProfiledServo("outtake_c", robot.hardware.outtakeClaw, Constraint.HALF.scale(10.5))

    private val coaxial = motionProfiledServo(
        "outtake_cr",
        robot.hardware.outtakeCoaxial,
        Constraint.HALF.scale(50.5)
    )
    private val leftRotation = motionProfiledServo(
        "outtake_lr",
        robot.hardware.outtakeRotationLeft,
        Constraint.HALF.scale(50.5)
    )
    private val rightRotation = motionProfiledServo(
        "outtake_rr",
        robot.hardware.outtakeRotationRight,
        Constraint.HALF.scale(50.5)
    )

    var clawState = OuttakeClawState.Closed
    var coaxialState = OuttakeCoaxialState.Ready
    var rotationState = OuttakeRotationState.Transfer

    fun readyRotation() = setRotation(OuttakeRotationState.Ready)
    fun transferRotation() = setRotation(OuttakeRotationState.Transfer)
    fun transferRotationForce() = setRotationForce(OuttakeRotationState.Transfer)
    fun forceRotation() = setRotation(OuttakeRotationState.Force)
    fun specimenRotation() = setRotation(OuttakeRotationState.Specimen)

    fun depositRotation() = setRotation(OuttakeRotationState.Deposit)
    fun autoPreDepositRotation() = setRotation(OuttakeRotationState.AutoPreDeposit)

    fun setRotation(state: OuttakeRotationState) = let {
        if (rotationState == state)
            return@let CompletableFuture.completedFuture(null)

        rotationState = state
        return@let updateRotationState()
    }

    fun setRotationForce(state: OuttakeRotationState) = let {
        if (rotationState == state)
            return@let CompletableFuture.completedFuture(null)

        rotationState = state
        return@let updateRotationStateForce()
    }

    private fun updateRotationState(): CompletableFuture<*> {
        return rotationRotateTo(rotationState.position)
    }

    private fun updateRotationStateForce(): CompletableFuture<*> {
        return rotationRotateToForce(rotationState.position)
    }

    fun openClaw() = setClaw(OuttakeClawState.Open)
    fun closeClaw() = setClaw(OuttakeClawState.Closed)

    fun setClaw(state: OuttakeClawState) = let {
        if (clawState == state)
            return@let CompletableFuture.completedFuture(null)

        clawState = state
        return@let updateClawState()
    }

    private fun updateClawState(): CompletableFuture<*> {
        CompletableFuture.runAsync {
            clawRotateTo(clawState.position)
        }

        return CompletableFuture.completedFuture(null)
    }

    fun readyCoaxial() = setCoaxial(OuttakeCoaxialState.Ready)
    fun transferCoaxial() = setCoaxial(OuttakeCoaxialState.Transfer)
    fun transferCoaxialForce() = setCoaxialForce(OuttakeCoaxialState.Transfer)
    fun depositCoaxial() = setCoaxial(OuttakeCoaxialState.Deposit)
    fun specimenCoaxial() = setCoaxial(OuttakeCoaxialState.Specimen)
    fun outsideIntakeCoaxial() = setCoaxial(OuttakeCoaxialState.OutsideIntake)

    fun setCoaxial(state: OuttakeCoaxialState) = let {
        if (coaxialState == state)
            return@let CompletableFuture.completedFuture(null)

        coaxialState = state
        return@let updateCoaxialState()
    }

    fun setCoaxialForce(state: OuttakeCoaxialState) = let {
        if (coaxialState == state)
            return@let CompletableFuture.completedFuture(null)

        coaxialState = state
        return@let updateCoaxialStateForce()
    }

    private fun updateCoaxialState(): CompletableFuture<*> {
        return coaxialRotateTo(coaxialState.position)
    }

    private fun updateCoaxialStateForce(): CompletableFuture<*> {
        return coaxialRotateToForce(coaxialState.position)
    }

    private fun clawRotateTo(position: Double) {
        claw.unwrapServo().position = position
    }

    private fun coaxialRotateTo(position: Double) =
        coaxial.setTarget(position, ServoBehavior.MotionProfile)

    private fun rotationRotateTo(position: Double) = let {
        CompletableFuture.allOf(
            leftRotation.setTarget(1.0 - position, ServoBehavior.MotionProfile),
//            rightRotation.setTarget(position, ServoBehavior.MotionProfile)
        )
    }

    private fun coaxialRotateToForce(position: Double) =
        coaxial.setTarget(position, ServoBehavior.Direct)

    private fun rotationRotateToForce(position: Double) = let {
        CompletableFuture.allOf(
            leftRotation.setTarget(1.0 - position, ServoBehavior.Direct),
//            rightRotation.setTarget(position, ServoBehavior.Direct)
        )
    }

    override fun start() {

    }

    override fun doInitialize() {

    }
}