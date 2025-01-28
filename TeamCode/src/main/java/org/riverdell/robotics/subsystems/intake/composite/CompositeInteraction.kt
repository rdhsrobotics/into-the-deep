package org.riverdell.robotics.subsystems.intake.composite

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.outtake.ClawState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel
import org.riverdell.robotics.subsystems.outtake.PivotState
import org.riverdell.robotics.subsystems.outtake.WristState
import java.util.concurrent.CompletableFuture

class CompositeInteraction(private val robot: HypnoticRobot) : AbstractSubsystem() {
    var state = InteractionCompositeState.Initial
    var attemptedState: InteractionCompositeState? = null
    var attemptTime = System.currentTimeMillis()
    var outtakeLevel = OuttakeLevel.rest
    var lastOuttakeBegin = System.currentTimeMillis()

    fun outtakeNext(): CompletableFuture<*> {
        if (outtakeLevel.next() == null) {
            return CompletableFuture.completedFuture(null)
        }

        outtakeLevel = outtakeLevel.next()!!
        return robot.lift.extendToAndStayAt(outtakeLevel.encoderLevel)
            .exceptionally { return@exceptionally null }
    }

    fun outtakePrevious(): CompletableFuture<*> {
        if (outtakeLevel.previous() == null) {
            return CompletableFuture.completedFuture(null)
        }

        outtakeLevel = outtakeLevel.previous()!!
        return robot.lift.extendToAndStayAt(outtakeLevel.encoderLevel)
            .exceptionally { return@exceptionally null }
    }

    fun outtakeLevel(newLevel: OuttakeLevel): CompletableFuture<Void> {
        outtakeLevel = newLevel
        return robot.lift.extendToAndStayAt(newLevel.encoderLevel)
            .thenRun {  }
    }

    private fun stateMachineRestrict(
        from: InteractionCompositeState, to: InteractionCompositeState,
        ignoreInProgress: Boolean = false,
        supplier: HypnoticRobot.() -> CompletableFuture<Void>
    ): CompletableFuture<Void> {
        if (state != from) {
            return CompletableFuture.completedFuture(null)
        }

        state = if (!ignoreInProgress) {
            InteractionCompositeState.InProgress
        } else {
            to
        }

        attemptedState = to
        attemptTime = System.currentTimeMillis()

        return supplier(robot)
            .whenComplete { _, exception ->
                exception?.printStackTrace()
                if (!ignoreInProgress) {
                    state = to
                }
            }
    }

    override fun doInitialize() {

    }

    override fun start() {
    }

    fun waitForState(state: InteractionCompositeState,
                     timeout: Long = 10000L): Boolean {
        val start = System.currentTimeMillis()
        while (this.state != state)
        {
            if (System.currentTimeMillis() - start > timeout)
            {
                return false
            }
            Thread.sleep(50L)
        }

        return true
    }

    fun fromHoverToDepositReady() = stateMachineRestrict(
        InteractionCompositeState.Hover,
        InteractionCompositeState.DepositReady
    ) {
        robot.outtake.setPivotState(PivotState.Pickup)
            .thenAcceptAsync {
                robot.outtake.setClawState(ClawState.Closed).join()
                Thread.sleep(125L)
            }
            .thenAcceptAsync {
                robot.outtake.setWrist(WristState.Lateral)
                robot.outtake.setPivotState(PivotState.Scoring).join()
            }
    }

    fun fromRestToHover() = stateMachineRestrict(
        InteractionCompositeState.Rest,
        InteractionCompositeState.Hover
    ) {
        robot.outtake.setPivotState(PivotState.Hover)
        robot.outtake.setClawState(ClawState.Open)
        CompletableFuture.completedFuture(null)
    }

    fun fromInitialToRest() = stateMachineRestrict(
        InteractionCompositeState.Initial,
        InteractionCompositeState.Rest
    ) {
        robot.outtake.setPivotState(PivotState.Scoring)
        CompletableFuture.completedFuture(null)
    }

    fun scoreAndRest() = stateMachineRestrict(
        InteractionCompositeState.DepositReady,
        InteractionCompositeState.Rest
    ) {
        robot.outtake.setClawState(ClawState.Open)
            .thenComposeAsync {
                robot.outtake.setPivotState(PivotState.PostScore).join()
                outtakeLevel(OuttakeLevel.rest)
            }
            .thenRun {
                robot.outtake.setClawState(ClawState.Closed)
            }

        CompletableFuture.completedFuture(null)
    }
}