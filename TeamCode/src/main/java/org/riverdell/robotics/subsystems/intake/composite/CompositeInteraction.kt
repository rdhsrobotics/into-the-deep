package org.riverdell.robotics.subsystems.intake.composite

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel
import java.util.concurrent.CompletableFuture

class CompositeInteraction(private val robot: HypnoticRobot) : AbstractSubsystem() {
    var state = InteractionCompositeState.Rest
    var attemptedState: InteractionCompositeState? = null
    var attemptTime = System.currentTimeMillis()
    var outtakeLevel = OuttakeLevel.Bar1
    var lastOuttakeBegin = System.currentTimeMillis()

    fun outtakeNext(): CompletableFuture<*> {
        if (state != InteractionCompositeState.Outtaking) {
            return CompletableFuture.completedFuture(null)
        }

        if (outtakeLevel.next() == null) {
            return CompletableFuture.completedFuture(null)
        }

        outtakeLevel = outtakeLevel.next()!!
        return robot.lift.extendToAndStayAt(outtakeLevel.encoderLevel)
            .exceptionally { return@exceptionally null }
    }

    fun outtakePrevious(): CompletableFuture<*> {
        if (state != InteractionCompositeState.Outtaking) {
            return CompletableFuture.completedFuture(null)
        }

        if (outtakeLevel.previous() == null) {
            return CompletableFuture.completedFuture(null)
        }

        outtakeLevel = outtakeLevel.previous()!!
        return robot.lift.extendToAndStayAt(outtakeLevel.encoderLevel)
            .exceptionally { return@exceptionally null }
    }

    fun outtakeLevel(newLevel: OuttakeLevel): CompletableFuture<*> {
        if (state != InteractionCompositeState.Outtaking) {
            return CompletableFuture.completedFuture(null)
        }

        outtakeLevel = newLevel
        return robot.lift.extendToAndStayAt(newLevel.encoderLevel)
            .exceptionally { return@exceptionally null }
    }

    fun initialOuttakeFromRest(preferredLevel: OuttakeLevel = OuttakeLevel.Bar1) =
        stateMachineRestrict(
            InteractionCompositeState.Rest,
            InteractionCompositeState.Outtaking,
            ignoreInProgress = robot !is HypnoticAuto.HypnoticAutoRobot
        ) {
            outtakeLevel = preferredLevel
            robot.outtake.depositRotation()
            robot.outtake.depositCoaxial()

            CompletableFuture.allOf(
                robot.lift.extendToAndStayAt(outtakeLevel.encoderLevel)
            )
        }

    fun initialOuttake(preferredLevel: OuttakeLevel = OuttakeLevel.HighBasket) = stateMachineRestrict(
        InteractionCompositeState.OuttakeReady,
        InteractionCompositeState.Outtaking,
        ignoreInProgress = robot !is HypnoticAuto.HypnoticAutoRobot
    ) {
        outtakeLevel = preferredLevel
        lastOuttakeBegin = System.currentTimeMillis()
        CompletableFuture.allOf(
            robot.lift.extendToAndStayAt(outtakeLevel.encoderLevel)
        )
    }

    fun outtakeCompleteAndRestFromOuttakeReady() = stateMachineRestrict(
        InteractionCompositeState.OuttakeReady,
        InteractionCompositeState.Rest,
        ignoreInProgress = true
    ) {
        CompletableFuture.runAsync {
            outtake.openClaw()
            Thread.sleep(350L)

            outtake.readyRotation()
            outtake.readyCoaxial()
        }
    }

    fun outtakeCompleteAndRest() = stateMachineRestrict(
        InteractionCompositeState.Outtaking,
        InteractionCompositeState.Rest,
        ignoreInProgress = true
    ) {
        CompletableFuture.runAsync {
            outtake.openClaw()
            Thread.sleep(125L)

            outtake.readyRotation()
            outtake.readyCoaxial().join()

            lift.extendToAndStayAt(0)
            CompletableFuture.completedFuture(null)
        }
    }

    fun specimenCompleteAndRest() = stateMachineRestrict(
        InteractionCompositeState.OuttakeReady,
        InteractionCompositeState.Rest,
        ignoreInProgress = true
    ) {
        CompletableFuture.runAsync {
            lift.extendToAndStayAt(1000)
            Thread.sleep(750L)
            outtake.openClaw()
            Thread.sleep(125L)

            lift.extendToAndStayAt(0)

            outtake.readyRotation()
            outtake.readyCoaxial()

            CompletableFuture.completedFuture(null)
        }
    }

    fun wallOuttakeFromRest() =
        stateMachineRestrict(
            InteractionCompositeState.Rest,
            InteractionCompositeState.WallIntakeViaOuttake
        ) {
            CompletableFuture.allOf(
                robot.outtake.forceRotation(),
                robot.outtake.outsideIntakeCoaxial(),
                robot.outtake.openClaw()
            )
        }

    fun inToOut() = stateMachineRestrict(
        InteractionCompositeState.Rest,
        InteractionCompositeState.OuttakeReady
    ) {
        CompletableFuture.allOf(
            robot.outtake.depositCoaxial(),
            robot.outtake.depositRotation()
        )
    }

    fun wallOuttakeToOuttakeReady() =
        stateMachineRestrict(
            InteractionCompositeState.WallIntakeViaOuttake,
            InteractionCompositeState.OuttakeReady
        ) {
            CompletableFuture.allOf(
                robot.outtake.closeClaw(),
                robot.outtake.depositCoaxial(),
                robot.outtake.depositRotation()
            )
        }

    fun prepareForPickup(
        wristState: WristState = WristState.Lateral,
        doNotUseAutoMode: Boolean = false,
        wideOpen: Boolean = false
    ) =
        stateMachineRestrict(InteractionCompositeState.Rest, InteractionCompositeState.Pickup) {
            intakeV4B.v4bUnlock()
                .thenAcceptAsync {
                    extension.extendToAndStayAt(IntakeConfig.MAX_EXTENSION)
                        .thenAccept {
                            if (robot !is HypnoticAuto.HypnoticAutoRobot)
                            {
                                extension.slides.idle()
                            }
                        }

                    outtake.readyCoaxial()
                    outtake.readyRotation()
                    outtake.openClaw()

                    CompletableFuture.allOf(
                        intakeV4B.v4bSampleGateway(doNotUseAutoMode),
                        intakeV4B.coaxialIntake()
                            .thenCompose {
                                intake.openIntake(wideOpen)
                            },
                        intake.setWrist(wristState)
                    ).join()
                }
        }

    fun intakeAndConfirm(extras: () -> Unit = {}) =
        stateMachineRestrict(InteractionCompositeState.Pickup, InteractionCompositeState.Confirm) {
            intakeV4B.v4bSamplePickup().thenRunAsync {
                Thread.sleep(125L)
                intake.closeIntake()
                Thread.sleep(125L)

                extras()

                CompletableFuture.allOf(
                    intakeV4B.v4bIntermediate(),
                    intakeV4B.coaxialIntermediate(),
                    intake.setWrist(WristState.Lateral)
                ).join()
            }
        }

    fun declineAndIntake() =
        stateMachineRestrict(InteractionCompositeState.Confirm, InteractionCompositeState.Pickup) {
            CompletableFuture.allOf(
                intakeV4B.v4bSampleGateway(),
                intakeV4B.coaxialIntake()
                    .thenCompose {
                        intake.openIntake()
                    },
                intake.lateralWrist()
            )
        }

    private fun HypnoticRobot.performTransferSequence() {
        CompletableFuture.allOf(
            outtake.transferRotation(),
            outtake.transferCoaxial()
        ).join()

        Thread.sleep(100)
        CompletableFuture.allOf(
            intake.openIntake(),
            outtake.closeClaw()
        )
        Thread.sleep(50)

        CompletableFuture.allOf(
            outtake.depositRotation()
                .thenRunAsync {
                    intake.closeIntake()
                },
            outtake.depositCoaxial()
        ).join()
    }

    fun reTransferOuttakeReady() =
        stateMachineRestrict(
            InteractionCompositeState.OuttakeReady,
            InteractionCompositeState.OuttakeReady
        ) {
            CompletableFuture.runAsync {
                outtake.openClaw()
                performTransferSequence()
            }
        }

    fun confirmAndTransferAndReady() =
        stateMachineRestrict(
            InteractionCompositeState.Confirm,
            InteractionCompositeState.OuttakeReady
        ) {
            CompletableFuture.allOf(
                extension.extendToAndStayAt(0),
                CompletableFuture.runAsync {
                    Thread.sleep(250L)
                    intakeV4B.coaxialRest().join()
                },
                intakeV4B.v4bUnlock()
            ).thenRunAsync {
                intakeV4B.v4bLock().join()
                performTransferSequence()
                extension.slides.idle()
            }
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
}