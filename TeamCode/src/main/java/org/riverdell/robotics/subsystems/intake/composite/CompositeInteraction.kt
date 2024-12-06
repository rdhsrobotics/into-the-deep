package org.riverdell.robotics.subsystems.intake.composite

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel
import java.util.concurrent.CompletableFuture

class CompositeInteraction(private val robot: HypnoticRobot) : AbstractSubsystem() {
    var state = InteractionCompositeState.Rest
    var attemptedState: InteractionCompositeState? = null
    var attemptTime = System.currentTimeMillis()
    var outtakeLevel = OuttakeLevel.Bar1

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
            ignoreInProgress = true
        ) {
            outtakeLevel = preferredLevel
            robot.outtake.depositRotation()
            robot.outtake.depositCoaxial()

            CompletableFuture.allOf(
                robot.lift.extendToAndStayAt(outtakeLevel.encoderLevel)
            )
        }

    fun initialOuttake(preferredLevel: OuttakeLevel = OuttakeLevel.Bar1) = stateMachineRestrict(
        InteractionCompositeState.OuttakeReady,
        InteractionCompositeState.Outtaking,
        ignoreInProgress = true
    ) {
        outtakeLevel = preferredLevel
        CompletableFuture.allOf(robot.lift.extendToAndStayAt(outtakeLevel.encoderLevel))
    }

    fun outtakeCompleteAndReturnToOuttakeReady() = stateMachineRestrict(
        InteractionCompositeState.Outtaking,
        InteractionCompositeState.OuttakeReady
    ) {
        CompletableFuture.runAsync {
            outtake.openClaw()
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

    fun exitOuttakeReadyToRest() = stateMachineRestrict(
        InteractionCompositeState.OuttakeReady,
        InteractionCompositeState.Rest,
        ignoreInProgress = true
    ) {
        robot.outtake.openClaw()
        CompletableFuture.allOf(
            robot.outtake.transferRotation(),
            robot.lift.extendToAndStayAt(0)
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
                robot.outtake.closeClaw()
            )
        }

    fun outtakeAndRest() =
        stateMachineRestrict(
            InteractionCompositeState.OuttakeReady,
            InteractionCompositeState.Rest
        ) {

            outtake.forceRotation()
                .thenRunAsync {
                    Thread.sleep(300L)
                    outtake.openClaw()
                    outtake.transferRotation().join()
                }
        }

    fun prepareForPickup(wristState: WristState = WristState.Lateral, wideOpen: Boolean = false) =
        stateMachineRestrict(InteractionCompositeState.Rest, InteractionCompositeState.Pickup) {
            intakeV4B.v4bUnlock()
                .thenAcceptAsync {
                    extension.extendToAndStayAt(IntakeConfig.MAX_EXTENSION)
                        .thenAccept {
                            extension.slides.idle()
                        }

                    outtake.transferRotation()
                    outtake.openClaw()

                    CompletableFuture.allOf(
                        intakeV4B.v4bSampleGateway(),
                        intakeV4B.coaxialIntake()
                            .thenCompose {
                                intake.openIntake(wideOpen)
                            },
                        intake.setWrist(wristState)
                    ).join()
                }
        }

    fun intakeAndConfirm() =
        stateMachineRestrict(InteractionCompositeState.Pickup, InteractionCompositeState.Confirm) {
//            intakeV4B.coaxialPickup()
            intakeV4B.v4bSamplePickup().thenRunAsync {
                Thread.sleep(125L)
                intake.closeIntake()
                Thread.sleep(125L)

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

    fun confirmAndTransferAndReady() =
        stateMachineRestrict(
            InteractionCompositeState.Confirm,
            InteractionCompositeState.OuttakeReady
        ) {
            CompletableFuture.allOf(
                extension.extendToAndStayAt(0),
                intakeV4B.coaxialRest(),
                intakeV4B.v4bTransfer()
            ).thenRunAsync {
                outtake.closeClaw()
                Thread.sleep(150)
                intake.openIntake()
                Thread.sleep(100)

                outtake.depositRotation()
                outtake.depositCoaxial()
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

        if (!ignoreInProgress) {
            state = InteractionCompositeState.InProgress
        } else {
            state = to
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
}