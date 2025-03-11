package org.riverdell.robotics.subsystems.intake.composite

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel
import org.riverdell.robotics.subsystems.slides.ExtensionConfig
import org.riverdell.robotics.subsystems.slides.LiftConfig
import org.riverdell.robotics.utilities.motionprofile.Constraint
import java.util.concurrent.CompletableFuture

class CompositeInteraction(private val robot: HypnoticRobot) : AbstractSubsystem() {
    var state = InteractionCompositeState.Rest
    var attemptedState: InteractionCompositeState? = null
    var attemptTime = System.currentTimeMillis()
    var outtakeLevel = OuttakeLevel.Bar2
    var lastOuttakeBegin = System.currentTimeMillis()
    var shouldAutoGuide = false

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

    fun initialOuttakeFromRest(
        preferredLevel: OuttakeLevel = OuttakeLevel.Bar2,
        shouldEnterPreDepositIfAvailable: Boolean = true
    ) = stateMachineRestrict(
        InteractionCompositeState.Rest,
        InteractionCompositeState.Outtaking,
        ignoreInProgress = robot !is HypnoticAuto.HypnoticAutoRobot
    ) {
        outtakeLevel = preferredLevel

        if (robot !is HypnoticAuto.HypnoticAutoRobot) {
            robot.outtake.depositRotation()
        } else {
            if (shouldEnterPreDepositIfAvailable) {
                robot.outtake.autoPreDepositRotation()
            } else {
                robot.outtake.depositRotation()
            }
        }

        robot.outtake.depositCoaxial()

        CompletableFuture.allOf(
            robot.lift.extendToAndStayAt(outtakeLevel.encoderLevel)
        )
    }

    fun initialOuttake(preferredLevel: OuttakeLevel = OuttakeLevel.HighBasket) =
        stateMachineRestrict(
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

    fun outtakeCompleteAndRest(waitPeriod: Long = 200L) = stateMachineRestrict(
        InteractionCompositeState.Outtaking,
        InteractionCompositeState.Rest,
        ignoreInProgress = true
    ) {
        CompletableFuture.runAsync {
            if (robot is HypnoticAuto.HypnoticAutoRobot) {
                robot.outtake.depositRotation()
                Thread.sleep(waitPeriod)
            }

            outtake.openClaw()

            if (robot is HypnoticAuto.HypnoticAutoRobot) {
                Thread.sleep(250L)
            } else {
                Thread.sleep(300L)
            }

            outtake.readyRotation()
            outtake.readyCoaxial().join()

            lift.extendToAndStayAt(0)
                .thenRun {
                    lift.slides.idle()
                }
            CompletableFuture.completedFuture(null)
        }
    }

    fun specimenCompleteAndRest() = stateMachineRestrict(
        InteractionCompositeState.SpecimenReady,
        InteractionCompositeState.Rest,
    ) {
        CompletableFuture.runAsync {
            lift.extendToAndStayAt(0)
                .thenRun {
                    lift.slides.idle()
                }
            Thread.sleep(500L)
            outtake.openClaw()
            Thread.sleep(200L)

            outtake.readyRotation()
            outtake.readyCoaxial()

            CompletableFuture.completedFuture(null)
        }
    }

    fun returnToRestNormal() = stateMachineRestrict(
        InteractionCompositeState.OuttakeReady,
        InteractionCompositeState.Rest,
    ) {
        outtake.openClaw()

        CompletableFuture.runAsync {
            lift.extendToAndStayAt(0)
                .thenRun {
                    lift.slides.idle()
                }
            outtake.readyRotation()
            outtake.readyCoaxial()
            CompletableFuture.completedFuture(null)
        }.thenRunAsync {
            outtake.closeClaw()
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

    fun wallOuttakeToSpecimenReady() =
        stateMachineRestrict(
            InteractionCompositeState.WallIntakeViaOuttake,
            InteractionCompositeState.OuttakeReady
        ) {
            CompletableFuture.allOf(
                robot.outtake.closeClaw(),
                robot.outtake.specimenCoaxial(),
                robot.outtake.specimenRotation()
            ).thenRunAsync {
                extension.extendToAndStayAt(150)
            }
        }

    fun prepareForPickup(
        wristState: WristState = WristState.Lateral,
        doNotUseAutoMode: Boolean = false,
        wideOpen: Boolean = false,
        submersibleOverride: Int? = null,
    ) =
        stateMachineRestrict(InteractionCompositeState.Rest, InteractionCompositeState.Pickup) {
            intakeV4B.v4bUnlock()
                .thenAcceptAsync {
                    extension.extendToAndStayAt(submersibleOverride ?: IntakeConfig.MAX_EXTENSION)
                        .thenAccept {
                            if (robot !is HypnoticAuto.HypnoticAutoRobot) {
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

    fun intakeAndConfirm(
        slowMode: Boolean = false,
        noPick: Boolean = false,
        extras: () -> Unit = {}
    ) =
        stateMachineRestrict(InteractionCompositeState.Pickup, InteractionCompositeState.Confirm) {
            if (slowMode) {
                opMode.robot.intakeV4B.leftRotation.constraints = Constraint.HALF.scale(5.5)
                opMode.robot.intakeV4B.rightRotation.constraints = Constraint.HALF.scale(5.5)
            }

            (if (!noPick) intakeV4B.v4bSamplePickup() else CompletableFuture.completedFuture(null))
                .thenRunAsync {
                    if (!noPick) {
                        Thread.sleep(50L)
                        intake.closeIntake()
                        Thread.sleep(50L)

                        extras()
                    }

                    if (slowMode) {
                        opMode.robot.intakeV4B.leftRotation.constraints =
                            Constraint.HALF.scale(30.5)
                        opMode.robot.intakeV4B.rightRotation.constraints =
                            Constraint.HALF.scale(30.5)
                    }

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

        /*CompletableFuture.allOf(
            outtake.transferRotationForce(),
            outtake.transferCoaxialForce()
        ).join()*/

//        Thread.sleep(100)
        CompletableFuture.allOf(
            intake.openIntake(true)
        )

        Thread.sleep(50)
        outtake.closeClaw()
        Thread.sleep(50)

        CompletableFuture.allOf(
            (if (robot is HypnoticAuto.HypnoticAutoRobot)
                outtake.autoPreDepositRotation() else outtake.depositRotation())
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
                extension.extendToAndStayAt(0)
                    .thenRun {
                        lift.slides.idle()
                    },
                CompletableFuture.runAsync {
                    Thread.sleep(250L)
                    intakeV4B.coaxialRest().join()
                },
                intakeV4B.v4bUnlock()
            ).thenRunAsync {
                outtake.openClaw()
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

    fun waitForState(
        state: InteractionCompositeState,
        timeout: Long = 10000L
    ): Boolean {
        val start = System.currentTimeMillis()
        while (this.state != state) {
            if (System.currentTimeMillis() - start > timeout) {
                return false
            }
            Thread.sleep(50L)
        }

        return true
    }

    fun prepareHangSituation() = stateMachineRestrict(
        InteractionCompositeState.Rest,
        InteractionCompositeState.Hang
    ) {
        intakeV4B.v4bSampleGateway().thenAcceptAsync {
            Thread.sleep(500L)
            extension.extendToAndStayAt(400).join()
        }
    }
}