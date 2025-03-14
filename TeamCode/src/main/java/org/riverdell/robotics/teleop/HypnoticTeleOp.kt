package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import io.liftgate.robotics.mono.Mono.commands
import io.liftgate.robotics.mono.gamepad.ButtonType
import org.riverdell.robotics.HypnoticOpMode
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.intake.composite.InteractionCompositeState
import org.riverdell.robotics.subsystems.intake.composite.IntakeConfig
import org.riverdell.robotics.subsystems.slides.Extension
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import java.util.concurrent.Executors
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sign

abstract class HypnoticTeleOp(internal val solo: Boolean = false) : HypnoticOpMode() {
    class TeleOpRobot(private val teleOp: HypnoticTeleOp) : HypnoticRobot(teleOp) {
        private val gp1Commands by lazy { commands(teleOp.gamepad1) }
        private val gp2Commands by lazy { commands(if (teleOp.solo) teleOp.gamepad1 else teleOp.gamepad2) }

//        val visionPipeline by lazy { VisionPipeline(teleOp) }

        override fun additionalSubSystems() = listOf(gp1Commands, gp2Commands /*visionPipeline*/)
        override fun initialize() {
//            visionPipeline.sampleDetection.supplyCurrentWristPosition { intake.wrist.unwrapServo().position }
//            visionPipeline.sampleDetection.setDetectionType(SampleType.BLUE)

            multipleTelemetry.addLine("Configured all subsystems. Waiting for start...")
            multipleTelemetry.update()

            while (!teleOp.isStarted) {
                runPeriodics()
            }
        }

        override fun opModeStart() {
            val robotDriver = GamepadEx(teleOp.gamepad1)
            buildCommands()

            multipleTelemetry.addLine("Started!")
            multipleTelemetry.update()

            if (ManagedMotorGroup.keepEncoderPositions) {
                lift.extendToAndStayAt(0)
                    .thenRun {
                        lift.slides.idle()
                    }
                    .join()
            }

            /**
             * Safety feature to prevent samples getting stuck in
             * an unwanted area after autonomous
             */
            val executor = Executors.newSingleThreadScheduledExecutor()
            executor.execute {
                intakeComposite.state = InteractionCompositeState.InProgress

                outtake.depositRotation().join()
                outtake.depositCoaxial().join()

                Thread.sleep(250L)
                outtake.openClaw()
                Thread.sleep(150L)

                outtake.readyCoaxial()
                outtake.readyRotation()
                outtake.closeClaw()

                intakeComposite.state = InteractionCompositeState.Rest
            }

            var loopTime: Long
            while (teleOp.opModeIsActive()) {
                loopTime = System.nanoTime()
                runPeriodics()

                val multiplier =
                    0.5 + if (intakeComposite.state == InteractionCompositeState.Pickup && teleOp.solo)
                        0.0 else (teleOp.gamepad1.right_trigger * 0.5)
                drivetrain.driveRobotCentric(robotDriver, multiplier)

                gp1Commands.run()
                gp2Commands.run()

                if (intakeComposite.state == InteractionCompositeState.Pickup) {
                    val gamepadTarget = if (teleOp.solo) teleOp.gamepad1 else teleOp.gamepad2
                    val wantedPower = -gamepadTarget.left_trigger + gamepadTarget.right_trigger
                    if (wantedPower.absoluteValue > 0.1 && !extension.slides.isTravelling()) {
                        if (wantedPower < 0) {
                            if (extension.slides.currentPosition() < 35) {
                                extension.slides.supplyPowerToAll(0.0)
                            } else {
                                extension.slides.supplyPowerToAll(
                                    (wantedPower.toDouble()
                                        .pow(2) * sign(wantedPower.toDouble())) / 3
                                )
                            }
                        } else {
                            if (extension.slides.currentPosition() >= IntakeConfig.MAX_EXTENSION) {
                                extension.slides.supplyPowerToAll(0.0)
                            } else {
                                extension.slides.supplyPowerToAll(
                                    (wantedPower.toDouble()
                                        .pow(2) * sign(wantedPower.toDouble())) / 3
                                )
                            }
                        }
                    } else if (!extension.slides.isTravelling()) {
                        extension.slides.supplyPowerToAll(0.0)
                    }
                }

                if (intakeComposite.state == InteractionCompositeState.InProgress) {
                    val timeInProgress = System.currentTimeMillis() - intakeComposite.attemptTime
                    if (timeInProgress > 2000L) {
                        if (intakeComposite.attemptedState != null) {
                            intakeComposite.state = intakeComposite.attemptedState!!
                            intakeComposite.attemptTime = System.currentTimeMillis()
                            intakeComposite.attemptedState = null
                        }
                    }
                }

                multipleTelemetry.addData(
                    "Loop Refresh Rate ",
                    1000000000 / (System.nanoTime() - loopTime).toDouble()
                )
                multipleTelemetry.addEssentialLines()
                multipleTelemetry.update()
            }

            executor.shutdownNow()
        }

        private fun buildCommands() {
            if (!teleOp.solo) {
                gp1Commands.apply {
                    where(ButtonType.ButtonX)
                        .onlyWhen {
                            intakeComposite.state == InteractionCompositeState.Rest
                        }
                        .triggers {
                            intakeComposite.prepareTouchHang()
                        }
                        .whenPressedOnce()

                    where(ButtonType.PlayStationLogo)
                        .onlyWhen { intakeComposite.state == InteractionCompositeState.Rest }
                        .triggers {
                            intakeComposite.configureRobotForHang()
                        }
                        .whenPressedOnce()

                    where(ButtonType.ButtonX)
                        .onlyWhen { intakeComposite.state == InteractionCompositeState.HangIdled }
                        .triggers {
                            lift.slides.idle()
                            hang.powered()
                            intakeComposite.state = InteractionCompositeState.HangActivated
                        }
                        .whenPressedOnce()

                    where(ButtonType.ButtonX)
                        .onlyWhen { intakeComposite.state == InteractionCompositeState.HangActivated }
                        .triggers {
                            hang.idle()
                            intakeComposite.state = InteractionCompositeState.HangIdled
                        }
                        .whenPressedOnce()

                    where(ButtonType.ButtonY)
                        .onlyWhen { intakeComposite.state == InteractionCompositeState.HangActivated &&
                                extension.extensionHangState == Extension.ExtensionHangState.Idle }
                        .triggers {
                            extension.retractForHang()
                        }
                        .whenPressedOnce()

                    where(ButtonType.ButtonY)
                        .onlyWhen { intakeComposite.state == InteractionCompositeState.HangActivated &&
                                extension.extensionHangState == Extension.ExtensionHangState.Powered }
                        .triggers {
                            extension.idleForHang()
                        }
                        .whenPressedOnce()
                }
            }

            gp2Commands.apply {
                where(ButtonType.DPadLeft)
                    .onlyWhen { intakeComposite.state == InteractionCompositeState.Pickup }
                    .triggers {
                        intake.perpendicularWrist()
                    }
                    .whenPressedOnce()

                where(ButtonType.DPadRight)
                    .onlyWhen { intakeComposite.state == InteractionCompositeState.Pickup }
                    .triggers {
                        intake.lateralWrist()
                    }
                    .whenPressedOnce()

                where(ButtonType.ButtonB)
                    .onlyWhen {
                        intakeComposite.state == InteractionCompositeState.OuttakeReady &&
                                System.currentTimeMillis() - intakeComposite.lastRetransferInvocation >= 1000L
                    }
                    .triggers {
                        intakeComposite.reTransferOuttakeReady()
                    }
                    .whenPressedOnce()

                where(ButtonType.BumperRight)
                    .onlyWhen { intakeComposite.state == InteractionCompositeState.Pickup }
                    .triggers {
                        intake.dynamicWrist(
                            min(intake.currentDynamicPosition() + 0.02, 1.0)
                        )
                    }
                    .repeatedlyWhilePressed()

                where(ButtonType.BumperLeft)
                    .onlyWhen { intakeComposite.state == InteractionCompositeState.Pickup }
                    .triggers {
                        intake.dynamicWrist(
                            max(intake.currentDynamicPosition() - 0.02, 0.0)
                        )
                    }
                    .repeatedlyWhilePressed()

                where(if (teleOp.solo) ButtonType.BumperLeft else ButtonType.DPadRight)
                    .onlyWhen {
                        intakeComposite.state == InteractionCompositeState.OuttakeReady
                    }
                    .triggers {
                        intakeComposite.outtakeCompleteAndRestFromOuttakeReady()
                    }
                    .whenPressedOnce()

                where(if (teleOp.solo) ButtonType.BumperLeft else ButtonType.DPadRight)
                    .onlyWhen {
                        intakeComposite.state == InteractionCompositeState.Outtaking &&
                                System.currentTimeMillis() - intakeComposite.lastOuttakeBegin >= 750L
                    }
                    .triggers {
                        intakeComposite.lastOuttakeRelease = System.currentTimeMillis()
                        outtake.openClaw()
                    }
                    .andIsHeldUntilReleasedWhere {
                        intakeComposite.outtakeCompleteAndRest(dynamicRelease = true)
                    }

                where(ButtonType.DPadLeft)
                    .onlyWhen { intakeComposite.state == InteractionCompositeState.SpecimenReady }
                    .triggers {
                        lift.extendToAndStayAt(900)
                    }
                    .andIsHeldUntilReleasedWhere {
                        intakeComposite.specimenCompleteAndRest()
                    }

                where(ButtonType.DPadLeft)
                    .onlyWhen {
                        intakeComposite.state == InteractionCompositeState.OuttakeReady
                    }
                    .triggers {
                        intakeComposite.returnToRestNormal()
                    }
                    .whenPressedOnce()

                where(ButtonType.DPadUp)
                    .onlyWhen { intakeComposite.state == InteractionCompositeState.Outtaking }
                    .triggers {
                        intakeComposite.outtakeNext(memory = true)
                    }
                    .whenPressedOnce()

                where(ButtonType.DPadDown)
                    .onlyWhen { intakeComposite.state == InteractionCompositeState.Outtaking }
                    .triggers {
                        intakeComposite.outtakePrevious(memory = true)
                    }
                    .whenPressedOnce()

                where(if (teleOp.solo) ButtonType.BumperRight else ButtonType.DPadUp)
                    .onlyWhen { intakeComposite.state == InteractionCompositeState.OuttakeReady }
                    .triggers {
                        intakeComposite.initialOuttake(preferMemory = true)
                    }
                    .whenPressedOnce()

                where(ButtonType.ButtonX)
                    .onlyWhen {
                        intakeComposite.state != InteractionCompositeState.InProgress &&
                                intakeComposite.state == InteractionCompositeState.Confirm
                    }
                    .triggers {
                        intakeComposite.declineAndIntake()
                    }
                    .whenPressedOnce()

                where(ButtonType.ButtonA)
                    .onlyWhen {
                        intakeComposite.state != InteractionCompositeState.InProgress
                    }
                    .triggers {
                        if (intakeComposite.state == InteractionCompositeState.Rest) {
                            intakeComposite.prepareForPickup()
                        } else {
                            if (intakeComposite.state == InteractionCompositeState.Pickup) {
                                intakeComposite.intakeAndConfirm()
                            } else if (intakeComposite.state == InteractionCompositeState.Confirm) {
                                intakeComposite.confirmAndTransferAndReady()
                            }
                        }
                    }
                    .whenPressedOnce()
            }

            gp1Commands.doButtonUpdatesManually()
            gp2Commands.doButtonUpdatesManually()
        }

        fun motorTest() {
            gp1Commands.where(ButtonType.ButtonA)
                .triggers {
                    hardware.backLeft.power = 1.0
                }
                .andIsHeldUntilReleasedWhere {
                    hardware.backLeft.power = 0.0
                }

            gp1Commands.where(ButtonType.ButtonB)
                .triggers {
                    hardware.backRight.power = 1.0
                }
                .andIsHeldUntilReleasedWhere {
                    hardware.backRight.power = 0.0
                }

            gp1Commands.where(ButtonType.ButtonX)
                .triggers {
                    hardware.frontRight.power = 1.0
                }
                .andIsHeldUntilReleasedWhere {
                    hardware.frontRight.power = 0.0
                }

            gp1Commands.where(ButtonType.ButtonY)
                .triggers {
                    hardware.frontLeft.power = 1.0
                }
                .andIsHeldUntilReleasedWhere {
                    hardware.frontLeft.power = 0.0
                }
        }
    }

    override fun buildRobot() = TeleOpRobot(this)
}