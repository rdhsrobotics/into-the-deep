package org.riverdell.robotics.autonomous.impl.intothedeep

import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.HypnoticAuto.HypnoticAutoRobot
import org.riverdell.robotics.autonomous.movement.geometry.Point
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.subsystems.intake.WristState

fun RootExecutionGroup.visionIntake(opMode: HypnoticAuto) {
    val visionPipeline = (opMode.robot as HypnoticAutoRobot).visionPipeline

    single("Search for sample") {
        opMode.robot.intakeComposite.prepareForPickup(
            WristState.Lateral,
            wideOpen = true
        ).join()

        Thread.sleep(400L)

        var position = 400
        var sample = visionPipeline.detectedSample
        while (sample?.translate == null) {
            sample = visionPipeline.detectedSample
            position -= 100
            if (position <= 0) {
                opMode.robot.intakeComposite.intakeAndConfirm(slowMode = true, noPick = true)
                    .join()
                opMode.robot.intakeComposite.confirmAndTransferAndReady().join()

                return@single
            }

            opMode.robot.extension.extendToAndStayAt(position).join()
            Thread.sleep(500L)
        }
    }

    var rotationAngle = 0.0
    var vector = Point(0.0, 0.0)
    single("Pick up sample") {
        val selection = (opMode.robot as HypnoticAutoRobot)
            .visionPipeline.detectedSample

        val guidanceVector = selection?.translate

        println(guidanceVector)

        if (guidanceVector == null) {
            opMode.robot.intakeComposite.intakeAndConfirm(slowMode = true, noPick = true).join()
            opMode.robot.intakeComposite.confirmAndTransferAndReady().join()

            return@single
        }

        visionPipeline.pause()

        rotationAngle = 0.5 + (selection.angle - 90) / 290.0
        //vector = guidanceVector TODO
    }

    single("Go to sample") {
        val current = opMode.robot.drivetrain.localizer.pose
        val target = Pose(current.x + vector.x, current.y + vector.y, 0.0)
        opMode.robot.hardware.intakeWrist.position = rotationAngle
        navigateTo(target) { withExtendoOut() }
    }

    single("Intake sample") {
        opMode.robot.intakeComposite.intakeAndConfirm(slowMode = true).join()
        opMode.robot.intakeComposite.confirmAndTransferAndReady()
            .thenAcceptAsync {
                opMode.robot.intakeComposite
                    .initialOuttake(org.riverdell.robotics.subsystems.outtake.OuttakeLevel.SomethingLikeThat)
            }
    }
}