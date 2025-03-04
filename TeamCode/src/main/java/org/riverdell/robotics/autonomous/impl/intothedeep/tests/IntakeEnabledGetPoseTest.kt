package org.riverdell.robotics.autonomous.impl.intothedeep.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.subsystems.intake.WristState

@Autonomous(name = "Intake Get Pose Test", group = "Test")
class IntakeEnabledGetPoseTest : HypnoticAuto(something@{ opMode ->
    single("hors") {
        opMode.robot.intakeComposite.prepareForPickup(
            WristState.Lateral,
            wideOpen = true,
            doNotUseAutoMode = true,
            submersibleOverride = 300
        ).join()

        opMode.robot.intake.dynamicWrist(0.6)

        //

        while (opMode.opModeIsActive()) {
            opMode.robot.multipleTelemetry.addLine("Pose(${
                -opMode.robot.hardware.pinpoint.position.getX(DistanceUnit.INCH)
            }, ${
                opMode.robot.hardware.pinpoint.position.getY(DistanceUnit.INCH)
            }, ${
                opMode.robot.hardware.pinpoint.position.getHeading(AngleUnit.DEGREES)
            }.degrees)")
            opMode.robot.multipleTelemetry.update()
        }
    }
})