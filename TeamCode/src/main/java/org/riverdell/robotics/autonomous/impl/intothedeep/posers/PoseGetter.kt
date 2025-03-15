package org.riverdell.robotics.autonomous.impl.intothedeep.posers

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel
import kotlin.math.pow
import kotlin.math.round

@Autonomous(name = "\uD83D\uDEE0\uFE0F Prepare Pose", group = "Tests")
class PoseGetter : HypnoticAuto({ opMode ->
    single("Outtake") {
        if (PoserConfig.outtakeEnabled) {
            opMode.robot.intakeComposite
                .initialOuttakeFromRest(OuttakeLevel.HighBasket)
        }
        if (PoserConfig.intakeEnabled) {
            opMode.robot.intakeComposite.prepareForPickup(
                WristState.Lateral,
                wideOpen = true,
                doNotUseAutoMode = true,
                submersibleOverride = PoserConfig.extendoPosition
            ).join()

            opMode.robot.intake.dynamicWrist(PoserConfig.wristPostion)
        }

        while (opMode.opModeIsActive()) {
            opMode.robot.multipleTelemetry.addLine("Pose(${
                -roundPlaces(opMode.robot.hardware.pinpoint.position.getX(DistanceUnit.INCH), 2)
            }, ${
                roundPlaces(opMode.robot.hardware.pinpoint.position.getY(DistanceUnit.INCH), 2)
            }, ${
                roundPlaces(opMode.robot.hardware.pinpoint.position.getHeading(AngleUnit.DEGREES), 2)
            }.degrees)")
            opMode.robot.multipleTelemetry.update()
        }
    }
}) {
    companion object {
        fun roundPlaces(num: Double, places: Int): Double {
            return round(num * 10.0.pow(places)) / (10.0.pow(places))
        }
    }
}