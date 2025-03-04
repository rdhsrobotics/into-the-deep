package org.riverdell.robotics.teleop.tests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.riverdell.robotics.HypnoticRobotHardware

@TeleOp(
    name = "Sensor Validation",
    group = "Tests"
)
@Disabled
class SensorValidation : LinearOpMode()
{
    override fun runOpMode()
    {
        val hardware = HypnoticRobotHardware(this)
        hardware.initializeHardware()

        waitForStart()

        while (opModeIsActive())
        {
            telemetry.addLine("LIFT Left Position: ${hardware.liftMotorLeft.currentPosition}")
            telemetry.addLine("LIFT Left Power: ${hardware.liftMotorLeft.power}")
            telemetry.addLine("LIFT Left Current: ${hardware.liftMotorLeft.getCurrent(CurrentUnit.AMPS)}")
            telemetry.addLine("--")
            telemetry.addLine("LIFT Right Position: ${hardware.liftMotorRight.currentPosition}")
            telemetry.addLine("LIFT Right Power: ${hardware.liftMotorRight.power}")
            telemetry.addLine("LIFT Right Current: ${hardware.liftMotorRight.getCurrent(CurrentUnit.AMPS)}")
            telemetry.addLine("--")
            telemetry.addLine("Extendo Left Position: ${hardware.extensionMotorLeft.currentPosition}")
            telemetry.addLine("Extendo Right Position: ${hardware.extensionMotorRight.currentPosition}")
            telemetry.update()
            Thread.sleep(50L)
        }
    }
}