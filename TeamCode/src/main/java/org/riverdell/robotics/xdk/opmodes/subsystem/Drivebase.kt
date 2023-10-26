package org.riverdell.robotics.xdk.opmodes.subsystem

import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import io.liftgate.robotics.mono.pipeline.StageContext
import io.liftgate.robotics.mono.subsystem.Subsystem
import org.riverdell.robotics.xdk.opmodes.pipeline.hardware

class Drivebase(private val opMode: LinearOpMode) : Subsystem
{
    private val motors by lazy {
        mutableListOf<DcMotor>(
            opMode.hardware("backLeft"),
            opMode.hardware("backRight"),
            opMode.hardware("frontLeft"),
            opMode.hardware("frontRight")
        )
    }

    private val backingDriveBase by lazy {
        val backLeft = Motor(opMode.hardwareMap, "backLeft")
        val backRight = Motor(opMode.hardwareMap, "backRight")
        val frontLeft = Motor(opMode.hardwareMap, "frontLeft")
        val frontRight = Motor(opMode.hardwareMap, "frontRight")

        MecanumDrive(frontLeft, frontRight, backLeft, backRight)
    }

    override fun composeStageContext() = object : StageContext
    {
        override fun isCompleted() = motors.none { it.isBusy }
    }

    fun driveRobotCentric(driverOp: GamepadEx, scaleFactor: Double)
    {
        backingDriveBase.driveRobotCentric(
            driverOp.leftX * scaleFactor,
            -driverOp.leftY * scaleFactor,
            driverOp.rightX * scaleFactor,
            true
        )
    }

    override fun initialize()
    {
        backingDriveBase
    }

    override fun isCompleted() = motors.none { it.isBusy }

    override fun dispose()
    {
        backingDriveBase.stop()
    }
}