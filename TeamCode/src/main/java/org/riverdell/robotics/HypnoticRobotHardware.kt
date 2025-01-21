package org.riverdell.robotics

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.riverdell.robotics.subsystems.outtake.ClawState
import org.riverdell.robotics.subsystems.outtake.PivotState
import org.riverdell.robotics.subsystems.outtake.WristState
import kotlin.math.absoluteValue

class HypnoticRobotHardware(private val opMode: LinearOpMode) {
    lateinit var liftMotorLeft: DcMotorEx
    lateinit var liftMotorRight: DcMotorEx

    lateinit var frontRight: DcMotorEx
    lateinit var frontLeft: DcMotorEx
    lateinit var backRight: DcMotorEx
    lateinit var backLeft: DcMotorEx

    lateinit var imu: IMU

    lateinit var claw: ServoImplEx
    lateinit var wrist: ServoImplEx

    lateinit var pivotRight: ServoImplEx
    lateinit var pivotLeft: ServoImplEx

    fun initializeHardware() {
        imu = opMode.hardwareMap["imu"] as IMU
        imu.initialize(
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
            )
        )
        imu.resetYaw()

        frontLeft = opMode.hardwareMap.get(DcMotorEx::class.java, "frontLeft")
        frontRight = opMode.hardwareMap.get(DcMotorEx::class.java, "frontRight")
        backLeft = opMode.hardwareMap.get(DcMotorEx::class.java, "backLeft")
        backRight = opMode.hardwareMap.get(DcMotorEx::class.java, "backRight")

        liftMotorLeft = opMode.hardwareMap["liftLeft"] as DcMotorEx
        liftMotorLeft.direction = DcMotorSimple.Direction.REVERSE

        liftMotorRight = opMode.hardwareMap["liftRight"] as DcMotorEx
        liftMotorRight.direction = DcMotorSimple.Direction.FORWARD

        var start = System.currentTimeMillis()
        if (HypnoticRobot.resetMode) {
            while (liftMotorLeft.velocity.absoluteValue > 0.1 || System.currentTimeMillis() - start < 500L) {
                liftMotorLeft.power = -0.3
                liftMotorRight.power = -0.3
            }
        }

        liftMotorLeft.power = 0.0
        liftMotorRight.power = 0.0

        liftMotorLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotorRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        claw = opMode.hardwareMap.get(ServoImplEx::class.java, "claw")
        claw.position = ClawState.Closed.position

        wrist = opMode.hardwareMap.get(ServoImplEx::class.java, "wrist")
        wrist.position = WristState.Lateral.position

        pivotRight = opMode.hardwareMap.get(ServoImplEx::class.java, "pivotRight")
        pivotRight.position = PivotState.Initialize.rightPosition

        pivotLeft = opMode.hardwareMap.get(ServoImplEx::class.java, "pivotLeft")
        pivotLeft.position = 1.0 - PivotState.Initialize.rightPosition
    }
}