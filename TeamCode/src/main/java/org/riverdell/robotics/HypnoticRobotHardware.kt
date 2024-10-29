package org.riverdell.robotics

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.ServoImplEx

class HypnoticRobotHardware(private val opMode: HypnoticOpMode)
{
    lateinit var liftMotorLeft: DcMotorEx
    lateinit var liftMotorRight: DcMotorEx

    lateinit var extensionMotorLeft: DcMotorEx
    lateinit var extensionMotorRight: DcMotorEx

    lateinit var frontRight: DcMotorEx
    lateinit var frontLeft: DcMotorEx
    lateinit var backRight: DcMotorEx
    lateinit var backLeft: DcMotorEx

    lateinit var imu: IMU

    lateinit var intakeV4BLeft: ServoImplEx
    lateinit var intakeV4BRight: ServoImplEx
    lateinit var intakeV4BCoaxial: ServoImplEx

    lateinit var intakeWrist: ServoImplEx
    lateinit var intakeClawLeft: ServoImplEx
    lateinit var intakeClawRight: ServoImplEx

    fun initializeHardware()
    {
        imu = opMode.hardwareMap["imu"] as IMU
        imu.initialize(
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
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
        liftMotorLeft.direction = DcMotorSimple.Direction.FORWARD

        extensionMotorLeft = opMode.hardwareMap["extendoLeft"] as DcMotorEx
        extensionMotorLeft.direction = DcMotorSimple.Direction.REVERSE

        extensionMotorRight = opMode.hardwareMap["extendoRight"] as DcMotorEx
        extensionMotorRight.direction = DcMotorSimple.Direction.FORWARD

        intakeV4BLeft = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeV4BLeft")
        intakeV4BRight = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeV4BRight")
        intakeV4BCoaxial = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeV4BCoaxial")

        intakeWrist = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeWrist")
        intakeClawLeft = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeClawLeft")
        intakeClawRight = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeClawRight")
    }
}