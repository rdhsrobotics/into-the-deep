package org.riverdell.robotics

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.riverdell.robotics.subsystems.intake.IntakeState
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.intake.v4b.CoaxialState
import org.riverdell.robotics.subsystems.intake.v4b.V4BState
import org.riverdell.robotics.subsystems.outtake.OuttakeClawState
import org.riverdell.robotics.subsystems.outtake.OuttakeRotationState

class HypnoticRobotHardware(private val opMode: LinearOpMode)
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

    lateinit var outtakeRotationLeft: ServoImplEx
    lateinit var outtakeRotationRight: ServoImplEx
    lateinit var outtakeClaw: ServoImplEx

//    lateinit var hangLeft: CRServoImplEx
//    lateinit var hangRight: CRServoImplEx

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
        liftMotorLeft.direction = DcMotorSimple.Direction.FORWARD

        liftMotorRight = opMode.hardwareMap["liftRight"] as DcMotorEx
        liftMotorRight.direction = DcMotorSimple.Direction.REVERSE

        var start = System.currentTimeMillis()
        while (liftMotorLeft.velocity > 0.1 || System.currentTimeMillis() - start < 500L)
        {
            liftMotorLeft.power = -0.8
            liftMotorRight.power = -0.8
        }

        liftMotorLeft.power = 0.0
        liftMotorRight.power = 0.0

        liftMotorLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotorRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        extensionMotorLeft = opMode.hardwareMap["extendoLeft"] as DcMotorEx
        extensionMotorLeft.direction = DcMotorSimple.Direction.REVERSE

        extensionMotorRight = opMode.hardwareMap["extendoRight"] as DcMotorEx
        extensionMotorRight.direction = DcMotorSimple.Direction.FORWARD

        intakeV4BLeft = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeV4BLeft")
        intakeV4BRight = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeV4BRight")

        intakeV4BLeft.position = 1.0 - V4BState.UnlockedIdleHover.position
        intakeV4BRight.position = V4BState.UnlockedIdleHover.position

        start = System.currentTimeMillis()
        while (extensionMotorRight.velocity > 0.1 || System.currentTimeMillis() - start < 500L)
        {
            extensionMotorLeft.power = -0.8
            extensionMotorRight.power = -0.8
        }

        extensionMotorLeft.power = 0.0
        extensionMotorRight.power = 0.0

        extensionMotorRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        extensionMotorLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        intakeV4BLeft.position = 1.0 - V4BState.Lock.position
        intakeV4BRight.position = V4BState.Lock.position

        intakeV4BCoaxial = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeV4BCoaxial")
        intakeV4BCoaxial.position = CoaxialState.Rest.position

        intakeWrist = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeWrist")
        intakeWrist.position = WristState.Lateral.position

        intakeClawLeft = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeClawLeft")
        intakeClawLeft.position = 1.0 - IntakeState.Closed.position

        intakeClawRight = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeClawRight")
        intakeClawRight.position = IntakeState.Closed.position

        outtakeRotationRight = opMode.hardwareMap.get(ServoImplEx::class.java, "outtakeRotationRight")
        outtakeRotationRight.position = OuttakeRotationState.Transfer.position

/*        outtakeRotationLeft = opMode.hardwareMap.get(ServoImplEx::class.java, "outtakeRotationLeft")
        outtakeRotationLeft.position = 1.0 - OuttakeRotationState.Transfer.position*/

        outtakeClaw = opMode.hardwareMap.get(ServoImplEx::class.java, "outtakeClaw")
        outtakeClaw.position = OuttakeClawState.Closed.position

        /*hangLeft = hardwareMap.get(CRServoImplEx::class.java, "hangLeft")
        hangLeft.pwmRange = PwmRange(500.0, 2500.0)

        hangRight = hardwareMap.get(CRServoImplEx::class.java, "hangRight")
        hangRight.pwmRange = PwmRange(500.0, 2500.0)*/
    }
}