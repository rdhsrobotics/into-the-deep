package org.riverdell.robotics

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcontroller.internal.localization.GoBildaPinpointDriver
import org.riverdell.robotics.subsystems.intake.IntakeState
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.intake.v4b.CoaxialState
import org.riverdell.robotics.subsystems.intake.v4b.V4BState
import org.riverdell.robotics.subsystems.outtake.OuttakeClawState
import org.riverdell.robotics.subsystems.outtake.OuttakeCoaxialState
import org.riverdell.robotics.subsystems.outtake.OuttakeRotationState
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import kotlin.math.absoluteValue

class HypnoticRobotHardware(private val opMode: LinearOpMode) {
    lateinit var liftMotorLeft: DcMotorEx
    lateinit var liftMotorRight: DcMotorEx

    lateinit var extensionMotorLeft: DcMotorEx
    lateinit var extensionMotorRight: DcMotorEx

    lateinit var frontRight: DcMotorEx
    lateinit var frontLeft: DcMotorEx
    lateinit var backRight: DcMotorEx
    lateinit var backLeft: DcMotorEx

    lateinit var pinpoint: GoBildaPinpointDriver

    lateinit var intakeV4BLeft: ServoImplEx
    lateinit var intakeV4BRight: ServoImplEx
    lateinit var intakeV4BCoaxial: ServoImplEx

    lateinit var intakeWrist: ServoImplEx
    lateinit var intakeClawLeft: ServoImplEx
    lateinit var intakeClawRight: ServoImplEx

    lateinit var outtakeRotationLeft: ServoImplEx
    lateinit var outtakeRotationRight: ServoImplEx
    lateinit var outtakeCoaxial: ServoImplEx
    lateinit var outtakeClaw: ServoImplEx

    lateinit var hangSecondary: CRServoImplEx

    fun initializeHardware() {
        val allHubs = opMode.hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        pinpoint = opMode.hardwareMap.get(
            GoBildaPinpointDriver::class.java,
            "pinpoint"
        )

        frontLeft = opMode.hardwareMap.get(DcMotorEx::class.java, "frontLeft")
        frontRight = opMode.hardwareMap.get(DcMotorEx::class.java, "frontRight")
        backLeft = opMode.hardwareMap.get(DcMotorEx::class.java, "backLeft")
        backRight = opMode.hardwareMap.get(DcMotorEx::class.java, "backRight")

        outtakeRotationRight = opMode.hardwareMap.get(ServoImplEx::class.java, "outtakeRotationRight")
        outtakeRotationRight.position = OuttakeRotationState.Ready.position

        outtakeRotationLeft = opMode.hardwareMap.get(ServoImplEx::class.java, "outtakeRotationLeft")
        outtakeRotationLeft.position = 1.0 - OuttakeRotationState.Ready.position

        outtakeCoaxial = opMode.hardwareMap.get(ServoImplEx::class.java, "outtakeCoaxial")
        outtakeCoaxial.position = OuttakeCoaxialState.Ready.position

        outtakeClaw = opMode.hardwareMap.get(ServoImplEx::class.java, "outtakeClaw")
        outtakeClaw.position = OuttakeClawState.Closed.position

        liftMotorLeft = opMode.hardwareMap["liftLeft"] as DcMotorEx
        liftMotorLeft.direction = DcMotorSimple.Direction.FORWARD

        liftMotorRight = opMode.hardwareMap["liftRight"] as DcMotorEx
        liftMotorRight.direction = DcMotorSimple.Direction.REVERSE

        var start = System.currentTimeMillis()
        if (HypnoticRobot.resetMode) {
            while (liftMotorLeft.velocity.absoluteValue > 0.1 || System.currentTimeMillis() - start < 500L) {
                liftMotorLeft.power = -0.7
                liftMotorRight.power = -0.7
            }
        }

        liftMotorLeft.power = 0.0
        liftMotorRight.power = 0.0

        if (!ManagedMotorGroup.keepEncoderPositions)
        {
            liftMotorLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            liftMotorRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        }

        extensionMotorLeft = opMode.hardwareMap["extendoLeft"] as DcMotorEx
        extensionMotorLeft.direction = DcMotorSimple.Direction.REVERSE

        hangSecondary = opMode.hardwareMap["hangSecondary"] as CRServoImplEx

        extensionMotorRight = opMode.hardwareMap["extendoRight"] as DcMotorEx
        extensionMotorRight.direction = DcMotorSimple.Direction.FORWARD

        intakeV4BLeft = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeV4BLeft")
        intakeV4BRight = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeV4BRight")

        if (HypnoticRobot.resetMode)
        {
            intakeV4BLeft.position = 1.0 - V4BState.UnlockedIdleHover.position
            intakeV4BRight.position = V4BState.UnlockedIdleHover.position

            start = System.currentTimeMillis()
            var hasReset = false

            while (extensionMotorRight.velocity.absoluteValue > 0.1 || System.currentTimeMillis() - start < 1000L) {
                if (System.currentTimeMillis() - start > 500L && !hasReset) {
                    intakeV4BLeft.position = 1.0 - V4BState.Lock.position
                    intakeV4BRight.position = V4BState.Lock.position
                    hasReset = true
                }

                extensionMotorLeft.power = -0.5
                extensionMotorRight.power = -0.5
            }

            extensionMotorLeft.power = 0.0
            extensionMotorRight.power = 0.0
        }

        if (!ManagedMotorGroup.keepEncoderPositions)
        {
            extensionMotorRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            extensionMotorLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        }

        intakeV4BCoaxial = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeV4BCoaxial")
        intakeV4BCoaxial.position = CoaxialState.Rest.position

        intakeWrist = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeWrist")
        intakeWrist.position = WristState.Lateral.position

        intakeClawLeft = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeClawLeft")
        intakeClawLeft.position = IntakeState.Closed.positionLeft

        intakeClawRight = opMode.hardwareMap.get(ServoImplEx::class.java, "intakeClawRight")
        intakeClawRight.position = IntakeState.Closed.positionRight
    }
}