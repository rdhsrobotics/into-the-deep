package org.riverdell.robotics.subsystems

import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.localization.TwoWheelLocalizer
import org.riverdell.robotics.utilities.hardware

class Drivetrain(private val robot: HypnoticRobot) : AbstractSubsystem()
{
    lateinit var frontRight: DcMotorEx
    lateinit var frontLeft: DcMotorEx

    lateinit var backRight: DcMotorEx
    lateinit var backLeft: DcMotorEx

    private lateinit var imu: IMU

    private val imuState by state(write = { _ -> }, read = { imu.robotYawPitchRollAngles })
    private val voltageState by state(write = { _ -> }, read = {
        robot.opMode.hardwareMap.voltageSensor.first().voltage
    })

    val localizer by lazy {
        TwoWheelLocalizer(robot)
    }

    private lateinit var backingDriveBase: MecanumDrive

    fun voltage() = voltageState.current()
    fun imu() = imuState.current()

    fun driveRobotCentric(driverOp: GamepadEx, scaleFactor: Double)
    {
        backingDriveBase.driveRobotCentric(
            -driverOp.leftX * scaleFactor,
            -driverOp.leftY * scaleFactor,
            -driverOp.rightX * scaleFactor,
            true
        )
    }


    override fun start()
    {

    }

    /**
     * Initializes both the IMU and all drivebase motors.
     */
    override fun doInitialize()
    {
        imu = robot.hardware<IMU>("imu")
        imu.initialize(
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
            )
        )
        imu.resetYaw()

        if (robot.opMode is HypnoticAuto)
        {
            frontLeft = robot.opMode.hardware<DcMotorEx>("frontLeft")
            frontRight = robot.opMode.hardware<DcMotorEx>("frontRight")
            backLeft = robot.opMode.hardware<DcMotorEx>("backLeft")
            backRight = robot.opMode.hardware<DcMotorEx>("backRight")

            frontLeft.direction = DcMotorSimple.Direction.FORWARD
            frontLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

            frontRight.direction = DcMotorSimple.Direction.REVERSE
            frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

            backLeft.direction = DcMotorSimple.Direction.REVERSE
            backLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

            backRight.direction = DcMotorSimple.Direction.FORWARD
            backRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

            runWithoutEncoders()
        } else
        {
            setupDriveBase()
        }
    }

    fun setupDriveBase()
    {
        val backLeft = Motor(robot.opMode.hardwareMap, "backLeft")
        val backRight = Motor(robot.opMode.hardwareMap, "backRight")
        val frontLeft = Motor(robot.opMode.hardwareMap, "frontLeft")
        val frontRight = Motor(robot.opMode.hardwareMap, "frontRight")

        backingDriveBase = MecanumDrive(
            frontLeft, frontRight, backLeft, backRight
        )
    }

    fun stopAndResetMotors() = configureMotorsToDo {
        it.power = 0.0
        it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    fun runWithoutEncoders() = configureMotorsToDo {
        it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    private fun configureMotorsToDo(consumer: (DcMotor) -> Unit)
    {
        listOf(backLeft, frontLeft, frontRight, backRight).forEach(consumer::invoke)
    }

    override fun isCompleted() = true
    override fun dispose()
    {
        if (robot.opMode is HypnoticAuto)
        {
            stopAndResetMotors()
        }
    }
}