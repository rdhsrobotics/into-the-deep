package org.riverdell.robotics.xdk.opmodes.pipeline

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import io.liftgate.robotics.mono.Mono
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.xdk.opmodes.pipeline.BlueLeft.MoveBackFromSpike
import org.riverdell.robotics.xdk.opmodes.pipeline.BlueLeft.MovePixelToSpike
import org.riverdell.robotics.xdk.opmodes.pipeline.BlueLeft.TurnDegreesTowardBackboard
import org.riverdell.robotics.xdk.opmodes.pipeline.BlueLeft.GoToBackboard
import org.riverdell.robotics.xdk.opmodes.pipeline.BlueLeft.BackUpFromBackboard
import org.riverdell.robotics.xdk.opmodes.pipeline.BlueLeft.StrafeIntoBackboardPosition
import org.riverdell.robotics.xdk.opmodes.pipeline.BlueLeft.ElevateElevatorAtBackboard
import org.riverdell.robotics.xdk.opmodes.pipeline.BlueLeft.StrafeIntoParkingZone
import org.riverdell.robotics.xdk.opmodes.pipeline.contexts.DrivebaseContext
import org.riverdell.robotics.xdk.opmodes.pipeline.detection.TapeSide

/**
 * @author Subham
 * @since 10/23/2023
 */
@Config
object BlueLeft
{
    @JvmField var MovePixelToSpike = 790.0
    @JvmField var MoveBackFromSpike = -300.0
    @JvmField var TurnDegreesTowardBackboard = 90.0
    @JvmField var GoToBackboard = 700.0
    @JvmField var StrafeIntoBackboardPosition = 650.0
    @JvmField var ElevateElevatorAtBackboard = 0.4
    @JvmField var BackUpFromBackboard = -150.0
    // elevator
    @JvmField var StrafeIntoParkingZone = -975.0
}

@Autonomous(name = "Blue | Left", preselectTeleOp = "prod")
class AutoPipelineBlueLeft : AbstractAutoPipeline()
{
    override fun buildExecutionGroup(_OLD_tapeSide: TapeSide) = Mono
        .buildExecutionGroup {
            single<DrivebaseContext>("move pixel to spike") {
                forward(MovePixelToSpike)
            }

            single<DrivebaseContext>("move back from spike") {
                forward(MoveBackFromSpike)
                Thread.sleep(500)
            }

            single<DrivebaseContext>("turn towards backboard") {
                turn(TurnDegreesTowardBackboard)
            }

            single<DrivebaseContext>("go to backboard") {
                forward(GoToBackboard)
            }

            single<DrivebaseContext>("align with backboard") {
                strafe(StrafeIntoBackboardPosition)
            }

            single("elevator") {
                elevatorSubsystem.configureElevatorManually(ElevateElevatorAtBackboard)
            }

            single<DrivebaseContext>("meow") {
                PIDToDistance(12.0)
            }

            single("drop shi") {
                Thread.sleep(350L)
                clawSubsystem.expandClaw(1.0)
                Thread.sleep(1000L)
            }

            single("drop shi2") {
                elevatorSubsystem.configureElevatorManually(0.0)
            }

            single<DrivebaseContext>("meow") {
                forward(BackUpFromBackboard)
            }

            single<DrivebaseContext>("Straf") {
                strafe(StrafeIntoParkingZone)
            }
        }
}