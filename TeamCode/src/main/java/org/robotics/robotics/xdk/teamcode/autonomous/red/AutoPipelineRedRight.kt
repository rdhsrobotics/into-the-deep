package org.robotics.robotics.xdk.teamcode.autonomous.red

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.robotics.robotics.xdk.teamcode.Global
import org.robotics.robotics.xdk.teamcode.autonomous.AbstractAutoPipeline
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile

/**
 * @author Subham
 * @since 10/23/2023
 */
@Autonomous(
    name = "Red | Player 1",
    group = "Red",
    preselectTeleOp = Global.RobotCentricTeleOpName
)
class AutoPipelineRedRight : AbstractAutoPipeline(AutonomousProfile.RedPlayer1)