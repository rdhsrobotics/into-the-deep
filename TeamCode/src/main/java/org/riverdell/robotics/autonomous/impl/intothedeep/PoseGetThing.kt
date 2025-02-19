package org.riverdell.robotics.autonomous.impl.intothedeep

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.geometry.Point
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.subsystems.intake.WristState

@Autonomous(name = "Pose Getter", group = "Test")
class PoseGetThing : HypnoticAuto(something@{ opMode ->
    visionIntake(opMode)

    /**
     * with((opMode.robot as HypnoticAutoRobot).visionPipeline.sampleDetection)
     *         {
     *             var shouldReverse = false
     *             while (guidanceVector == null) {
     *                 if (!shouldReverse)
     *                 {
     *                     if (opMode.robot.extension.slides.currentPosition() >= 330) {
     *                         shouldReverse = true
     *                     }
     *                 } else
     *                 {
     *                     if (opMode.robot.extension.slides.currentPosition() <= 10) {
     *                         shouldReverse = false
     *                     }
     *                 }
     *
     *                 opMode.robot.extension.slides.supplyPowerToAll((if (shouldReverse) -1.0 else 1.0) * 0.2)
     *             }
     *
     *             opMode.robot.extension.slides.supplyPowerToAll(0.0)
     *         }
     *
     *         var lastPose = opMode.robot.drivetrain.localizer.pose
     *         VisionPositionAction(
     *             { current ->
     *                 val guidance =
     *                     (opMode.robot as HypnoticAutoRobot)
     *                         .visionPipeline
     *                         .sampleDetection.guidanceVector
     *
     *                 if (guidance != null)
     *                 {
     *                     lastPose = current.add(Pose(guidance.x - 0.5, 0.0, 0.0))
     *                 }
     *
     *                 lastPose
     *             },
     *             { current, target ->
     *                 val vector = (opMode.robot as HypnoticAutoRobot)
     *                     .visionPipeline
     *                     .sampleDetection.guidanceVector
     *
     *                 vector != null && vector.x <= 0.01
     *             },
     *             this@something
     *         ).executeBlocking()
     */

    /*single("y") {
        val pipe = (opMode.robot as HypnoticAutoRobot).visionPipeline
        with(pipe) {
            while (sampleDetection.guidanceVector != null)
            {
                if (sampleDetection.guidanceVector!!.y < 0.01)
                {
                    break
                }

                opMode.robot.extension.slides.supplyPowerToAll(
                    -sampleDetection.guidanceVector!!.y
                )
            }
        }

        opMode.robot.extension.slides.supplyPowerToAll(0.0)
    }*/


})