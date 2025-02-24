package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.navigatePurePursuit
import org.riverdell.robotics.autonomous.movement.navigateToPosition
import org.riverdell.robotics.autonomous.movement.purePursuitNavigateTo
import org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint

// dude u gotta do this stuff not the last day befroec omp bruh
// i change notihg
@Autonomous(name = "Test | Move Forward", group = "Test")
class TestMoveForward : HypnoticAuto({ opMode ->
    navigateToPosition(
        Pose(
            0.0,
            0.0,
            0.0
        )
    )
    navigatePurePursuit(
        FieldWaypoint(
            Pose(0.0, 0.0, 0.degrees),
            15.0
        ),
        FieldWaypoint(
            Pose(0.0, -40.0, 0.degrees),
            15.0
        )
    )

    single("move forward and backward") {
        purePursuitNavigateTo(
            FieldWaypoint(
                Pose(0.0, 0.0, 0.degrees),
                15.0
            ),
            FieldWaypoint(
                Pose(0.0, -40.0, 0.degrees),
                15.0
            ),
        )
    }
})