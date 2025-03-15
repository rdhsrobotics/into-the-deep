package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.lockToPosition

@Autonomous(name = "\uD83E\uDD16 Position Lock", group = "Test")
class TestLock : HypnoticAuto({ opMode ->
    lockToPosition(Pose(0.0, 0.0, 0.0.degrees)) { _, _ ->
        false
    }
})