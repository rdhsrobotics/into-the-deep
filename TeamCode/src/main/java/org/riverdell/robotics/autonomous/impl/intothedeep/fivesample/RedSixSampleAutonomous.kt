package org.riverdell.robotics.autonomous.impl.intothedeep.fivesample

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.riverdell.robotics.autonomous.detection.SampleType

@Autonomous(name = "\uD83D\uDD34 4+2 Basket", group = "Prod")
class RedSixSampleAutonomous : SixSampleAutonomous(SampleType.Red)