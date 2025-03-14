package org.riverdell.robotics.autonomous.impl.intothedeep.sixsample

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.riverdell.robotics.autonomous.detection.SampleType

@Autonomous(name = "\uD83D\uDD35 4+2 Basket", group = "Prod")
class BlueSixSampleAutonomous : SixSampleAutonomous(SampleType.Blue)