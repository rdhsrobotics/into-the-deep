package org.riverdell.robotics.autonomous

import io.liftgate.robotics.mono.Mono
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.riverdell.robotics.HypnoticOpMode
import org.riverdell.robotics.autonomous.movement.konfig.NavigationConfig
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.movement.PositionChangeAction
import org.riverdell.robotics.subsystems.intake.composite.InteractionCompositeState
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import kotlin.concurrent.thread

abstract class HypnoticAuto(
    internal val blockExecutionGroup: RootExecutionGroup.(HypnoticAuto) -> Unit
) : HypnoticOpMode()
{
    companion object
    {
        @JvmStatic
        lateinit var instance: HypnoticAuto
    }

   inner class HypnoticAutoRobot : HypnoticRobot(this@HypnoticAuto)
   {
       val navigationConfig = NavigationConfig()
//       val visionPipeline by lazy { VisionPipeline(this@HypnoticAuto) } // TODO: new season

       override fun additionalSubSystems() = listOf<AbstractSubsystem>(/*visionPipeline*/)
       override fun initialize()
       {
           HypnoticAuto.instance = this@HypnoticAuto

           while (opModeInInit())
           {
               runPeriodics()

               imuProxy.allPeriodic()
               drivetrain.localizer.update()

               multipleTelemetry.addLine("--- Initialization ---")

               multipleTelemetry.addLine("Alternative IMU: ${drivetrain.alternativeImu().getYaw(AngleUnit.DEGREES)}")
               multipleTelemetry.addLine("Normal IMU: ${drivetrain.imu().getYaw(AngleUnit.DEGREES)}")

               multipleTelemetry.addData(
                   "Voltage",
                   drivetrain.voltage()
               )
               multipleTelemetry.addData(
                   "Heading",
                   drivetrain.imu().getYaw(AngleUnit.DEGREES)
               )
               multipleTelemetry.addData(
                   "X Position Error",
                   0.0
               )
               multipleTelemetry.addData(
                   "Y Position Error",
                   0.0
               )
               multipleTelemetry.addData(
                   "Pose",
                   drivetrain.localizer.pose
               )

               multipleTelemetry.addData(
                   "H Velocity Error",
                   0.0
               )

               multipleTelemetry.addData(
                   "X Velocity Error",
                   0.0
               )


               multipleTelemetry.addData(
                   "Y Velocity Error",
                   0.0
               )

               multipleTelemetry.addData(
                   "Period",
                   0.0
               )

               multipleTelemetry.update()
           }
       }

       override fun opModeStart()
       {
           thread {
               while (!isStopRequested)
               {
                   imuProxy.allPeriodic()
                   drivetrain.localizer.update()

               }
           }

           thread {
               while (!isStopRequested)
               {
//                   multipleTelemetry.addLine("Alternative IMU: ${drivetrain.alternativeImu().getYaw(AngleUnit.DEGREES)}")
//                   multipleTelemetry.addLine("Normal IMU: ${drivetrain.imu().getYaw(AngleUnit.DEGREES)}")
//
//                   multipleTelemetry.addData(
//                       "Pose",
//                       "Pose(${
//                           "%.2f".format(drivetrain.localizer.pose.x.toFloat())
//                       }, ${
//                           "%.2f".format(drivetrain.localizer.pose.y.toFloat())
//                       }, (${
//                           "%.2f".format(drivetrain.imu().getYaw(AngleUnit.DEGREES).toFloat())
//                       }).degrees)"
//                   )
//
//                   multipleTelemetry.update()
               }
           }

           thread {
               while (!isStopRequested)
               {
                   kotlin.runCatching {
                       runPeriodics()
                   }.onFailure {
                       it.printStackTrace()
                   }
                   /*
*//*
                   if (intakeComposite.state == InteractionCompositeState.InProgress) {
                       val timeInProgress = System.currentTimeMillis() - intakeComposite.attemptTime
                       if (timeInProgress > 7500L) {
                           if (intakeComposite.attemptedState != null) {
                               intakeComposite.state = intakeComposite.attemptedState!!
                               intakeComposite.attemptTime = System.currentTimeMillis()
                               intakeComposite.attemptedState = null
                           }
                       }
                   }*//*

                   multipleTelemetry.addLine("--- Autonomous ---")
                   multipleTelemetry.addData(
                       "Voltage",
                       drivetrain.voltage()
                   )
                   multipleTelemetry.addData(
                       "IMU",
                       drivetrain.imu().getYaw(AngleUnit.DEGREES)
                   )


//                   multipleTelemetry.addData(
//                       "H Velocity Error",
//                       PositionChangeAction.hController.velocityError
//                   )
//
//                   multipleTelemetry.addData(
//                       "Y Velocity Error",
//                       PositionChangeAction.yController.velocityError
//                   )
//
                   kotlin.runCatching {
                       multipleTelemetry.addData(
                           "X Velocity Error",
                           PositionChangeAction.xController.averageVelocity
                       )
                       multipleTelemetry.addData(
                           "Y Velocity Error",
                           PositionChangeAction.yController.averageVelocity
                       )
                   }
//
//

                   multipleTelemetry.addEssentialLines()
                   multipleTelemetry.update()*/
               }
           }

           val executionGroup = Mono.buildExecutionGroup {
               blockExecutionGroup(
                   this@HypnoticAuto
               )
           }

           var operatingThreadFinishedProperly = false
           ManagedMotorGroup.keepEncoderPositions = true

           val operatingThread = thread {
               runCatching {
                   executionGroup.executeBlocking()
               }.onFailure {
                   it.printStackTrace()
               }

               operatingThreadFinishedProperly = true
           }

           while (opModeIsActive())
           {
               if (operatingThreadFinishedProperly)
               {
                   break
               }

               Thread.sleep(50L)
           }

           if (operatingThreadFinishedProperly)
           {
               operatingThread.interrupt()
           }
       }
   }

    override fun buildRobot() = HypnoticAutoRobot()
}
