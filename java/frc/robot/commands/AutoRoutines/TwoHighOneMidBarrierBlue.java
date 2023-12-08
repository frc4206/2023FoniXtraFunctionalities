// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetPipeline;
import frc.robot.commands.Arm.Claw.ClawDownSlow;
import frc.robot.commands.Arm.Claw.ClawInCommand;
import frc.robot.commands.Arm.Claw.ClawOutCommand;
import frc.robot.commands.Combos.ArmLowCube;
import frc.robot.commands.Combos.ArmMiddleCube;
import frc.robot.commands.Combos.ArmTRex;
import frc.robot.commands.Combos.ArmTop;
import frc.robot.commands.Combos.Automation.GrabCubeGround;
import frc.robot.commands.Swerve.PID_DistanceOdometry;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoHighOneMidBarrierBlue extends SequentialCommandGroup {

  public TwoHighOneMidBarrierBlue(Limelight limelight, SwerveSubsystem swerve, ElevatorSubsystem elev, ShoulderSubsystem shoulder, WristSubsystem wrist, ClawSubsystem claw) {
    addCommands(
      // new ArmTop(elev, shoulder, wrist).withTimeout(1),
      // new ClawOutCommand(claw).withTimeout(.1),
      // new ArmTRex(elev, shoulder, wrist).withTimeout(.45),
      // new ParallelCommandGroup(
      //   new PID_DistanceOdometry(swerve, true, true, 6.9, 4.44, 354, 2.5, true, false),
      //   new ArmLowCube(elev, shoulder, wrist),
      //   new ClawInCommand(claw)
      // ).withTimeout(2.45),
      // new ParallelCommandGroup(
      //   new ArmTRex(elev, shoulder, wrist),
      //   new ClawInCommand(claw),
      //   new PID_DistanceOdometry(swerve, true, true, Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.xfinish+0.25, Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.yfinish, 183, 3, true, false)
      // ).withTimeout(2.5),
      // new ArmTop(elev, shoulder, wrist).withTimeout(0.9),
      // new ClawOutCommand(claw).withTimeout(.1),
      // new ArmTRex(elev, shoulder, wrist).withTimeout(.45),
      // new ParallelCommandGroup(
      //   new PID_DistanceOdometry(swerve, true, true, 7.05, 4.45, 270, 2.5, true, false),
      //   new ArmLowCube(elev, shoulder, wrist),
      //   new ClawInCommand(claw)
      // ).withTimeout(2.45),
      // new ParallelCommandGroup(
      //   new PID_DistanceOdometry(swerve, true, true, 7.2875, 2.5, 270, 0.5, true, false),
      //   new ClawInCommand(claw)
      // ).withTimeout(.9),
      // new ParallelCommandGroup(
      //   new ArmTRex(elev, shoulder, wrist),
      //   new ClawInCommand(claw),
      //   new PID_DistanceOdometry(swerve, true, true, 5.7, 5.4, 180, 2.5, true, false)
      // ).withTimeout(1.875),
      // new PID_DistanceOdometry(swerve, true, true, Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.xfinish+0.675, Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.yfinish, 183, 3, true, false).withTimeout(2),
      // new ToggleCoast(swerve).withTimeout(.05),
      // new ArmMiddleCube(elev, shoulder, wrist).withTimeout(0.9),
      // new ClawOutCommand(claw).withTimeout(.1)
      new ArmTop(elev, shoulder, wrist).withTimeout(1.1),
      new ClawOutCommand(claw).withTimeout(.1),
      new ArmTRex(elev, shoulder, wrist).withTimeout(.45),

      new ParallelCommandGroup(
        new ArmLowCube(elev, shoulder, wrist),
        new PID_DistanceOdometry(swerve, true, true, 6.2, 4.45, 345, 2.5, true, false, false, false)
      ).withTimeout(1.5),

      new SetPipeline(limelight),
      new GrabCubeGround(limelight, swerve, elev, shoulder, wrist, claw).withTimeout(0.75),
      new SetPipeline(limelight),

      new ParallelCommandGroup(
        new ArmTRex(elev, shoulder, wrist),
        new ClawInCommand(claw),
        new PID_DistanceOdometry(swerve, true, true, Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.xfinish+0.45, Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.yfinish+0.4, 194, 3, true, false, false, false)
      ).withTimeout(2.5),
      
      new ParallelCommandGroup(
        new PID_DistanceOdometry(swerve, true, true, Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.xfinish+0.385, Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.yfinish+0.15, 186, 3, true, false, false, false).withTimeout(0.375),
        new ArmTop(elev, shoulder, wrist).withTimeout(0.9)
      ).withTimeout(.85),

      new ParallelCommandGroup(
        new ClawDownSlow(claw).withTimeout(.1),
        new ArmTRex(elev, shoulder, wrist).withTimeout(.25)
      ),

      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new PID_DistanceOdometry(swerve, true, true, 6.5, 4.8, 285, 2.5, true, false, false, false).withTimeout(1.6),
          new PID_DistanceOdometry(swerve, true, true, 7.375, 4, 285, 2.5, true, false, false, false).withTimeout(0.5)
        ),
        new ArmLowCube(elev, shoulder, wrist),
        new ClawInCommand(claw)
      ).withTimeout(1.675),

      new SetPipeline(limelight),
      new GrabCubeGround(limelight, swerve, elev, shoulder, wrist, claw).withTimeout(0.675),
      
      new ParallelCommandGroup(
        new SetPipeline(limelight),
        new PID_DistanceOdometry(swerve, true, true, 7.2875, 3.5, 270, 0.5, true, false, false, false),
        new ClawInCommand(claw)
      ).withTimeout(.5),
      new ParallelCommandGroup(
        new ArmTRex(elev, shoulder, wrist),
        new ClawInCommand(claw),
        new PID_DistanceOdometry(swerve, true, true, 5.7, 5.5, 180, 2.5, true, false, false, false)
      ).withTimeout(1.875),

      new ParallelCommandGroup(
        new PID_DistanceOdometry(swerve, true, true, Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.xfinish+0.575, Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.yfinish, 183, 3, true, false, false, false).withTimeout(2),
        new ArmMiddleCube(elev, shoulder, wrist)
      ).withTimeout(2),
      new ClawOutCommand(claw).withTimeout(.1)
    );
  }
}
