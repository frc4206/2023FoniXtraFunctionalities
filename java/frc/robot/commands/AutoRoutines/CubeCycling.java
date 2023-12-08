// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetPipeline;
import frc.robot.commands.Arm.Claw.ClawInCommand;
import frc.robot.commands.Arm.Claw.ClawOutCommand;
import frc.robot.commands.Combos.ArmLowCone;
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
public class CubeCycling extends SequentialCommandGroup {
  public CubeCycling(Limelight limelight, SwerveSubsystem swerve, ElevatorSubsystem elev, ShoulderSubsystem shoulder, WristSubsystem wrist, ClawSubsystem claw) {
    addCommands(
      new SetPipeline(limelight),
      new PID_DistanceOdometry(swerve, true, true, 12, 5, 90, 2.5, true, false, true, true),
      new GrabCubeGround(limelight, swerve, elev, shoulder, wrist, claw),
      new SetPipeline(limelight),
      new PID_DistanceOdometry(swerve, true, true, 7, 4.96, 175, 2.5, true, false, false, true).withTimeout(0.5),
      new ParallelCommandGroup(
        new PID_DistanceOdometry(swerve, true, true, 2.3, 4.9, 175, 2.5, true, false, false, true),
        new ArmLowCone(elev, shoulder, wrist)
      ).withTimeout(2.5),
      new ClawOutCommand(claw).withTimeout(.1),
      new SetPipeline(limelight),
      new ClawInCommand(claw).withTimeout(0.1),
      new PID_DistanceOdometry(swerve, true, true, 12, 5, 90, 2.5, true, false, true, true),
      new GrabCubeGround(limelight, swerve, elev, shoulder, wrist, claw),
      new SetPipeline(limelight),
      new PID_DistanceOdometry(swerve, true, true, 7, 4.96, 175, 2.5, true, false, false, true).withTimeout(0.5),
      new ParallelCommandGroup(
        new PID_DistanceOdometry(swerve, true, true, 2.65, 4.375, 180, 2.5, true, false, false, true),
        new ArmLowCone(elev, shoulder, wrist)
      ).withTimeout(3),
      new ClawOutCommand(claw).withTimeout(.1)
    );
  }
}
