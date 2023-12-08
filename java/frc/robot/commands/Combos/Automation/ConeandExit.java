// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combos.Automation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.Claw.ClawInCommand;
import frc.robot.commands.Combos.ArmRetrieveCone;
import frc.robot.commands.Swerve.PID_DistanceOdometryFF;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeandExit extends SequentialCommandGroup {
  public ConeandExit(SwerveSubsystem swerve, ElevatorSubsystem elevator, ShoulderSubsystem shoulder, WristSubsystem wrist, ClawSubsystem claw) {
    // int id = (int)SmartDashboard.getNumber("Location", 1);
    // int pos = (int)SmartDashboard.getNumber("Position", 1);
    // Pose2d pose1 = Limelight.getDesiredPositions(id, pos, 1);
    // Pose2d pose3 = Limelight.getDesiredPositions(id, pos, 3);
    addCommands(
      new PID_DistanceOdometryFF(swerve, true, true, 2.507, 6.791, -180, 3, false).withTimeout(0.675),
      new ArmRetrieveCone(elevator, shoulder, wrist).withTimeout(1),
      new PID_DistanceOdometryFF(swerve, true, true, 1.35, 5.895, -180, 3, false).withTimeout(1),
      new ClawInCommand(claw)
      // new ArmTRex(elevator, shoulder, wrist).withTimeout(0.75),
      // new ArmTRex(elevator, shoulder, wrist)
    );
  }
}