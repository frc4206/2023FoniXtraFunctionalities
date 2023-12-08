// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combos.Automation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Combos.ArmTRex;
import frc.robot.commands.Combos.ArmTop;
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
public class HighandExit extends SequentialCommandGroup {
  int id = (int)SmartDashboard.getNumber("Location", 6);
  int pos = (int)SmartDashboard.getNumber("Position", 1);
  double[] pose1 = Limelight.getDesiredPositions(id, pos, 1, true);
  double[] pose2 = Limelight.getDesiredPositions(id, pos, 2, true);
  double[] pose3 = Limelight.getDesiredPositions(id, pos, 3, true);
  double[] pose4 = Limelight.getDesiredPositions(id, pos, 4, true);
  public HighandExit(SwerveSubsystem swerve, ElevatorSubsystem elevator, ShoulderSubsystem shoulder, WristSubsystem wrist, ClawSubsystem claw, double timeout1, double timeout2) {
    addCommands(
      new ArmTRex(elevator, shoulder, wrist).withTimeout(0.875),
      new PID_DistanceOdometry(swerve, true, true, pose1[0], pose1[1], pose1[2], timeout1, true, false, false, false),
      // new ArmTRex(elevator, shoulder, wrist),
      new PID_DistanceOdometry(swerve, true, true, pose2[0], pose2[1], pose2[2], timeout1, true, false, false, false),
      new PID_DistanceOdometry(swerve, true, true, pose3[0], pose3[1], pose3[2], timeout1, true, false, false, false),
      new ArmTop(elevator, shoulder, wrist).withTimeout(0.675),
      new PID_DistanceOdometry(swerve, true, true, pose4[0], pose4[1], pose4[2], timeout2, true, false, false, false)
      // new ClawOutCommand(claw).withTimeout(0.2),
      // new ArmTRex(elevator, shoulder, wrist)
    );
  }
}