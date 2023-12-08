// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combos.Automation;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Combos.ArmLowCube;
import frc.robot.commands.Swerve.CubeLock;
import frc.robot.commands.Swerve.PID_DistanceOdometryCube;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabCubeGround extends SequentialCommandGroup {
  public GrabCubeGround(Limelight limelight, SwerveSubsystem swerve, ElevatorSubsystem elevator, ShoulderSubsystem shoulder, WristSubsystem wrist, ClawSubsystem claw) {
    addCommands(
      new CubeLock(swerve, RobotContainer.driver, RobotContainer.translationAxis, RobotContainer.strafeAxis, RobotContainer.rotationAxis, false, true).withTimeout(2),
      new ParallelCommandGroup(
        new ArmLowCube(elevator, shoulder, wrist),
        new PID_DistanceOdometryCube(limelight, swerve, claw, false, true, 1.25, true)
      ).withTimeout(1.5)
    );
  }
}
