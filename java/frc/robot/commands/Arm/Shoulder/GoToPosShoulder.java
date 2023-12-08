// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.Shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShoulderSubsystem;

public class GoToPosShoulder extends CommandBase {
  private ShoulderSubsystem shoulderSubsystem;
  private double setpoint;
  private double startPoint;
  private boolean isStarted;
  private boolean isFinished;
  private boolean isReversed;
  public GoToPosShoulder(ShoulderSubsystem m_ShoulderSubsystem, double m_setpoint) {
    shoulderSubsystem = m_ShoulderSubsystem;
    setpoint = m_setpoint;
    addRequirements(shoulderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isStarted = false;
    isFinished = false;
    isReversed = false;
    startPoint = shoulderSubsystem.shoulderPosition;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!isStarted) {
      isStarted = shoulderSubsystem.goToPositionShoulder(setpoint);
    } else {
      if (!isReversed) {
        if (Math.abs(shoulderSubsystem.shoulderPosition - setpoint) <= Constants.Arm.Shoulder.allowedErr) {
          isFinished = true;
        }
      } else {
        if (Math.abs(shoulderSubsystem.shoulderPosition - startPoint) <= Constants.Arm.Shoulder.allowedErr) {
          isReversed = false;
          shoulderSubsystem.goToPositionShoulder(setpoint);
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
