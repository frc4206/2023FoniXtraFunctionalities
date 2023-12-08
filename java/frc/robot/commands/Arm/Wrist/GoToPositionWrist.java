// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class GoToPositionWrist extends CommandBase {
  private WristSubsystem wristSubsystem;
  private double setpoint;
  private boolean isFinished = false;
  public GoToPositionWrist(WristSubsystem m_WristSubsystem, double m_setpoint) {
    wristSubsystem = m_WristSubsystem;
    setpoint = m_setpoint;
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristSubsystem.goToPositionWrist(setpoint);

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
