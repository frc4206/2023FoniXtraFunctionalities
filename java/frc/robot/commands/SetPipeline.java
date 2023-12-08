// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetPipeline extends CommandBase {
  private frc.robot.subsystems.Limelight limelight;
  private boolean isfinished = false;
  public SetPipeline(frc.robot.subsystems.Limelight m_ll) {
    limelight = m_ll;
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.changePipeline();
    isfinished = true;
    isFinished();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isfinished;
  }
}
