// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class CubeLock extends CommandBase {
  SwerveSubsystem swerveSubsystem;
  boolean isfinished; 

  private double rotation;

  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  
  double StartCommandTime;
  double CurrentTime;
  
  public double lastErrorYaw = 0;

  public double yawSet = 14;

  double rAxis;
  double currRotation = 0;
  double currHeight = 0;
  double currArea = 0;

  double outputYaw;
  int translationAxis;
  int strafeAxis;
  int rotationAxis;

  public PIDController pidyaw = new PIDController(Constants.Limelight.RPID.kP, Constants.Limelight.RPID.kI, Constants.Limelight.RPID.kD);
  public PIDController pidyawi = new PIDController(Constants.Limelight.RPID.kPi, Constants.Limelight.RPID.kIi, Constants.Limelight.RPID.kDi);

  private Joystick controller;

  public CubeLock(SwerveSubsystem m_swerveSubsystem, Joystick m_controller, int m_translationAxis, int m_strafeAxis, int m_rotationAxis, boolean m_fieldRelative, boolean m_openLoop) {
    swerveSubsystem = m_swerveSubsystem;
    addRequirements(swerveSubsystem);
    
    translationAxis = m_translationAxis;
    strafeAxis = m_strafeAxis;
    rotationAxis = m_rotationAxis;
    controller = m_controller;

    fieldRelative = m_fieldRelative;
    openLoop = m_openLoop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isfinished = false;
    StartCommandTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CurrentTime = Timer.getFPGATimestamp() - StartCommandTime;
    currRotation = Limelight.limelighttable.getEntry("tx").getDouble(0.0);
    currHeight = Limelight.limelighttable.getEntry("ty").getDouble(0.0);
    currArea = Limelight.limelighttable.getEntry("ta").getDouble(0.0);

    double errorYaw = currRotation - yawSet;
    if (Math.abs(errorYaw) > 1) {
      outputYaw = pidyaw.calculate(currRotation, yawSet);
    } else {
      outputYaw = pidyawi.calculate(currRotation, yawSet);
    }
    
    if (currArea == 0 || currHeight > 0){
      rAxis = -controller.getRawAxis(rotationAxis)*0.75;
    } else {
      rAxis = outputYaw;
    }

    double yAxis = -controller.getRawAxis(translationAxis)*0.75;
    double xAxis = -controller.getRawAxis(strafeAxis)*0.75;
    
    yAxis = (Math.abs(yAxis) < Constants.stickDeadband) || (Limelight.limelighttable.getEntry("ty").getDouble(0.0) < -13) ? 0 : yAxis;
    xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;

    translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
    rotation = rAxis * Constants.Swerve.maxAngularVelocity;
    swerveSubsystem.drive(translation, rotation, fieldRelative, openLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isfinished;
  }
}
