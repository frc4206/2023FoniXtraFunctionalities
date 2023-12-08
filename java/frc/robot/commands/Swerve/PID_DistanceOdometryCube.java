  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class PID_DistanceOdometryCube extends CommandBase {
  Limelight Photonvision;
  SwerveSubsystem swerveSubsystem;
  boolean ATfirstDetected;
  boolean isfinished; 

  private double rotation;

  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  
  double StartCommandTime;
  double CurrentTime;
  
  double currRotation;
  double currHeight;
  double currArea;

  double[] cubecoords;
  double cubeX;
  double cubeY;
  double outputX;
  double outputY;
  public double timeout;

  Limelight ll;

  double rAxis;

  ClawSubsystem claw;

  double xspeed = 0.35;

  double outputYaw;
  boolean timeoutoncube;

  public PIDController pidx = new PIDController(Constants.Limelight.XPID.kPcube, 0, 0);
  public PIDController pidxi = new PIDController(Constants.Limelight.XPID.kPslowcube, Constants.Limelight.XPID.kIi, Constants.Limelight.XPID.kDi);

  public PIDController pidy = new PIDController(Constants.Limelight.YPID.kPcube, 0, 0);
  public PIDController pidyi = new PIDController(Constants.Limelight.YPID.kPslowcube, Constants.Limelight.YPID.kIi, Constants.Limelight.YPID.kDi);

  public PIDController pidyaw = new PIDController(2*Constants.Limelight.RPID.kPcube, Constants.Limelight.RPID.kI, Constants.Limelight.RPID.kD);
  public PIDController pidyawi = new PIDController(Constants.Limelight.RPID.kPcube, Constants.Limelight.RPID.kIi, Constants.Limelight.RPID.kDi);

  public PID_DistanceOdometryCube(Limelight limelight, SwerveSubsystem m_swerveSubsystem, ClawSubsystem m_claw, boolean m_fieldRelative, boolean m_openLoop, double m_timeout, boolean m_timeoutoncube) {
    swerveSubsystem = m_swerveSubsystem;
    ll = limelight;
    claw = m_claw;
    addRequirements(ll);
    addRequirements(swerveSubsystem);
    addRequirements(claw);

    timeout = m_timeout;

    fieldRelative = m_fieldRelative;
    openLoop = m_openLoop;
    timeoutoncube = m_timeoutoncube;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isfinished = false;
    StartCommandTime = Timer.getFPGATimestamp();
    cubecoords = Limelight.getCubeDistance();
    cubeX = cubecoords[0]*Constants.Limelight.cubeFloorMultiplierX;
    cubeY = cubecoords[1]*Constants.Limelight.cubeFloorMultiplierY;
    if (cubeX == 0){
      isfinished = true;
      isFinished();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timeoutoncube){
      if (Limelight.limelighttable.getEntry("ty").getDouble(0.0) < -13){
        xspeed = 0.2;
      } else {
        xspeed = 0.5;
      }

      if (claw.clawMotor.getOutputCurrent() > 36){
        isfinished = true;
        isFinished();
      }
    }

    claw.clawMotor.set(0.85);

    CurrentTime = Timer.getFPGATimestamp() - StartCommandTime;
    double m_MaxError = 0.025;
    double m_MinError = 0.001;
    double errorX = cubeX - 0;
    
    if (Math.abs(errorX) > m_MaxError) {
      outputX = pidx.calculate(cubeX, 0);
    } else if (Math.abs(errorX) < m_MaxError && Math.abs(errorX) > m_MinError){
      outputX = pidxi.calculate(cubeX, 0);
    } else if (Math.abs(errorX) < m_MinError){
      outputX = 0;
    } else {
      outputX = 0;
    }

    double errorY = cubeY - 0;

    if (Math.abs(errorY) > m_MaxError) {
      outputY = pidy.calculate(cubeY, 0);
    } else if (Math.abs(errorY) < m_MaxError && Math.abs(errorY) > m_MinError){
      outputY = pidxi.calculate(cubeY, 0);
    } else if (Math.abs(errorY) < m_MinError){
      outputY = 0;
    } else {
      outputY = 0;
    }
    
    currRotation = Limelight.limelighttable.getEntry("tx").getDouble(0.0);
    currHeight = Limelight.limelighttable.getEntry("ty").getDouble(0.0);
    currArea = Limelight.limelighttable.getEntry("ta").getDouble(0.0);

    double errorYaw = currRotation - 15;
    if (Math.abs(errorYaw) > 0.25) {
      outputYaw = pidyaw.calculate(currRotation, 15);
    } else {
      outputYaw = pidyawi.calculate(currRotation, 15);
    }
  
    rAxis = outputYaw;        
    translation = new Translation2d(xspeed, 0).times(Constants.Swerve.maxSpeed);
    rotation = rAxis * Constants.Swerve.maxAngularVelocity;
    swerveSubsystem.drive(translation, rotation, fieldRelative, openLoop);

    if (Math.abs(errorY) < m_MinError && Math.abs(errorX) < m_MinError) {
      isfinished = true;
      isFinished();
    }

    if (!timeoutoncube){
      if (CurrentTime > timeout) {
        isfinished = true;
        isFinished();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (SwerveModule mod : swerveSubsystem.mSwerveMods){
      mod.mDriveMotor.setNeutralMode(NeutralMode.Brake);
      mod.mAngleMotor.setNeutralMode(NeutralMode.Brake);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    for (SwerveModule mod : swerveSubsystem.mSwerveMods){
      mod.mDriveMotor.setNeutralMode(NeutralMode.Brake);
      mod.mAngleMotor.setNeutralMode(NeutralMode.Brake);
    }
    return isfinished;
  }
}
