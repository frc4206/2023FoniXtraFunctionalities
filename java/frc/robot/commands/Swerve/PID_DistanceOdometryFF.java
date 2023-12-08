  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class PID_DistanceOdometryFF extends CommandBase {
  Limelight Photonvision;
  SwerveSubsystem swerveSubsystem;
  Joystick controller;
  boolean ATfirstDetected;
  boolean isfinished; 

  private double rotation;

  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  
  double StartCommandTime;
  double CurrentTime;
  
  public double lastErrorX = 0;
  public double lastErrorY = 0;
  public double lastErrorYaw = 0;

  public double xSet;
  public double ySet;
  public double yawSet;
  public double timeout;

  boolean useYaw;
  double rAxis;

  double outputYaw;

  public PIDController pidx = new PIDController(Constants.Limelight.XPID.kPslow, 0, Constants.Limelight.XPID.kD);
  public PIDController pidxi = new PIDController(Constants.Limelight.XPID.kPslow, Constants.Limelight.XPID.kIslow, Constants.Limelight.XPID.kDslow);

  public PIDController pidy = new PIDController(Constants.Limelight.YPID.kPslow, 0, Constants.Limelight.YPID.kD);
  public PIDController pidyi = new PIDController(Constants.Limelight.YPID.kPslow, Constants.Limelight.YPID.kIslow, Constants.Limelight.YPID.kDslow);

  public PIDController pidyaw = new PIDController(Constants.Limelight.RPID.kP, Constants.Limelight.RPID.kI, Constants.Limelight.RPID.kD);
  public PIDController pidyawi = new PIDController(Constants.Limelight.RPID.kPi, Constants.Limelight.RPID.kIi, Constants.Limelight.RPID.kDi);

  public PID_DistanceOdometryFF(SwerveSubsystem m_swerveSubsystem, boolean m_fieldRelative, boolean m_openLoop, double m_xSetpoint, double m_ySetpoint, double m_yawSetpoint, double m_timeout, boolean m_useYaw) {
    swerveSubsystem = m_swerveSubsystem;
    addRequirements(swerveSubsystem);
    
    xSet = m_xSetpoint;
    ySet = m_ySetpoint;
    yawSet = m_yawSetpoint;
    timeout = m_timeout;
    useYaw = m_useYaw;

    fieldRelative = m_fieldRelative;
    openLoop = m_openLoop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isfinished = false;
    ATfirstDetected = false;
    GlobalVariables.aprilReliance = true;
    StartCommandTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CurrentTime = Timer.getFPGATimestamp() - StartCommandTime;
    ATfirstDetected = Limelight.ATDetected();
    double[] botposearray = Limelight.getBotPoseAdjusted();
    double m_MaxError = .025;
    double errorX = botposearray[0] - xSet;
    double outputX;
    
    if (Math.abs(errorX) > m_MaxError) {
      outputX = pidx.calculate(botposearray[0], xSet);
    } else {
      outputX = pidxi.calculate(botposearray[0], xSet);
    }

    double errorY = botposearray[1] - ySet;
    double outputY;

    if (Math.abs(errorY) > m_MaxError) {
      outputY = pidy.calculate(botposearray[1], ySet);
    } else {
      outputY = pidyi.calculate(botposearray[1], ySet);
    }

    double errorYaw = botposearray[2] - yawSet;
    if (Math.abs(errorYaw) > 3.5) {
      outputYaw = pidyaw.calculate(botposearray[2], yawSet);
    } else {
      outputYaw = pidyawi.calculate(botposearray[2], yawSet);
    }
    
    double yAxis = outputX;
    double xAxis = outputY;
    if (useYaw){
      rAxis = outputYaw;
    } else {
      rAxis = 0;
    }
        
    translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
    rotation = rAxis * Constants.Swerve.maxAngularVelocity;
    swerveSubsystem.drive(translation, rotation, fieldRelative, openLoop);

    if (Math.abs(errorY) < m_MaxError && Math.abs(errorX) < m_MaxError) {
      isfinished = true;
      isFinished();
    }

    if (CurrentTime > timeout) {
      isfinished = true;
      isFinished();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    GlobalVariables.ATdetect = false;
    GlobalVariables.aprilReliance = false;
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
