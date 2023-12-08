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

public class PID_Distance extends CommandBase {
  Limelight Photonvision;
  SwerveSubsystem swerveSubsystem;
  Joystick controller;
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

  public double m_kP = Constants.Limelight.XPID.kP;
  public double m_kI = Constants.Limelight.XPID.kI;
  public double m_kD = Constants.Limelight.XPID.kD;
  public double m_YkP = Constants.Limelight.YPID.kP;
  public double m_YkD = Constants.Limelight.YPID.kI;
  public double m_YkI = Constants.Limelight.YPID.kD;
  public double m_YawkP = Constants.Limelight.RPID.kP;
  public double m_YawkI = Constants.Limelight.RPID.kI;
  public double m_YawkD = Constants.Limelight.RPID.kD;
  public PIDController pidx = new PIDController(m_kP, 0, m_kD);
  public PIDController pidxi = new PIDController(m_kP, m_kI, m_kD);

  public PIDController pidy = new PIDController(m_YkP, 0, m_YkD);
  public PIDController pidyi = new PIDController(m_YkP, m_YkI, m_YkD);

  /** Creates a new PID_Distance. */
  public PID_Distance(Limelight m_photonVision, SwerveSubsystem m_swerveSubsystem, Joystick m_controller, int m_translationAxis, int m_strafeAxis, int m_rotationAxis, boolean m_fieldRelative, boolean m_openLoop, double m_xSetpoint, double m_ySetpoint, double m_yawSetpoint, double m_timeout) {
    Photonvision = m_photonVision;
    swerveSubsystem = m_swerveSubsystem;
    addRequirements(Photonvision);
    addRequirements(swerveSubsystem);
    

    xSet = m_xSetpoint;
    ySet = m_ySetpoint;
    yawSet = m_yawSetpoint;
    timeout = m_timeout;

    controller = m_controller;


    fieldRelative = m_fieldRelative;
    openLoop = m_openLoop;
    // Use addRequirements() here to declare subsystem dependencies.
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

    double ySetPoint = 0;
    CurrentTime = Timer.getFPGATimestamp() - StartCommandTime;
    m_kP = Constants.Limelight.XPID.kP;
    m_kI = Constants.Limelight.XPID.kI;
    m_kD = Constants.Limelight.XPID.kD;
    m_YkP = Constants.Limelight.YPID.kP;
    m_YkD = Constants.Limelight.YPID.kI;
    m_YkI = Constants.Limelight.YPID.kD;
    m_YawkP = Constants.Limelight.RPID.kP;
    m_YawkI = Constants.Limelight.RPID.kI;
    m_YawkD = Constants.Limelight.RPID.kD;
    double m_MaxError = .025;
    //double[] robotToCam = {.1143, .00762};

    //0.57
    double errorX = Photonvision.X + 3;


    double outputX;
    
    //= errorX*m_kP + errorDifferenceX*m_kD

    if (Math.abs(errorX) > .15) {
      outputX = pidx.calculate(Photonvision.X, -xSet);
    } else {
      outputX = pidxi.calculate(Photonvision.X, -xSet);
    }

    double errorY = Photonvision.Y - ySetPoint;


    double outputY;

    
    if (Math.abs(errorY) > .15) {
      outputY = pidy.calculate(Photonvision.Y, ySet);
    } else {
      outputY = pidyi.calculate(Photonvision.Y, ySet);
    }
    double errorYaw = Photonvision.Yaw - yawSet;
    double errorDifferenceYaw = errorYaw - lastErrorYaw;
    double errorSumYaw = errorYaw += errorYaw;
    double outputYaw = errorYaw*m_YawkP - errorDifferenceYaw*m_YawkD + errorSumYaw*m_YawkI;
    
    double yAxis = outputX;
    double xAxis = outputY;
    double rAxis = -outputYaw;
    
    //(Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis
    //xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
    //rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;
    
    //translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
    translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
    rotation = rAxis * Constants.Swerve.maxAngularVelocity;
    swerveSubsystem.drive(translation, rotation, fieldRelative, openLoop);

    lastErrorX = errorX;
    lastErrorY = errorY;
    lastErrorYaw = errorYaw;

    //0.05038
    /*if (Math.abs(errorX) < m_MaxError && Math.abs(errorY) < m_MaxError && Math.abs(errorYaw) < m_MaxError) {
      System.out.println("terminated");

      isfinished = true;
      isFinished();
    } */
    

    /* if (Math.abs(errorX) < m_MaxError) {
      System.out.println("terminated");
      isfinished = true;
      isFinished();
    }*/
    if (Math.abs(errorY) < m_MaxError && Math.abs(errorX) < m_MaxError) {
      System.out.println("terminated");
      isfinished = true;
      isFinished();
    }

    if (CurrentTime > timeout) {
      System.out.println("terminated on timeout");
      System.out.println(errorX);
      System.out.println(errorY);
      isfinished = true;
      isFinished();
    }
    
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
