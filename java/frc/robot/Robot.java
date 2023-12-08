// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.BatteryScanner;
import frc.robot.Constants.Swerve.Mod0;
import frc.robot.Constants.Swerve.Mod1;
import frc.robot.Constants.Swerve.Mod2;
import frc.robot.Constants.Swerve.Mod3;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  public static CTREConfigs ctreConfigs;
  private RobotContainer m_robotContainer;
  private SwerveSubsystem swerve;
  // private Leds led;
  private String batteryName;
  // private String battFilePath = "/home/lvuser/batts.txt";
  // private int startnumber;
  // private boolean sameBattery = false;
  
  @Override
  public void robotInit() {
    batteryName = BatteryScanner.scanBattery(10);
    System.out.println(batteryName);
    Logger.getInstance().recordMetadata("BatteryName", "BAT-" + batteryName);

    // try (BufferedReader reader = new BufferedReader(new FileReader(battFilePath))) {
    //   String line;
    //   startnumber = 1;
    //   while ((line = reader.readLine()) != null) {
    //     System.out.println(line);
    //     startnumber++;
    //   }
    //   System.out.println("Startnumber: " + startnumber);
    // } catch (IOException e) {
    //   e.printStackTrace();
    //   System.out.println();
    // }

    // try (BufferedWriter writer = new BufferedWriter(new FileWriter(battFilePath))) {
    //   writer.write(startnumber + ": " + batteryName);
    //   writer.newLine();
    //   System.out.println("Battery Name has been written to the file successfully.");
    // } catch (IOException e) {
    //   e.printStackTrace();
    //   System.out.println();
    // }

    // try (BufferedReader reader = new BufferedReader(new FileReader(battFilePath))) {
    //   String line;
    //   int linenumber = 0;
    //   while ((line = reader.readLine()) != null) {
    //     System.out.println(line);
    //     linenumber++;
    //     if ((line.contains(batteryName)) && (startnumber-1 == linenumber)) {
    //       sameBattery = true;
    //       for (int i = 0; i < 10; i++){ 
    //         System.out.println("SAME BATTERY WARNING");
    //       }
    //     }
    //   }
    // } catch (IOException e) {
    //   e.printStackTrace();
    //   System.out.println();
    // }

    setUseTiming(false);
    if (isReal()) {
        Logger.getInstance().addDataReceiver(new NT4Publisher());
        try (PowerDistribution pd = new PowerDistribution(1, ModuleType.kRev)) {
          pd.clearStickyFaults();
        }
    } else {
        String logPath = LogFileUtil.findReplayLog();
        Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }
    Logger.getInstance().start();

    ctreConfigs = new CTREConfigs();
    try {
      m_robotContainer = new RobotContainer();
      swerve = new SwerveSubsystem();
      // led = new Leds();
    } catch (IOException e) {
      e.printStackTrace();
    }

    Mod0.angleOffset = swerve.mSwerveMods[0].getCanCoder().getDegrees();
    Mod1.angleOffset = swerve.mSwerveMods[1].getCanCoder().getDegrees();
    Mod2.angleOffset = swerve.mSwerveMods[2].getCanCoder().getDegrees();
    Mod3.angleOffset = swerve.mSwerveMods[3].getCanCoder().getDegrees();
    for (SwerveModule mod : swerve.mSwerveMods){
      mod.mDriveMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  //C:\Users\Team4206\AppData\Local\Programs\advantagescope
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // if (RobotController.getBatteryVoltage() < 12){
    //   GlobalVariables.batteryLow = true;
    //   led.solid(Section.FULL, Color.kRed);
    // } else {
    //   GlobalVariables.batteryLow = false;
    // }
    // Logger.getInstance().recordOutput("BatteryLow", GlobalVariables.batteryLow);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    double i;
    for (i = 0; i < 10; i+=1){
      System.out.println(GlobalVariables.swerveRuntime);
    }
    for (SwerveModule mod : swerve.mSwerveMods){
      mod.mDriveMotor.setNeutralMode(NeutralMode.Brake);
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    for (SwerveModule mod : swerve.mSwerveMods){
      mod.mDriveMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    for (SwerveModule mod : swerve.mSwerveMods){
      mod.mDriveMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if (GlobalVariables.swerveRuntime >= 600){
    //   Command command = new Stop(m_robotContainer.swerve, m_robotContainer.driver, m_robotContainer.translationAxis, m_robotContainer.strafeAxis, m_robotContainer.rotationAxis, true, true);
    //   command.schedule();
    // }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
