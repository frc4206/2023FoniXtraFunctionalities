// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Combos.ArmTop;
import frc.robot.commands.Combos.Automation.ConeandExit;
import frc.robot.commands.Combos.Automation.GrabCubeGround;
import frc.robot.commands.Combos.Automation.HighandExit;
import frc.robot.commands.Combos.Automation.LowandExit;
import frc.robot.commands.Combos.Automation.MiddleandExit;
import frc.robot.commands.Combos.Automation.SelectLocation;
import frc.robot.commands.Combos.Automation.SelectPosition;
import frc.robot.commands.Combos.ArmMiddle;
import frc.robot.commands.Combos.ArmMiddleCube;
import frc.robot.commands.Combos.ArmRetrieveCone;
import frc.robot.commands.Combos.ArmCheeseCubeMid;
import frc.robot.commands.Combos.ArmCubeThrow;
import frc.robot.commands.Combos.ArmLowCone;
import frc.robot.commands.Combos.ArmLowCube;
import frc.robot.commands.Combos.ArmTRex;
import frc.robot.commands.Combos.ArmTRexCube;
import frc.robot.commands.Combos.ArmTRexMid;
import frc.robot.commands.SetPipeline;
import frc.robot.commands.Arm.Claw.ClawDownCubeCommand;
import frc.robot.commands.Arm.Claw.ClawInCommand;
import frc.robot.commands.Arm.Claw.ClawOutCommand;
import frc.robot.commands.Arm.Claw.ClawOutFastCommand;
import frc.robot.commands.Arm.Claw.ClawPulseCommand;
import frc.robot.commands.Arm.Elevator.ElevatorDownCommand;
import frc.robot.commands.Arm.Elevator.ElevatorGoToPosition;
import frc.robot.commands.Arm.Elevator.ElevatorUpCommand;
import frc.robot.commands.Arm.Shoulder.GoToPosShoulder;
import frc.robot.commands.Arm.Shoulder.ShoulderDownCommand;
import frc.robot.commands.Arm.Shoulder.ShoulderUpCommand;
import frc.robot.commands.Arm.Wrist.GoToPositionWrist;
import frc.robot.commands.Arm.Wrist.ResetWrist;
import frc.robot.commands.Arm.Wrist.WristDownCommand;
import frc.robot.commands.Arm.Wrist.WristUpCommand;
import frc.robot.commands.AutoRoutines.CubeCycling;
import frc.robot.commands.AutoRoutines.TwoHighOneMidBarrierBlue;
import frc.robot.commands.Swerve.AutoBalanceCloseCommand;
import frc.robot.commands.Swerve.AutoBalanceFarCommand;
import frc.robot.commands.Swerve.CubeLock;
import frc.robot.commands.Swerve.HalfSpeed;
import frc.robot.commands.Swerve.SetHeadingState;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.commands.Swerve.ToggleCoast;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {
  public final static Joystick driver = new Joystick(Constants.driveport);
  private final Joystick operator = new Joystick(Constants.opport);
  private final Joystick buttonBox = new Joystick(Constants.opport2);
  private final Joystick locationSelector = new Joystick(Constants.opport3);
  private final Joystick positionSelector = new Joystick(Constants.opport4);
  private final Joystick operationSelector = new Joystick(Constants.opport5);

  final static SendableChooser<String> autoChooser = new SendableChooser<String>();
  final static SendableChooser<String> exitChooser = new SendableChooser<String>();

  public static final int translationAxis = XboxController.Axis.kLeftY.value;
  public static final int strafeAxis = XboxController.Axis.kLeftX.value;
  public static final int rotationAxis = XboxController.Axis.kRightX.value;

  public final SwerveSubsystem swerve = new SwerveSubsystem();
  public final Limelight limelight = new Limelight();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ShoulderSubsystem shoulder = new ShoulderSubsystem();
  private final WristSubsystem wrist = new WristSubsystem();
  private final ClawSubsystem claw = new ClawSubsystem();
  // public final LEDSubsystem led = new LEDSubsystem();
  // private final Leds leds = new Leds();

  public static Map<String, Command> blankEventmap = new HashMap<>();
  public static Map<String, Command> exitEventmap = new HashMap<>();
  public static Map<String, Command> b1Eventmap = new HashMap<>();
  public static Map<String, Command> b2Eventmap = new HashMap<>();
  public static Map<String, Command> b3Eventmap = new HashMap<>();
  public static Map<String, Command> b4sectionsEventmap = new HashMap<>();
  public static Map<String, Command> m1Eventmap = new HashMap<>();
  public static Map<String, Command> m2Eventmap = new HashMap<>();
  public static Map<String, Command> m3Eventmap = new HashMap<>();
  public static Map<String, Command> w1Eventmap = new HashMap<>();
  public static Map<String, Command> w2Eventmap = new HashMap<>();
  public static Map<String, Command> w3Eventmap = new HashMap<>();
  public static Map<String, Command> w4Eventmap = new HashMap<>();
  public static Map<String, Command> w5Eventmap = new HashMap<>();

  CommandBase closeBalance;
  CommandBase farBalance;

  ParallelRaceGroup highScore;
  ParallelRaceGroup trexStow;
  ParallelRaceGroup midScore;
    
  /** The container for the robot. Contains subsystems, OI devices, and commands. 
   * @throws IOException*/
  public RobotContainer() throws IOException {
    closeBalance = new AutoBalanceCloseCommand(swerve);
    farBalance = new AutoBalanceFarCommand(swerve);

    trexStow = new ArmTRex(elevator, shoulder, wrist).withTimeout(0.5);
    highScore = new ArmTop(elevator, shoulder, wrist).withTimeout(Constants.AutoConstants.armHighDelay);
    midScore = new ArmMiddle(elevator, shoulder, wrist).withTimeout(Constants.AutoConstants.armMidDelay);

    swerve.setDefaultCommand(new TeleopSwerve(swerve, driver, translationAxis, strafeAxis, rotationAxis, true, true));

    exitEventmap.put("low", new ArmLowCube(elevator, shoulder, wrist));
  
    b1Eventmap.put("stow1", new ArmLowCone(elevator, shoulder, wrist));
    b1Eventmap.put("armpickup1", new ArmLowCube(elevator, shoulder, wrist));
    b1Eventmap.put("clawin1", new ClawInCommand(claw));
    b1Eventmap.put("stow2", new ArmLowCone(elevator, shoulder, wrist));
    b1Eventmap.put("armmid1", new ArmCheeseCubeMid(elevator, shoulder, wrist));
    b1Eventmap.put("clawout1", new ClawOutCommand(claw));
    b1Eventmap.put("stow3", new ArmLowCone(elevator, shoulder, wrist));
    b1Eventmap.put("armpickup2", new ArmLowCube(elevator, shoulder, wrist));
    b1Eventmap.put("clawin2", new ClawInCommand(claw));
    b1Eventmap.put("stow4", new ArmLowCone(elevator, shoulder, wrist));
    b1Eventmap.put("armmiddle2", new ArmMiddleCube(elevator, shoulder, wrist));
    b1Eventmap.put("clawout2", new ClawOutCommand(claw));
  
    b2Eventmap.put("pickup", new ArmLowCube(elevator, shoulder, wrist));
    b2Eventmap.put("clawin", new SequentialCommandGroup(new ClawInCommand(claw).withTimeout(3.5), new ClawPulseCommand(claw)));
    b2Eventmap.put("high", new ArmTop(elevator, shoulder, wrist));
    b2Eventmap.put("clawout", new ClawDownCubeCommand(claw).withTimeout(1));

    b3Eventmap.put("pickup", new ArmLowCube(elevator, shoulder, wrist));
    b3Eventmap.put("clawin", new SequentialCommandGroup(new ClawInCommand(claw).withTimeout(3.5), new ClawPulseCommand(claw)));
    b3Eventmap.put("high", new ArmTop(elevator, shoulder, wrist));
    b3Eventmap.put("clawout", new ClawDownCubeCommand(claw).withTimeout(1));
    b3Eventmap.put("trex", new ArmTRex(elevator, shoulder, wrist));
  
    b4sectionsEventmap.put("clawout", new ClawOutCommand(claw));

    w1Eventmap.put("stow1", new ArmLowCone(elevator, shoulder, wrist));
    w1Eventmap.put("armpickup1", new ArmLowCube(elevator, shoulder, wrist));
    w1Eventmap.put("clawin", new ClawInCommand(claw).withTimeout(9));
    w1Eventmap.put("stow2", new ArmLowCone(elevator, shoulder, wrist));
    w1Eventmap.put("armhigh", new ArmTop(elevator, shoulder, wrist));
    w1Eventmap.put("clawout", new ClawOutCommand(claw));
  
    w2Eventmap.put("armpickup", new ArmLowCube(elevator, shoulder, wrist));
    w2Eventmap.put("clawin", new ClawInCommand(claw));
    w2Eventmap.put("armhigh", new ArmTop(elevator, shoulder, wrist));
    w2Eventmap.put("clawout", new ClawOutCommand(claw));
  
    w3Eventmap.put("stow1", new ArmLowCone(elevator, shoulder, wrist));
    w3Eventmap.put("armlow1", new ArmLowCube(elevator, shoulder, wrist));
    w3Eventmap.put("clawin", new ClawInCommand(claw));
    w3Eventmap.put("stow2", new ArmLowCone(elevator, shoulder, wrist));
    w3Eventmap.put("armhigh", new ArmMiddleCube(elevator, shoulder, wrist));
    w3Eventmap.put("clawout", new ClawOutCommand(claw));
    w3Eventmap.put("trex", new ArmTRex(elevator, shoulder, wrist));

    w4Eventmap.put("armpickup", new ArmLowCube(elevator, shoulder, wrist));
    w4Eventmap.put("clawin", new ClawInCommand(claw));
    w4Eventmap.put("armlowcone", new SequentialCommandGroup(new ArmTRexCube(elevator, shoulder, wrist).withTimeout(0.5), new ClawOutCommand(claw)));
    w4Eventmap.put("armpickup2", new ArmLowCube(elevator, shoulder, wrist));
    w4Eventmap.put("clawin2", new ClawInCommand(claw));
    w4Eventmap.put("armmid", new ArmMiddle(elevator, shoulder, wrist));
    w4Eventmap.put("clawout2", new ClawOutCommand(claw));

    w5Eventmap.put("clawout0", new ClawOutCommand(claw));
    w5Eventmap.put("armpickup", new ArmLowCube(elevator, shoulder, wrist));
    w5Eventmap.put("clawin", new ClawInCommand(claw));
    w5Eventmap.put("armlowcone", new ArmTRex(elevator, shoulder, wrist).withTimeout(1));
    w5Eventmap.put("clawout1", new ClawOutCommand(claw));
    w5Eventmap.put("armpickup2", new ArmLowCube(elevator, shoulder, wrist));
    w5Eventmap.put("clawin2", new ClawInCommand(claw));
    w5Eventmap.put("armmid", new ArmTRexMid(elevator, shoulder, wrist));
    w5Eventmap.put("clawout2", new ClawOutCommand(claw));
      
    m1Eventmap.put("stow", new ArmLowCone(elevator, shoulder, wrist));

    m2Eventmap.put("stow", new SequentialCommandGroup(new ArmLowCone(elevator, shoulder, wrist).withTimeout(0.1), new ResetWrist(wrist)));
    m2Eventmap.put("pickup", new ArmLowCube(elevator, shoulder, wrist));
    m2Eventmap.put("clawin", new ClawInCommand(claw).withTimeout(3));
    m2Eventmap.put("stow2", new ArmLowCone(elevator, shoulder, wrist));
    
    m3Eventmap.put("stow", new SequentialCommandGroup(new ArmLowCone(elevator, shoulder, wrist).withTimeout(0.1), new ResetWrist(wrist)));
    
    autoChooser.addOption("B1 Blue", "B1 Blue");
    autoChooser.addOption("B1 Red", "B1 Red");
    autoChooser.addOption("B2 Blue", "B2 Blue");
    autoChooser.addOption("B2 Red", "B2 Red");
    autoChooser.addOption("B3 Blue", "B3 Blue");
    autoChooser.addOption("B3 Red", "B3 Red");
    autoChooser.addOption("B5 Test", "B5 Test");
    autoChooser.addOption("M1 Blue", "M1 Blue");
    autoChooser.addOption("M1 Red", "M1 Red");
    autoChooser.addOption("M2 Blue", "M2 Blue");
    autoChooser.addOption("M2 Red", "M2 Red");
    autoChooser.addOption("M3 Blue", "M3 Blue");
    autoChooser.addOption("M3 Red", "M3 Red");
    autoChooser.addOption("W1 Blue", "W1 Blue");
    autoChooser.addOption("W1 Red", "W1 Red");
    autoChooser.addOption("W2 Blue", "W2 Blue");
    autoChooser.addOption("W2 Red", "W2 Red");
    autoChooser.addOption("W3 Blue", "W3 Blue");
    autoChooser.addOption("W3 Red", "W3 Red");
    autoChooser.addOption("W4 Blue", "W4 Blue");
    autoChooser.addOption("W4 Red", "W4 Red");
    autoChooser.addOption("W5 Blue", "W5 Blue");
    autoChooser.addOption("W5 Red", "W5 Red");

    exitChooser.addOption("B2 Blue Exit", "B2 Blue Exit");
    exitChooser.addOption("B2 Red Exit", "B2 Red Exit");
    exitChooser.addOption("W1 Blue Exit", "W1 Blue Exit");
    exitChooser.addOption("W1 Red Exit", "W1 Red Exit");

    SmartDashboard.putData("Auto Selector", autoChooser);
    SmartDashboard.putData("Exit Chooser", exitChooser);

    configureBindings();
  }

  private void configureBindings() throws IOException {
    //DrIvAr CnTrLs
    new POVButton(driver, 0).onTrue(new ArmTop(elevator, shoulder, wrist));
    new POVButton(driver, 90).onTrue(new ArmMiddle(elevator, shoulder, wrist));
    new POVButton(driver, 180).onTrue(new ArmLowCube(elevator, shoulder, wrist));
    new POVButton(driver, 270).onTrue(new ArmTRex(elevator, shoulder, wrist));
    new JoystickButton(driver, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    new JoystickButton(driver, XboxController.Button.kA.value).toggleOnTrue(new TeleopSwerve(swerve, driver, translationAxis, strafeAxis, rotationAxis, false, true)); //robotRelative
    new JoystickButton(driver, XboxController.Button.kB.value).whileTrue(new ClawInCommand(claw));
    new JoystickButton(driver, XboxController.Button.kY.value).whileTrue(new ClawOutFastCommand(claw));
    new JoystickButton(driver, XboxController.Button.kStart.value).toggleOnTrue(new HalfSpeed(swerve, driver, translationAxis, strafeAxis, rotationAxis, true, true)); //halfSpeed
    new JoystickButton(driver, XboxController.Button.kBack.value).onTrue(new SetHeadingState(swerve));
    new JoystickButton(driver, XboxController.Button.kRightStick.value).onTrue(new ToggleCoast(swerve));
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value).whileTrue(new WristUpCommand(wrist));
    new JoystickButton(driver, XboxController.Button.kRightBumper.value).whileTrue(new WristDownCommand(wrist));


    //OpErAtEr CnTrLs
    new JoystickButton(operator, XboxController.Button.kY.value).whileTrue(new ElevatorUpCommand(elevator));
    new JoystickButton(operator, XboxController.Button.kA.value).whileTrue(new ElevatorDownCommand(elevator));
    new JoystickButton(operator, XboxController.Button.kStart.value).whileTrue(new WristUpCommand(wrist));
    new JoystickButton(operator, XboxController.Button.kBack.value).whileTrue(new WristDownCommand(wrist));
    new JoystickButton(operator, XboxController.Button.kRightBumper.value).whileTrue(new ShoulderUpCommand(shoulder));
    new JoystickButton(operator, XboxController.Button.kLeftBumper.value).whileTrue(new ShoulderDownCommand(shoulder));
    new JoystickButton(operator, XboxController.Button.kRightStick.value).whileTrue(new ElevatorGoToPosition(elevator, 100));
    new JoystickButton(operator, XboxController.Button.kLeftStick.value).whileTrue(new ElevatorGoToPosition(elevator, 25));
    new JoystickButton(operator, XboxController.Button.kX.value).whileTrue(new GoToPosShoulder(shoulder, 0.3));
    new JoystickButton(operator, XboxController.Button.kB.value).whileTrue(new GoToPositionWrist(wrist, 20));
    

    //BuTtOn BoX CnTrLs
    new JoystickButton(buttonBox, 1).onTrue(new ArmLowCone(elevator, shoulder, wrist));
    new JoystickButton(buttonBox, 2).onTrue(new ArmMiddle(elevator, shoulder, wrist));
    new JoystickButton(buttonBox, 3).onTrue(new ArmTop(elevator, shoulder, wrist));
    new JoystickButton(buttonBox, 4).onTrue(new ArmLowCube(elevator, shoulder, wrist));
    // new JoystickButton(buttonBox, 5).whileFalse(new HalfSpeed(swerve, driver, translationAxis, strafeAxis, rotationAxis, true, true));
    new JoystickButton(buttonBox, 5).whileFalse(new CubeLock(swerve, driver, translationAxis, strafeAxis, rotationAxis, true, true));
    new JoystickButton(buttonBox, 6).onTrue(new ArmRetrieveCone(elevator, shoulder, wrist));
    // new JoystickButton(buttonBox, 7).onTrue(new LEDCycleCommand(led));
    new JoystickButton(buttonBox, 8).onTrue(new ArmCubeThrow(elevator, shoulder, wrist, claw));
    new JoystickButton(buttonBox, 9).onTrue(new ArmTRexCube(elevator, shoulder, wrist));
    // new JoystickButton(buttonBox, 10).toggleOnTrue(new HalfSpeed(swerve, driver, translationAxis, strafeAxis, rotationAxis, true, true));;
    new JoystickButton(buttonBox, 10).onTrue(new InstantCommand(() -> swerve.zeroGyro()));


    //location selector controls
    new JoystickButton(locationSelector, 1).onTrue(new SelectLocation(1));
    new JoystickButton(locationSelector, 2).onTrue(new SelectLocation(2));
    new JoystickButton(locationSelector, 3).onTrue(new SelectLocation(3));
    new JoystickButton(locationSelector, 4).onTrue(new SelectLocation(4));
    new JoystickButton(locationSelector, 5).onTrue(new SelectLocation(5));
    new JoystickButton(locationSelector, 6).onTrue(new SelectLocation(6));
    new JoystickButton(locationSelector, 7).onTrue(new SelectLocation(7));
    new JoystickButton(locationSelector, 8).onTrue(new SelectLocation(8));


    //position selector controls
    new JoystickButton(positionSelector, 1).onTrue(new SelectPosition(1));
    new JoystickButton(positionSelector, 2).onTrue(new SelectPosition(2));
    new JoystickButton(positionSelector, 3).onTrue(new SelectPosition(3));


    //operation selector controls
    new JoystickButton(operationSelector, 1).onTrue(new LowandExit(swerve, elevator, shoulder, wrist, claw));
    new JoystickButton(operationSelector, 2).onTrue(new MiddleandExit(swerve, elevator, shoulder, wrist, claw));
    new JoystickButton(operationSelector, 3).onTrue(new HighandExit(swerve, elevator, shoulder, wrist, claw, 2.5, 1.25));
    new JoystickButton(operationSelector, 4).onTrue(new ArmLowCube(elevator, shoulder, wrist));
    new JoystickButton(operationSelector, 5).whileFalse(new CubeLock(swerve, driver, translationAxis, strafeAxis, rotationAxis, false, true));
    new JoystickButton(operationSelector, 6).onTrue(new ConeandExit(swerve, elevator, shoulder, wrist, claw));
    new JoystickButton(operationSelector, 7).onTrue(new SetPipeline(limelight));
    new JoystickButton(operationSelector, 8).onTrue(new GrabCubeGround(limelight, swerve, elevator, shoulder, wrist, claw));
    new JoystickButton(operationSelector, 9).onTrue(new ArmTRexCube(elevator, shoulder, wrist));
    new JoystickButton(operationSelector, 10).onTrue(new CubeCycling(limelight, swerve, elevator, shoulder, wrist, claw));
  }

  public void setRumble(){
    driver.setRumble(RumbleType.kBothRumble, 1);
  }

  public void offRumble(){
    driver.setRumble(RumbleType.kBothRumble, 0);
  }

  public void autoLEDLineup(){
    if (DriverStation.isDisabled()){
      if (RobotContainer.autoChooser.getSelected() == "B2 Blue"){
        if ((swerve.getPose().getX() > (1.6) && swerve.getPose().getX() < (2)) && (swerve.getPose().getY() > (1.8) && swerve.getPose().getY() < (2.2))){
          GlobalVariables.isAligned = true;
        }
        else{
          GlobalVariables.isAligned = false;
        }
      } else if (RobotContainer.autoChooser.getSelected() == "M2 Blue"){
        if ((swerve.getPose().getX() > (1.6) && swerve.getPose().getX() < (2)) && (swerve.getPose().getY() > (1.8) && swerve.getPose().getY() < (2.2))){
          GlobalVariables.isAligned = true;
        }
        else{
          GlobalVariables.isAligned = false;
        }
      }
    } else {
      GlobalVariables.isAligned = false;
    }
  }

  // private CommandBase makeAuto(Map<String, Command> eventmap, String path, double velo, double acceleration){
  //   List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(path, new PathConstraints(velo, acceleration)); 
  //   SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
  //       limelight::getBotPosePose,
  //       swerve::resetOdometry,
  //       Constants.Swerve.swerveKinematics,
  //       new PIDConstants(Constants.AutoConstants.kPController, 0.0, 0.0),
  //       new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0),
  //       swerve::setModuleStates,
  //       eventmap,
  //       false,
  //       swerve
  //   );
  //   return autoBuilder.fullAuto(pathGroup);
  // }

  public Command getAutonomousCommand() {
    // String selectedPath = autoChooser.getSelected();
    // String exitPath = exitChooser.getSelected();
    
    // double autoVelocity = Constants.AutoConstants.AutoSpeeds.mvelo;
    // double autoAcceleration = Constants.AutoConstants.AutoSpeeds.macc;
    // Map<String, Command> autoEventmap = m2Eventmap;
    // Map<String, Command> autoexitEventmap = exitEventmap;
    // CommandBase autoBalancePlan = farBalance;

    /*return new SequentialCommandGroup(
      new ClawInCommand(claw).withTimeout(0.1),
      highScore,
      new ClawOutCommand(claw).withTimeout(0.2),
      makeAuto(autoEventmap, selectedPath, autoVelocity, autoAcceleration),
      autoBalancePlan
      //makeAuto(autoexitEventmap, exitPath, autoVelocity, autoAcceleration)
    ); STILL WORKS JUST DOING MANUAL AUTOS*/
    return new TwoHighOneMidBarrierBlue(limelight, swerve, elevator, shoulder, wrist, claw);
  }
}