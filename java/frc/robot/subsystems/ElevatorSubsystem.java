// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class ElevatorSubsystem extends SubsystemBase {
  public static CANSparkMax elevatorMotor = new CANSparkMax(Constants.Arm.Elevator.elevatorMotorID, MotorType.kBrushless);
  RelativeEncoder elevatorenc = elevatorMotor.getEncoder();
  SparkMaxPIDController epidController;
  public DigitalInput lowlimSwitch = new DigitalInput(Constants.Arm.Elevator.lowlimSwitchPort);

  boolean elevatorSafeToRun = true;

  Mechanism2d mech;
  public MechanismRoot2d mechRoot;
  MechanismLigament2d elevator;
  MechanismLigament2d shoulder;
  MechanismLigament2d wrist;

  public ElevatorSubsystem() {
    elevatorMotor.restoreFactoryDefaults();
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.setInverted(true);
    elevatorMotor.setSmartCurrentLimit((int)Constants.Arm.Elevator.ampLimit); //max currrent rating not exceed 60A or 100A more than 2 sec

    elevatorenc = elevatorMotor.getEncoder();
    epidController = elevatorMotor.getPIDController();
    epidController.setFeedbackDevice(elevatorenc);

    epidController.setP(Constants.Arm.Elevator.kP);
    epidController.setI(Constants.Arm.Elevator.kI);
    epidController.setD(Constants.Arm.Elevator.kD);
    epidController.setIZone(0);
    epidController.setFF(Constants.Arm.Elevator.kFF);
    epidController.setOutputRange(-1, 1);
    epidController.setSmartMotionMaxVelocity(Constants.Arm.Elevator.maxVelo, Constants.Arm.slotID);
    epidController.setSmartMotionMinOutputVelocity(0, Constants.Arm.slotID);
    epidController.setSmartMotionMaxAccel(Constants.Arm.Elevator.maxAcc, Constants.Arm.slotID);
    epidController.setSmartMotionAllowedClosedLoopError(Constants.Arm.Elevator.allowedErr, Constants.Arm.slotID);
    
    mech = new Mechanism2d(Constants.Arm.AKpanellengths, Constants.Arm.AKpanellengths);
    mechRoot = mech.getRoot("elevator", Constants.Arm.AKrootx, Constants.Arm.AKrooty);

    elevator = mechRoot.append(new MechanismLigament2d(
      "elevator", 
      Constants.Arm.Elevator.AKelevinitlength, 
      Constants.Arm.Elevator.AKelevangle, 
      Constants.Arm.Elevator.AKelevlinewidth, 
      new Color8Bit(Constants.Arm.AKrootred, Constants.Arm.AKrootgreen, Constants.Arm.AKrootblue)
    ));

    shoulder = elevator.append(new MechanismLigament2d(
      "shoulder", 
      Constants.Arm.Shoulder.AKshoulderlength, 
      Constants.Arm.Shoulder.AKshoulderinitangle, 
      Constants.Arm.Shoulder.AKshoulderlinewidth, 
      new Color8Bit(Constants.Arm.AKrootred, Constants.Arm.AKrootgreen, Constants.Arm.AKrootblue)
    ));

    wrist = shoulder.append(new MechanismLigament2d(
      "wrist", 
      Constants.Arm.Wrist.AKwristlength, 
      Constants.Arm.Wrist.AKwristinitangle, 
      Constants.Arm.Wrist.AKwristlinewidth, 
      new Color8Bit(Constants.Arm.AKrootred, Constants.Arm.AKrootgreen, Constants.Arm.AKrootblue)
    ));
  }

  public void logMech(){
    Logger.getInstance().recordOutput("elevenc", GlobalVariables.elevatorenc/80);
    Logger.getInstance().recordOutput("shoulderenec", GlobalVariables.shoulderabsEncoder);
    Logger.getInstance().recordOutput("wristenc", GlobalVariables.wristrelEnc/40);
    Logger.getInstance().recordOutput("elevdesiredpos", GlobalVariables.elevatordesiredpos/80);
    Logger.getInstance().recordOutput("shoulderdesiredpos", GlobalVariables.shoulderdesiredpos);
    Logger.getInstance().recordOutput("wristdesiredpos", GlobalVariables.wristdesiredpos/40);
    elevator.setLength(Constants.Arm.Elevator.AKelevinitlength+GlobalVariables.elevatorenc*Constants.Arm.Elevator.AKelevmult);
    shoulder.setAngle(Constants.Arm.Shoulder.AKshoulderinitangle+(GlobalVariables.shoulderabsEncoder)*Constants.Arm.Shoulder.AKshouldermult);
    wrist.setAngle(Constants.Arm.Wrist.AKwristinitangle-GlobalVariables.wristrelEnc*Constants.Arm.Wrist.AKwristmult);
    Logger.getInstance().recordOutput("mech", mech);
  }

  //Manual Up Command
  public void elevatorUp(){
    elevatorMotor.set(Constants.Arm.Elevator.defaultSpeed);
  }

  //Manual Down Command
  public void elevatorDown(){
    if (lowlimSwitch.get()) {
      elevatorMotor.set(-(Constants.Arm.Elevator.defaultSpeed));
    } else {
      elevatorMotor.set(0);
      elevatorMotor.getEncoder().setPosition(0);
    }
  }

  public void elevatorStop(){
    elevatorMotor.set(0);
  }

  //Automatically Resets Encoder
  public void goToZero() {
    if (lowlimSwitch.get()) {
      elevatorMotor.set(-(Constants.Arm.Elevator.slowSpeed));
    } else {
      elevatorMotor.set(0);
      elevatorMotor.getEncoder().setPosition(0);
    }
  }

  //Sets elevator to go to certain position without restrictions
  public void goToPositionElevatorUR(double setpoint){
    epidController.setReference(setpoint, ControlType.kSmartMotion);
  }

  public boolean elevatorsafetycheck(){
    if (elevatorenc.getPosition() < 20){
      elevatorMotor.set(0.6);
      return false;
    } else {
      return true;
    }
  }

  //Using shoulder position, determine whether elevator can travel to certain position (OUTDATED BUT STILL OPERATIONAL - TREX FUNCTIONALITY)
  public void goToPositionElevator(double setpoint) {   
    if (elevatorenc.getPosition() > setpoint-Constants.Arm.Elevator.allowedErr && elevatorenc.getPosition() < setpoint+Constants.Arm.Elevator.allowedErr){
      elevatorMotor.set(0);
      return;
    }
    
    if (setpoint < 30) {
      if (Constants.Arm.Shoulder.desiredPos < .03) {
        if (SmartDashboard.getNumber("Shoulder Encoder", 0) > .03) {
          if (elevatorenc.getPosition() < 30) {
            elevatorSafeToRun = false;
          } else {
            elevatorSafeToRun = true;
          }
        }
      } else {
        elevatorSafeToRun = true;
      }
    } else {
      elevatorSafeToRun = true;
    }

    if (elevatorSafeToRun) {
      epidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    } else {
      elevatorMotor.set(0);
    }
  }
  
  @Override
  public void periodic() {
    GlobalVariables.elevatorenc = elevatorenc.getPosition();
    logMech();
  }
}
