// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  CANSparkMax wristMotor = new CANSparkMax(Constants.Arm.Wrist.wristMotorID, MotorType.kBrushless);
  RelativeEncoder wristRelEncoder = wristMotor.getEncoder();
  SparkMaxPIDController wpidController;
  boolean wristSafeToRun = false;
  DigitalInput wristLimSwitch = new DigitalInput(8);

  Mechanism2d wristmechanism;
  MechanismRoot2d wristroot;

  public WristSubsystem() {
    wristMotor.restoreFactoryDefaults();
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setInverted(true);
    wristMotor.setSmartCurrentLimit((int)Constants.Arm.Wrist.ampLimit); //max currrent rating not exceed 60A or 100A more than 2 sec

    wpidController = wristMotor.getPIDController();
    wpidController.setFeedbackDevice(wristRelEncoder);

    wpidController.setP(Constants.Arm.Wrist.kP);
    wpidController.setI(Constants.Arm.Wrist.kI);
    wpidController.setD(Constants.Arm.Wrist.kD);
    wpidController.setIZone(0);
    wpidController.setFF(Constants.Arm.Wrist.kFF);
    wpidController.setOutputRange(-0.5, 0.5);
    wpidController.setSmartMotionMaxVelocity(Constants.Arm.Wrist.maxVelo, Constants.Arm.slotID);
    wpidController.setSmartMotionMinOutputVelocity(Constants.Arm.Wrist.minVelo, Constants.Arm.slotID);
    wpidController.setSmartMotionMaxAccel(Constants.Arm.Wrist.maxAcc, Constants.Arm.slotID);
    wpidController.setSmartMotionAllowedClosedLoopError(Constants.Arm.Wrist.relallowedErr, Constants.Arm.slotID);

    wristmechanism = new Mechanism2d(0.2794, 0.2921);
    wristroot = wristmechanism.getRoot("elevator", 0.2794, 0.2921);
  }

  public void wristUp(){
    wristMotor.set(Constants.Arm.Wrist.manualspeed);
  }

  public void wristDown(){
    if (wristLimSwitch.get()){
      wristMotor.set(-(Constants.Arm.Wrist.manualspeed));
    } else {
      wristMotor.set(0.0);
      wristRelEncoder.setPosition(0);
    }
  }

  public void wristStop(){
    wristMotor.set(0);
  }

  public void resetWrist(){
    while (wristRelEncoder.getPosition() != 0){
      wristDown();
    }
  }

  public void goToZero(){
    double wristPos = wristRelEncoder.getPosition();
    double currPosShoulder = GlobalVariables.shoulderabsEncoder;
    double currPosElevator = GlobalVariables.elevatorenc;

    if (wristPos > Constants.Arm.Wrist.wristlowerlimit) {
      wristSafeToRun = false;
    } else if (wristPos <= Constants.Arm.Wrist.wristlowerlimit) {
      wristSafeToRun = false;
    } else {
      if (currPosElevator >= 30 && currPosShoulder >= .03){
        wristSafeToRun = true;
      } else {
        wristSafeToRun = false;
      }
    }
    if (currPosElevator >= 34 || currPosShoulder > .03){
      wristSafeToRun = true;
    }

    if (wristSafeToRun){
      if (wristLimSwitch.get()){
        wristMotor.set(-(Constants.Arm.Wrist.manualspeed));
      } else {
        wristMotor.set(0);
        wristRelEncoder.setPosition(0);
      }
    }
  }


  public void goToPositionWrist(double setPosition){
    double wristPos = wristRelEncoder.getPosition();
    double currPosShoulder = GlobalVariables.shoulderabsEncoder;
    double currPosElevator = GlobalVariables.elevatorenc;
    
    if (wristPos > Constants.Arm.Wrist.wristupperlimit) {
      wristSafeToRun = false;
    } else if (wristPos <= Constants.Arm.Wrist.wristlowerlimit) {
      wristSafeToRun = false;
    } else {
      if (currPosElevator >= 30 && currPosShoulder >= .03){
        wristSafeToRun = true;
      } else {
        wristSafeToRun = false;
      }
    }

    if (currPosElevator >= 34 || currPosShoulder > .03){
      wristSafeToRun = true;
    }
 
    if (wristSafeToRun) {
      wpidController.setReference(setPosition, CANSparkMax.ControlType.kSmartMotion);
    } else {
      wristMotor.set(0);
    }
  }


  @Override
  public void periodic() {
    GlobalVariables.wristrelEnc = wristRelEncoder.getPosition();
    Logger.getInstance().recordOutput("wrist", wristmechanism);
  }
}
