// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderSubsystem extends SubsystemBase {
  CANSparkMax shoulderMotor = new CANSparkMax(Constants.Arm.Shoulder.shoulderMotorID, MotorType.kBrushless);
  AbsoluteEncoder shoulderabsEncoder = shoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
  SparkMaxPIDController spidController;

  DigitalInput shoulderlimSwitch = new DigitalInput(Constants.Arm.Shoulder.shoulderlimSwitchPort);

  public double shoulderPosition = shoulderabsEncoder.getPosition();
  double shoulderCurrent = shoulderMotor.getOutputCurrent();

  public static boolean shoulderSafeToRun = true;
  
  boolean init = false;

  Mechanism2d shouldermechanism;
  MechanismRoot2d shoulderroot;

  public ShoulderSubsystem() {
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setIdleMode(IdleMode.kBrake);
    shoulderMotor.setInverted(true);
    shoulderMotor.setSmartCurrentLimit((int)Constants.Arm.Shoulder.ampLimit); //max currrent rating not exceed 60A or 100A more than 2 sec
    
    spidController = shoulderMotor.getPIDController();
    spidController.setFeedbackDevice(shoulderabsEncoder);
    
    spidController.setP(Constants.Arm.Shoulder.kP, Constants.Arm.slotID);
    spidController.setI(Constants.Arm.Shoulder.kI, Constants.Arm.slotID);
    spidController.setD(Constants.Arm.Shoulder.kD, Constants.Arm.slotID);
    spidController.setIZone(0.1, Constants.Arm.slotID);
    spidController.setFF(Constants.Arm.Shoulder.kFF, Constants.Arm.slotID);
    shoulderMotor.setClosedLoopRampRate(Constants.Arm.Shoulder.rampRate);
    spidController.setOutputRange(-0.9, 0.9, Constants.Arm.slotID);

    spidController.setSmartMotionMaxVelocity(Constants.Arm.Shoulder.maxVelo, Constants.Arm.slotID);
    spidController.setSmartMotionMinOutputVelocity(Constants.Arm.Shoulder.minVelo, Constants.Arm.slotID);
    spidController.setSmartMotionMaxAccel(Constants.Arm.Shoulder.maxAcc, Constants.Arm.slotID);
    spidController.setSmartMotionAllowedClosedLoopError(Constants.Arm.Shoulder.allowedErr, Constants.Arm.slotID);

    shouldermechanism = new Mechanism2d(0.508, 0.0762);
    shoulderroot = shouldermechanism.getRoot("elevator", 0.508, 0.0762);
  }

  //Manual Up Movement
  public void shoulderUp(){
    shoulderMotor.set(Constants.Arm.Shoulder.manualspeed);
  }

  //Manual Down Movement
  public void shoulderDown(){
    if (shoulderlimSwitch.get()){
      shoulderMotor.set(-(Constants.Arm.Shoulder.manualspeed));
    } else {
      shoulderMotor.set(0);
    }  
  }

  public void shoulderStop(){
    shoulderMotor.set(0);
  }

  //OUTDATED - TREX FUNCTIONALITY
  public void shoulderGoToZero(){
    double currPosShoulder = shoulderabsEncoder.getPosition();
    double currPosElevator = SmartDashboard.getNumber("Elevator Encoder", 0);

    if (currPosElevator < 30.0 && currPosShoulder <= .05) {
      shoulderSafeToRun = false;
    } else {
      shoulderSafeToRun = true;
    }

    if (shoulderSafeToRun){
      if (shoulderlimSwitch.get()){
        shoulderMotor.set(-0.6);
      } else {
        shoulderMotor.set(0);
      }
    }
  }

  //Using elevator position, determine whether to run shoulder to certain position in encoder
  public boolean goToPositionShoulder(double setPosition){
    Constants.Arm.Shoulder.desiredPos = setPosition;
    double currPosShoulder = shoulderabsEncoder.getPosition();
    double currPosElevator = SmartDashboard.getNumber("Elevator Encoder", 0);
    
    if (currPosElevator < 30.0 && currPosShoulder <= .03) {
      shoulderSafeToRun = false;
    } else {
      shoulderSafeToRun = true;
    }

    if (shoulderSafeToRun) {
      spidController.setReference(setPosition, CANSparkMax.ControlType.kSmartMotion, 0);
      return true;
    } else {
      shoulderMotor.set(0);
      return false;
    }
  }


  @Override
  public void periodic() {
    GlobalVariables.shoulderabsEncoder = shoulderabsEncoder.getPosition();
    Logger.getInstance().recordOutput("shoulder", shouldermechanism);
    Logger.getInstance().recordOutput("shouldervelo", shoulderabsEncoder.getVelocity());
  }
}
