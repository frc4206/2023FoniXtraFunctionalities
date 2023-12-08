// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  public CANSparkMax clawMotor = new CANSparkMax(Constants.Arm.Claw.clawMotorID, MotorType.kBrushless); 
  SparkMaxPIDController clawPidController = clawMotor.getPIDController();

  public ClawSubsystem() {
    clawMotor.setSmartCurrentLimit(Constants.Arm.Claw.ampLimit);
  }

  //Default Intake Speed
  public void ClawUp(){
    clawMotor.set(Constants.Arm.Claw.clawupspeed);
  }

  //Running Claw at low speed to keep pieces in intake
  public void PulseClaw(){
    clawMotor.set(Constants.Arm.Claw.clawpulsingspeed);
  }

  //Fast Outtake Speed (Cones)
  public void ClawDownFast(){
    clawMotor.set(Constants.Arm.Claw.clawdownfastspeed);
  }

  //Slow Outtake Speed
  public void clawdownSlow(){
    clawMotor.set(Constants.Arm.Claw.clawdownslowspeed);
  }

  //Default Outtake Speed
  public void ClawDown(){
    clawMotor.set(Constants.Arm.Claw.clawdowndefaultspeed);
  }

  //Cube Outtake Speed
  public void ClawDownCube(){
    clawMotor.set(Constants.Arm.Claw.clawdowncubespeed);
  }

  public void clawStop(){
    clawMotor.set(0);
  }

  @Override
  public void periodic() {
    Logger.getInstance().recordOutput("clawamps", clawMotor.getOutputCurrent());
  }
}
