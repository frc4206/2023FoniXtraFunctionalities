// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.commands.Arm.Elevator.ElevatorGoToPosition;
import frc.robot.commands.Arm.Shoulder.GoToPosShoulder;
import frc.robot.commands.Arm.Wrist.GoToPositionWrist;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmRetrieveCone extends SequentialCommandGroup {
  public ArmRetrieveCone(ElevatorSubsystem elevator, ShoulderSubsystem shoulder, WristSubsystem wrist) {
    GlobalVariables.elevatordesiredpos = 117.40*0.6;
    GlobalVariables.shoulderdesiredpos = Constants.Arm.Shoulder.targetPosition5-Constants.Arm.Shoulder.shoulderOffset;
    GlobalVariables.wristdesiredpos = 30;
    addCommands(
      new ParallelCommandGroup(
        new ElevatorGoToPosition(elevator, 117.40 * .6).withTimeout(2),
        new GoToPosShoulder(shoulder, Constants.Arm.Shoulder.targetPosition5-Constants.Arm.Shoulder.shoulderOffset).withTimeout(1),
        new GoToPositionWrist(wrist, 30).withTimeout(1) //0.552
      ).withTimeout(2.5)
    );
  }
}
