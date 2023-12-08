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
import frc.robot.commands.Arm.Shoulder.ShoulderStopCommand;
import frc.robot.commands.Arm.Wrist.GoToPositionWrist;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmReturn extends SequentialCommandGroup {
  public ArmReturn(ElevatorSubsystem elevator, ShoulderSubsystem shoulder, WristSubsystem wrist) {
    GlobalVariables.elevatordesiredpos = 31*0.6;
    GlobalVariables.shoulderdesiredpos = Constants.Arm.Shoulder.targetPosition0;
    GlobalVariables.wristdesiredpos = 0.3;
    addCommands(
      new ParallelCommandGroup(
        new ElevatorGoToPosition(elevator, 31 *.6).withTimeout(2),
        new GoToPosShoulder(shoulder, Constants.Arm.Shoulder.targetPosition0).withTimeout(1),
        new GoToPositionWrist(wrist, 0.3).withTimeout(1) //0.277
      ).withTimeout(2.5),
      new ShoulderStopCommand(shoulder).withTimeout(0.2)
    );
  }
}