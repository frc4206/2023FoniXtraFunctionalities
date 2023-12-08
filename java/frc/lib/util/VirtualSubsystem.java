// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.ArrayList;
import java.util.List;

/** Represents a subsystem unit that requires a periodic callback but not a hardware mutex. */
public abstract class VirtualSubsystem {
  private static List<VirtualSubsystem> subsystems = new ArrayList<>();

  public VirtualSubsystem() {
    subsystems.add(this);
  }

  /** Calls {@link #periodic()} on all virtual subsystems. */
  public static void periodicAll() {
    for (var subsystem : subsystems) {
      subsystem.periodic();
    }
  }

  /** This method is called periodically once per loop cycle. */
  public abstract void periodic();
}