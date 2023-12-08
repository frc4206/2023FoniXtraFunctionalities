// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  // Robot state tracking
  public int loopCycleCount = 0;
  public HPGamePiece hpGamePiece = HPGamePiece.NONE;
  public boolean hpConeTipped = false;
  public boolean hpDoubleSubstation = false;
  public boolean hpThrowGamePiece = false;
  public boolean gripperStopped = false;
  public boolean intakeReady = false;
  public boolean autoScore = false;
  public boolean autoSubstation = false;
  public boolean distraction = false;
  public boolean fallen = false;
  public boolean endgameAlert = false;
  public boolean sameBattery = false;
  public boolean armCoast = false;
  public boolean armEstopped = false;
  public boolean autoFinished = false;
  public double autoFinishedTime = 0.0;
  public boolean lowBatteryAlert = false;
  public boolean demoMode = false;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  // private final Notifier loadingNotifier;

  // Constants
  // private static final int minLoopCycleCount = 10;
  private static final int length = 25;
  private static final int staticLength = 8;
  private static final int staticSectionLength = 2;
  // private static final double strobeFastDuration = 0.1;
  // private static final double strobeSlowDuration = 0.2;
  private static final double breathDuration = 1;
  // private static final double rainbowCycleLength = 25;
  // private static final double rainbowDuration = 0.25;
  private static final double waveExponent = 0.4;
  // private static final double waveFastCycleLength = 25;
  // private static final double waveFastDuration = 0.25;
  // private static final double waveSlowCycleLength = 25;
  // private static final double waveSlowDuration = 3;
  // private static final double waveAllianceCycleLength = 25;
  // private static final double waveAllianceDuration = 2;
  // private static final double autoFadeTime = 2; // 3s nominal
  // private static final double autoFadeMaxTime = 5; // Return to normal

  public Leds() {
    leds = new AddressableLED(8);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    for (int i = 0; i < 25; i++) {
      buffer.setLED(i, Color.kWhite);
    }
    leds.setData(buffer);
    leds.setData(buffer);
    leds.start();
    // loadingNotifier =
    //     new Notifier(
    //         () -> {
    //           synchronized (this) {
    //             breath(
    //                 Section.STATIC_LOW,
    //                 Color.kWhite,
    //                 Color.kBlack,
    //                 0.25,
    //                 System.currentTimeMillis() / 1000.0);
    //             leds.setData(buffer);
    //           }
    //         });
    // loadingNotifier.startPeriodic(0.02);
  }

  @Override
  public void periodic() {
    // solid(Section.FULL, Color.kRed);
    // leds.setData(buffer);
  }

  public void solid(Section section, Color color) {
    if (color != null) {
      for (int i = 0; i < 25; i++) {
        buffer.setLED(i, color);
      }
    }
    leds.setData(buffer);
  }

  public void solid(double percent, Color color) {
    for (int i = 0; i < MathUtil.clamp(length * percent, 0, length); i++) {
      buffer.setLED(i, color);
    }
  }

  public void strobe(Section section, Color color, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(section, on ? color : Color.kBlack);
  }

  public void breath(Section section, Color c1, Color c2, double duration) {
    breath(section, c1, c2, duration, Timer.getFPGATimestamp());
  }

  private void breath(Section section, Color c1, Color c2, double duration, double timestamp) {
    double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(section, new Color(red, green, blue));
  }

  public void rainbow(Section section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (i >= section.start()) {
        buffer.setHSV(i, (int) x, 255, 255);
      }
    }
  }

  public void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      if (i >= section.start()) {
        double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        buffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  public static enum HPGamePiece {
    NONE,
    CUBE,
    CONE
  }

  public static enum Section {
    STATIC,
    SHOULDER,
    FULL,
    STATIC_LOW,
    STATIC_MID,
    STATIC_HIGH;

    public int start() {
      switch (this) {
        case STATIC:
          return 0;
        case SHOULDER:
          return staticLength;
        case FULL:
          return 0;
        case STATIC_LOW:
          return 0;
        case STATIC_MID:
          return staticSectionLength;
        case STATIC_HIGH:
          return staticLength - staticSectionLength;
        default:
          return 0;
      }
    }

    public int end() {
      switch (this) {
        case STATIC:
          return staticLength;
        case SHOULDER:
          return length;
        case FULL:
          return length;
        case STATIC_LOW:
          return staticSectionLength;
        case STATIC_MID:
          return staticLength - staticSectionLength;
        case STATIC_HIGH:
          return staticLength;
        default:
          return length;
      }
    }
  }
}