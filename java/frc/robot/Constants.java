// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * 
 * <p>PID:
 * kP = Proportional Gain - 
 * kI = Integral - 
 * kD = Derivative - 
 */
public final class Constants {
  public static final double stickDeadband = 0.2;
  public static final double slowstickDeadband = 0.125;
  public static final String Canivore1 = "Canivore1";
  public static final int driveport = 0;
  public static final int opport = 1;
  public static final int opport2 = 2;
  public static final int opport3 = 3;
  public static final int opport4 = 4;
  public static final int opport5 = 5;

  public static final class LEDS{
    public static final int sparkPort = 9;
  }

  public static final class LEDStrip {
    public static final int singleLED = 1;
    public static final int numLEDs = 33;
    public static final int candleID = 60;
  }

  public static final class Limelight{
    public static final double camHeight = 20.5;
    public static final double targetHeight = 12.75;
    public static final double camdegrees = -20;

    public static final double cubeFloorMultiplierX = 2;
    public static final double cubeFloorMultiplierY = 2;

    public static final class XPID{
      public static final double kP = 0.73;
      public static final double kI = 0.67;
      public static final double kD = 0;
      public static final double kPi = 0.4;
      public static final double kIi = 0;
      public static final double kDi = 0;

      public static final double kPslow = 0.35;
      public static final double kIslow = 0;
      public static final double kDslow = 0;

      public static final double kPcube = 0.25;
      public static final double kPslowcube = 0.01;
    }

    public static final class YPID{
      public static final double kP = 0.55;
      public static final double kI = 0.6;
      public static final double kD = 0;
      public static final double kPi = 0.3;
      public static final double kIi = 0;
      public static final double kDi = 0;

      public static final double kPslow = 0.35;
      public static final double kIslow = 0;
      public static final double kDslow = 0;

      public static final double kPcube = 0.1;
      public static final double kPslowcube = 0.01;
    }

    public static final class RPID{
      public static final double kP = 0.004;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kPi = 0.002;
      public static final double kIi = 0;
      public static final double kDi = 0;
      public static final double kPcube = 0.002;
    }

    public static final class Coords{
      // public static final double xOffset = 6.1;
      // public static final double yOffset = 2.3;
      public static final class Red{
        public static final class Score1{
          public static final class ScoreLeft{
            public static final double xinit = 0;
            public static final double yinit = 0;
            public static final double rinit = 0;
            public static final double xmiddle = 0;
            public static final double ymiddle = 0;
            public static final double rmiddle = 0;
            public static final double xfinal = 0;
            public static final double yfinal = 0;
            public static final double rfinal = 0;
            public static final double xfinish = 0;
            public static final double yfinish = 0;
            public static final double rfinish = 0;
          }
          public static final class ScoreMiddle{
            public static final double xinit = 0;
            public static final double yinit = 0;
            public static final double rinit = 0;
            public static final double xmiddle = 0;
            public static final double ymiddle = 0;
            public static final double rmiddle = 0;
            public static final double xfinal = 0;
            public static final double yfinal = 0;
            public static final double rfinal = 0;
            public static final double xfinish = 0;
            public static final double yfinish = 0;
            public static final double rfinish = 0;
          }
          public static final class ScoreRight{
            public static final double xinit = 0;
            public static final double yinit = 0;
            public static final double rinit = 0;
            public static final double xmiddle = 0;
            public static final double ymiddle = 0;
            public static final double rmiddle = 0;
            public static final double xfinal = 0;
            public static final double yfinal = 0;
            public static final double rfinal = 0;
            public static final double xfinish = 0;
            public static final double yfinish = 0;
            public static final double rfinish = 0;
          }
        }
        public static final class Score2{
          public static final class ScoreLeft{
            public static final double xinit = 0;
            public static final double yinit = 0;
            public static final double rinit = 0;
            public static final double xmiddle = 0;
            public static final double ymiddle = 0;
            public static final double rmiddle = 0;
            public static final double xfinal = 0;
            public static final double yfinal = 0;
            public static final double rfinal = 0;
            public static final double xfinish = 0;
            public static final double yfinish = 0;
            public static final double rfinish = 0;
          }
          public static final class ScoreMiddle{
            public static final double xinit = 0;
            public static final double yinit = 0;
            public static final double rinit = 0;
            public static final double xmiddle = 0;
            public static final double ymiddle = 0;
            public static final double rmiddle = 0;
            public static final double xfinal = 0;
            public static final double yfinal = 0;
            public static final double rfinal = 0;
            public static final double xfinish = 0;
            public static final double yfinish = 0;
            public static final double rfinish = 0;
          }
          public static final class ScoreRight{
            public static final double xinit = 0;
            public static final double yinit = 0;
            public static final double rinit = 0;
            public static final double xmiddle = 0;
            public static final double ymiddle = 0;
            public static final double rmiddle = 0;
            public static final double xfinal = 0;
            public static final double yfinal = 0;
            public static final double rfinal = 0;
            public static final double xfinish = 0;
            public static final double yfinish = 0;
            public static final double rfinish = 0;
          }
        }
        public static final class Score3{
          public static final class ScoreLeft{
            public static final double xinit = 0;
            public static final double yinit = 0;
            public static final double rinit = 0;
            public static final double xmiddle = 0;
            public static final double ymiddle = 0;
            public static final double rmiddle = 0;
            public static final double xfinal = 0;
            public static final double yfinal = 0;
            public static final double rfinal = 0;
            public static final double xfinish = 0;
            public static final double yfinish = 0;
            public static final double rfinish = 0;
          }
          public static final class ScoreMiddle{
            public static final double xinit = 0;
            public static final double yinit = 0;
            public static final double rinit = 0;
            public static final double xmiddle = 0;
            public static final double ymiddle = 0;
            public static final double rmiddle = 0;
            public static final double xfinal = 0;
            public static final double yfinal = 0;
            public static final double rfinal = 0;
            public static final double xfinish = 0;
            public static final double yfinish = 0;
            public static final double rfinish = 0;
          }
          public static final class ScoreRight{
            public static final double xinit = 0;
            public static final double yinit = 0;
            public static final double rinit = 0;
            public static final double xmiddle = 0;
            public static final double ymiddle = 0;
            public static final double rmiddle = 0;
            public static final double xfinal = 0;
            public static final double yfinal = 0;
            public static final double rfinal = 0;
            public static final double xfinish = 0;
            public static final double yfinish = 0;
            public static final double rfinish = 0;
          }
        }
        public static final class Pickup4{
          public static final class PickupLeft{
            public static final double xinit = 0;
            public static final double yinit = 0;
            public static final double rinit = 0;
            public static final double xfinal = 0;
            public static final double yfinal = 0;
            public static final double rfinal = 0;
          }
          public static final class PickupRight{
            public static final double xinit = 0;
            public static final double yinit = 0;
            public static final double rinit = 0;
            public static final double xfinal = 0;
            public static final double yfinal = 0;
            public static final double rfinal = 0;
          }
        }
      }
      public static final class Blue{
        public static final class Pickup5{
          public static final class PickupLeft{
            public static final double xinit = 0;
            public static final double yinit = 0;
            public static final double rinit = 180;
            public static final double xfinal = 0;
            public static final double yfinal = 0;
            public static final double rfinal = 180;
            public static final double xfinish = 0;
            public static final double yfinish = 0;
            public static final double rfinish = 180;
          }
          public static final class PickupRight{
            public static final double xinit = 0;
            public static final double yinit = 0;
            public static final double rinit = 180;
            public static final double xfinal = 0;
            public static final double yfinal = 0;
            public static final double rfinal = 180;
            public static final double xfinish = 0;
            public static final double yfinish = 0;
            public static final double rfinish = 180;
          }
        }
        public static final class Score6{
          public static final class ScoreLeft{
            public static final double xinit = 5.2;
            public static final double yinit = 4.775;
            public static final double rinit = 180;
            public static final double xmiddle = 2.05;
            public static final double ymiddle = 4.625;
            public static final double rmiddle = 180;
            public static final double xfinal = 2.1;
            public static final double yfinal = 5.12;
            public static final double rfinal = 177;
            public static final double xfinish = 1.475;
            public static final double yfinish = 5.12;
            public static final double rfinish = 177;
          }
          public static final class ScoreMiddle{
            public static final double xinit = 5.2;
            public static final double yinit = 4.775;
            public static final double rinit = 180;
            public static final double xmiddle = 2.05;
            public static final double ymiddle = 4.625;
            public static final double rmiddle = 180;
            public static final double xfinal = 2.1;
            public static final double yfinal = 4.45;
            public static final double rfinal = 180;
            public static final double xfinish = 1.475;
            public static final double yfinish = 4.39;
            public static final double rfinish = 180;
          }
          public static final class ScoreRight{
            public static final double xinit = 5.2;
            public static final double yinit = 4.775;
            public static final double rinit = 180;
            public static final double xmiddle = 2.05;
            public static final double ymiddle = 4.625;
            public static final double rmiddle = 180;
            public static final double xfinal = 2.1;
            public static final double yfinal = 3.75;
            public static final double rfinal = 170;
            public static final double xfinish = 1.475;
            public static final double yfinish = 3.75;
            public static final double rfinish = 170;
          }
        }
        public static final class Score7{
          public static final class ScoreLeft{
            public static final double xinit = 5.2;
            public static final double yinit = 4.775;
            public static final double rinit = 180;
            public static final double xmiddle = 2.05;
            public static final double ymiddle = 4.625;
            public static final double rmiddle = 180;
            public static final double xfinal = 2.1;
            public static final double yfinal = 3.26;
            public static final double rfinal = 170;
            public static final double xfinish = 1.475;
            public static final double yfinish = 3.26;
            public static final double rfinish = 170;
          }
          public static final class ScoreMiddle{
            public static final double xinit = 5.2;
            public static final double yinit = 4.775;
            public static final double rinit = 180;
            public static final double xmiddle = 2.05;
            public static final double ymiddle = 4.625;
            public static final double rmiddle = 180;
            public static final double xfinal = 2.1;
            public static final double yfinal = 2.73;
            public static final double rfinal = 180;
            public static final double xfinish = 1.475;
            public static final double yfinish = 2.73;
            public static final double rfinish = 180;
          }
          public static final class ScoreRight{
            public static final double xinit = 5.2;
            public static final double yinit = 4.775;
            public static final double rinit = 180;
            public static final double xmiddle = 2.05;
            public static final double ymiddle = 4.625;
            public static final double rmiddle = 180;
            public static final double xfinal = 2.1;
            public static final double yfinal = 2.135;
            public static final double rfinal = 180;
            public static final double xfinish = 1.475;
            public static final double yfinish = 2.135;
            public static final double rfinish = 180;
          }
        }
        public static final class Score8{
          public static final class ScoreLeft{
            public static final double xinit = 5.2;
            public static final double yinit = 4.775;
            public static final double rinit = 180;
            public static final double xmiddle = 2.05;
            public static final double ymiddle = 4.625;
            public static final double rmiddle = 180;
            public static final double xfinal = 2.1;
            public static final double yfinal = 1.55;
            public static final double rfinal = 174;
            public static final double xfinish = 1.475;
            public static final double yfinish = 1.55;
            public static final double rfinish = 174;
          }
          public static final class ScoreMiddle{
            public static final double xinit = 5.2;
            public static final double yinit = 4.775;
            public static final double rinit = 180;
            public static final double xmiddle = 2.05;
            public static final double ymiddle = 4.625;
            public static final double rmiddle = 180;
            public static final double xfinal = 2.1;
            public static final double yfinal = 1;
            public static final double rfinal = 180;
            public static final double xfinish = 1.475;
            public static final double yfinish = 1;
            public static final double rfinish = 180;
          }
          public static final class ScoreRight{
            public static final double xinit = 5.2;
            public static final double yinit = 4.775;
            public static final double rinit = 175;
            public static final double xmiddle = 2.05;
            public static final double ymiddle = 4.625;
            public static final double rmiddle = 180;
            public static final double xfinal = 2.1;
            public static final double yfinal = 0.332;
            public static final double rfinal = 180;
            public static final double xfinish = 1.475;
            public static final double yfinish = 0.347;
            public static final double rfinish = 180;
          }
        }
      }
    }
    public static final double xOffSet = -8.275;
    public static final double yOffSet = -4.35;
  }


  public static final class Arm{
    public static final int slotID = 0;
    public static final double AKpanellengths = 1.75;
    public static final double AKrootx = 0.1016;
    public static final double AKrooty = 0.2413;
    public static final int AKrootred = 235;
    public static final int AKrootgreen = 157;
    public static final int AKrootblue = 52;
    public static final class Elevator{
      public static final int elevatorMotorID = 14;
      public static final int lowlimSwitchPort = 7;
  
      public static final double kP = 0.0005;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kFF = 0.000156;

      public static final double ampLimit = 60;
      public static final double maxVelo = 5676;
      public static final double maxAcc = 56760;
      public static final double allowedErr = 2;

      public static final double slowSpeed = 0.4;
      public static final double defaultSpeed = 0.8;
      
      public static final double AKelevangle = 45;
      public static final int AKelevlinewidth = 15;
      public static final double AKelevinitlength = 0.66;
      public static final double AKelevmult = 0.007915;
    }

    public static final class Shoulder {
      public static final int shoulderMotorID = 15;
      public static final int shoulderlimSwitchPort = 6;
  
      public static final double shoulderOffset = 0.127;

      public static final double kP = 0.0008;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kFF = 0.000025;
      
      public static final double ampLimit = 60;
      public static final double maxVelo = 11352;
      public static final double minVelo = 0;
      public static final double maxAcc = (8514*9)/8;
      public static final double allowedErr = 0.001;
      public static final double rampRate = 0.4;

      public static final double manualspeed = 0.45;

      public static double desiredPos = 0.1;
      public static final double targetPosition0 = 0.07;
      public static final double targetPosition1 = 0.025;
      public static final double targetPosition2 = 0.237;
      public static final double targetPosition3 = 0.221;
      public static final double targetPosition4 = 0.663;
      public static final double targetPosition5 = 0.692;
      public static final double targetPosition6 = 0.724;
      public static final double targetPosition7 = 0.684;
      public static final double targetPosition8 = 0.280;
      public static final double targetPosition9 = 0.99;
      public static final double targetPosition10 = 0.545;
      public static final double targetPosition11 = 0.45;

      public static final double AKshoulderlength = 0.5588;
      public static final int AKshoulderlinewidth = 10;
      public static final double AKshoulderinitangle = 225;
      public static final double AKshouldermult = 252.45;
    }

    public static final class Wrist {
      public static final int wristMotorID = 16;

      public static final double kP = 0.00007;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kFF = 0.000156;

      public static final double ampLimit = 60;
      public static final double maxVelo = 5676;
      public static final double minVelo = 0;
      public static final double maxAcc = 11352;
      public static final double allowedErr = 0.0005;
      public static final double relallowedErr = 0.25;
      public static final double wristupperlimit = 44;
      public static final double wristlowerlimit = 0;

      public static final double manualspeed = 0.4;

      public static final double AKwristlength = 0.2794;
      public static final int AKwristlinewidth = 5;
      public static final double AKwristinitangle = 180;
      public static final double AKwristmult = 7.5;
    }

    public static final class Claw {
      public static final int clawMotorID = 17;
      public static final int ampLimit = 60;
      public static final double clawdownfastspeed = -1;
      public static final double clawdowndefaultspeed = -0.8;
      public static final double clawdowncubespeed = -0.7;
      public static final double clawdownslowspeed = -0.6;
      public static final double clawpulsingspeed = 0.25;
      public static final double clawupspeed = 0.85;
    }
  }

  public static final class Swerve {
    public static final int pigeonID = 20;
    public static final boolean invertGyro = false;

    public static final double translationMultiplier = 1.25;
    public static final double rotationMultiplier = 0.75;

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.5);
    public static final double wheelBase = Units.inchesToMeters(23.75);
    public static final double wheelDiameter = Units.inchesToMeters(3.94);//change to 3.7ish for MK4s when sure 
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.12 / 1.0);
    public static final double angleGearRatio = (12.8 / 1.0);

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.6;
    public static final double angleKI = 0.0;
    public static final double angleKD = 12.0;
    public static final double angleKF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.10;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
    public static final double driveKV = (2.44 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    public static final double maxSpeed = 6.925;
    public static final double maxAcceleration = maxSpeed*5;
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static NeutralMode driveNeutralMode = NeutralMode.Coast;
    public static NeutralMode angleNeutralMode = NeutralMode.Coast;

    /* Motor Inverts */
    public static final boolean driveMotorInvert = false;
    public static final boolean angleMotorInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
        public static final int driveMotorID = 7; //3
        public static final int angleMotorID = 8; //4
        public static final int canCoderID = 11; //9
        public static double angleOffset = 180.97;//314.5 these aren't accurate just refences
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }


    /* Front Right Module - Module 1 */
    public static final class Mod1 {
        public static final int driveMotorID = 18; //7
        public static final int angleMotorID = 6; //8
        public static final int canCoderID = 10; //11
        public static double angleOffset = 287.13;//246.7
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }


    /* Back Left Module - Module 2 */
    public static final class Mod2 {
        public static final int driveMotorID = 3; //3
        public static final int angleMotorID = 4; //4
        public static final int canCoderID = 9; //9
        public static double angleOffset = 297.86;//.47
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }


    /* Back Right Module - Module 3 */
    public static final class Mod3 {
        public static final int driveMotorID = 13; //13
        public static final int angleMotorID = 2; //2
        public static final int canCoderID = 12; //12
        public static double angleOffset = 226.49;//257.95
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }


  public static final class AutoConstants {
    public static final double kPController = 1;
    public static final double kPThetaController = 2;
    
    public static final double armHighDelay = 1.5;
    public static final double armMidDelay = 0.625;

    public static final class AutoSpeeds{
      public static final double bsafevelo = 2.5;
      public static final double bsafeacc = 2.5;

      public static final double bsafeengagevelo = 2.75;
      public static final double bsafeengageacc = 2.75;

      public static final double mvelo = 1.75;
      public static final double macc = 2;

      public static final double wsafevelo = 2;
      public static final double wsafeacc = 2;

      public static final double wthreecubevelo = 3;
      public static final double wthreecubeacc = 3;
    }
  }
}
