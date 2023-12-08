// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.IOException;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase  {
  public static double currPipeline = 0;
  static SwerveSubsystem swerve = new SwerveSubsystem();
    
  static double xI;
  static double yI;
  static double rI;
  static double xM;
  static double yM;
  static double rM;
  static double xF;
  static double yF;
  static double rF;
  static double xFF;
  static double yFF;
  static double rFF;
  static double[] pose1;
  static double[] pose2;
  static double[] pose3;
  static double[] pose4;

  public static double[] CubeCoordinates;
  public static double CubeX;
  public static double CubeY;

  static double[] rawbotpose;
  static double[] botpose;
  static int ctr;

  static double aprilid = 0;
  public double X = 0;
  public double Y = 0;
  public double Z = 0;
  public double Yaw = 0;
  double[] cords = {X, Y, Z};

  static double distX = 0;
  static double distY = 0;

  Supplier<Double> supplier = () -> 0.1;
  double range;
  static double camAngleDegrees = 9;
  static double camFromCubeMeters = 0.2032;
  static double camtoCenterX = 0.2286;
  static double camtoCenterY = 0.254;
  static double distfrombot;
  static double XDegrees;
  static double YDegrees;
  static boolean isForward;
  static boolean quadrantI;
  static boolean quadrantII;
  static boolean quadrantIII;
  static boolean quadrantIV;

  public static NetworkTable limelighttable;
  public Limelight() throws IOException {
    limelighttable = NetworkTableInstance.getDefault().getTable("limelight-hubert");
    limelighttable.getEntry("pipeline").setNumber(0);
  }

  public void changePipeline() {
    double currPipeline = limelighttable.getEntry("pipeline").getDouble(0);
    if (currPipeline == 0) {
      limelighttable.getEntry("pipeline").setDouble(3);
      currPipeline = 3;
    } else if (currPipeline == 3){
      limelighttable.getEntry("pipeline").setDouble(0);
      currPipeline = 0;
    } else {
      limelighttable.getEntry("pipeline").setDouble(0);
    }
  }

  public static double[] getDesiredPositions(int id, int pos, int stage, boolean isHigh){
    xI = 0;
    yI = 0;
    rI = 0;
    xM = 0;
    yM = 0;
    rM = 0;
    xF = 0;
    yF = 0;
    rF = 0;
    xFF = 0;
    yFF = 0;
    rFF = 0;
    switch (id){
      case 1:
        switch (pos){
          case 1:
          xI = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreLeft.xinit;
          yI = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreLeft.yinit;
          rI = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreLeft.rinit;
          xM = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreLeft.xmiddle;
          yM = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreLeft.ymiddle;
          rM = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreLeft.rmiddle;
          xF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreLeft.xfinal;
          yF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreLeft.yfinal;
          rF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreLeft.rfinal;
          xFF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreLeft.xfinish;
          yFF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreLeft.yfinish;
          rFF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreLeft.rfinish;
          break;
          case 2:
          xI = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreMiddle.xinit;
          yI = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreMiddle.yinit;
          rI = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreMiddle.rinit;
          xM = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreMiddle.xmiddle;
          yM = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreMiddle.ymiddle;
          rM = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreMiddle.rmiddle;
          xF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreMiddle.xfinal;
          yF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreMiddle.yfinal;
          rF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreMiddle.rfinal;
          xFF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreMiddle.xfinish;
          yFF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreMiddle.yfinish;
          rFF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreMiddle.rfinish;
          break;
          case 3:
          xI = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreRight.xinit;
          yI = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreRight.yinit;
          rI = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreRight.rinit;
          xM = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreRight.xmiddle;
          yM = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreRight.ymiddle;
          rM = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreRight.rmiddle;
          xF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreRight.xfinal;
          yF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreRight.yfinal;
          rF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreRight.rfinal;
          xFF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreRight.xfinish;
          yFF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreRight.yfinish;
          rFF = frc.robot.Constants.Limelight.Coords.Red.Score1.ScoreRight.rfinish;
          break;
          }
          break;
          case 2:
          switch (pos){
            case 1:
            xI = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreLeft.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreLeft.yinit;
            rI = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreLeft.rinit;
            xM = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreLeft.xmiddle;
            yM = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreLeft.ymiddle;
            rM = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreLeft.rmiddle;
            xF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreLeft.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreLeft.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreLeft.rfinal;
            xFF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreLeft.xfinish;
            yFF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreLeft.yfinish;
            rFF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreLeft.rfinish;
            break;
            case 2:
            xI = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreMiddle.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreMiddle.yinit;
            rI = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreMiddle.rinit;
            xM = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreMiddle.xmiddle;
            yM = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreMiddle.ymiddle;
            rM = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreMiddle.rmiddle;
            xF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreMiddle.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreMiddle.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreMiddle.rfinal;
            xFF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreMiddle.xfinish;
            yFF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreMiddle.yfinish;
            rFF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreMiddle.rfinish;
            break;
          case 3:
          xI = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreRight.xinit;
          yI = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreRight.yinit;
          rI = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreRight.rinit;
          xM = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreRight.xmiddle;
          yM = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreRight.ymiddle;
          rM = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreRight.rmiddle;
          xF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreRight.xfinal;
          yF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreRight.yfinal;
          rF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreRight.rfinal;
          xFF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreRight.xfinish;
          yFF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreRight.yfinish;
          rFF = frc.robot.Constants.Limelight.Coords.Red.Score2.ScoreRight.rfinish;
          break;
        }
        break;
        case 3:
        switch (pos){
          case 1:
          xI = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreLeft.xinit;
          yI = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreLeft.yinit;
          rI = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreLeft.rinit;
            xM = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreLeft.xmiddle;
            yM = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreLeft.ymiddle;
            rM = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreLeft.rmiddle;
            xF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreLeft.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreLeft.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreLeft.rfinal;
            xFF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreLeft.xfinish;
          yFF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreLeft.yfinish;
          rFF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreLeft.rfinish;
            break;
            case 2:
            xI = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreMiddle.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreMiddle.yinit;
            rI = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreMiddle.rinit;
            xM = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreMiddle.xmiddle;
            yM = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreMiddle.ymiddle;
            rM = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreMiddle.rmiddle;
            xF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreMiddle.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreMiddle.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreMiddle.rfinal;
            xFF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreMiddle.xfinish;
            yFF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreMiddle.yfinish;
            rFF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreMiddle.rfinish;
            break;
            case 3:
            xI = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreRight.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreRight.yinit;
            rI = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreRight.rinit;
            xM = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreRight.xmiddle;
            yM = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreRight.ymiddle;
            rM = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreRight.rmiddle;
            xF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreRight.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreRight.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreRight.rfinal;
            xFF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreRight.xfinish;
            yFF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreRight.yfinish;
            rFF = frc.robot.Constants.Limelight.Coords.Red.Score3.ScoreRight.rfinish;
            break;
          }
          break;
          case 4:
          switch (pos){
            case 1:
            xI = frc.robot.Constants.Limelight.Coords.Red.Pickup4.PickupLeft.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Red.Pickup4.PickupLeft.yinit;
            rI = frc.robot.Constants.Limelight.Coords.Red.Pickup4.PickupLeft.rinit;
            xF = frc.robot.Constants.Limelight.Coords.Red.Pickup4.PickupLeft.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Red.Pickup4.PickupLeft.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Red.Pickup4.PickupLeft.rfinal;
            break;
            case 2:
            xI = frc.robot.Constants.Limelight.Coords.Red.Pickup4.PickupRight.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Red.Pickup4.PickupRight.yinit;
            rI = frc.robot.Constants.Limelight.Coords.Red.Pickup4.PickupRight.rinit;
            xF = frc.robot.Constants.Limelight.Coords.Red.Pickup4.PickupRight.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Red.Pickup4.PickupRight.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Red.Pickup4.PickupRight.rfinal;
            break;
          }
          break;
          case 5:
          switch (pos){
            case 1:
            xI = frc.robot.Constants.Limelight.Coords.Blue.Pickup5.PickupLeft.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Blue.Pickup5.PickupLeft.xinit;
            rI = frc.robot.Constants.Limelight.Coords.Blue.Pickup5.PickupLeft.xinit;
            xF = frc.robot.Constants.Limelight.Coords.Blue.Pickup5.PickupLeft.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Blue.Pickup5.PickupLeft.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Blue.Pickup5.PickupLeft.rfinal;
            break;
            case 2:
            xI = frc.robot.Constants.Limelight.Coords.Blue.Pickup5.PickupRight.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Blue.Pickup5.PickupRight.xinit;
            rI = frc.robot.Constants.Limelight.Coords.Blue.Pickup5.PickupRight.xinit;
            xF = frc.robot.Constants.Limelight.Coords.Blue.Pickup5.PickupRight.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Blue.Pickup5.PickupRight.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Blue.Pickup5.PickupRight.rfinal;
            break;
          }
          break;
          case 6:
          switch (pos){
            case 1:
            xI = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreLeft.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreLeft.yinit;
            rI = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreLeft.rinit;
            xM = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreLeft.xmiddle;
            yM = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreLeft.ymiddle;
            rM = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreLeft.rmiddle;
            xF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreLeft.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreLeft.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreLeft.rfinal;
            xFF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreLeft.xfinish;
            yFF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreLeft.yfinish;
            rFF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreLeft.rfinish;
            break;
            case 2:
            xI = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.yinit;
            rI = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.rinit;
            xM = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.xmiddle;
            yM = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.ymiddle;
            rM = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.rmiddle;
            xF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.rfinal;
            xFF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.xfinish;
            yFF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.yfinish;
            rFF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreMiddle.rfinish;
            break;
            case 3:
            xI = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreRight.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreRight.yinit;
            rI = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreRight.rinit;
            xM = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreRight.xmiddle;
            yM = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreRight.ymiddle;
            rM = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreRight.rmiddle;
            xF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreRight.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreRight.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreRight.rfinal;
            xFF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreRight.xfinish;
            yFF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreRight.yfinish;
            rFF = frc.robot.Constants.Limelight.Coords.Blue.Score6.ScoreRight.rfinish;
            break;
          }
          break;
          case 7:
          switch (pos){
            case 1:
            xI = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreLeft.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreLeft.yinit;
            rI = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreLeft.rinit;
            xM = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreLeft.xmiddle;
            yM = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreLeft.ymiddle;
            rM = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreLeft.rmiddle;
            xF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreLeft.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreLeft.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreLeft.rfinal;
            xFF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreLeft.xfinish;
            yFF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreLeft.yfinish;
            rFF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreLeft.rfinish;
            break;
            case 2:
            xI = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreMiddle.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreMiddle.yinit;
            rI = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreMiddle.rinit;
            xM = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreMiddle.xmiddle;
            yM = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreMiddle.ymiddle;
            rM = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreMiddle.rmiddle;
            xF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreMiddle.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreMiddle.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreMiddle.rfinal;
            xFF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreMiddle.xfinish;
            yFF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreMiddle.yfinish;
            rFF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreMiddle.rfinish;
            break;
            case 3:
            xI = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreRight.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreRight.yinit;
            rI = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreRight.rinit;
            xM = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreRight.xmiddle;
            yM = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreRight.ymiddle;
            rM = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreRight.rmiddle;
            xF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreRight.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreRight.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreRight.rfinal;
            xFF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreRight.xfinish;
            yFF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreRight.yfinish;
            rFF = frc.robot.Constants.Limelight.Coords.Blue.Score7.ScoreRight.rfinish;
            break;
          }
          break;
          case 8:
          switch (pos){
            case 1:
            xI = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreLeft.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreLeft.yinit;
            rI = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreLeft.rinit;
            xM = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreLeft.xmiddle;
            yM = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreLeft.ymiddle;
            rM = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreLeft.rmiddle;
            xF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreLeft.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreLeft.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreLeft.rfinal;
            xFF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreLeft.xfinish;
            yFF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreLeft.yfinish;
            rFF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreLeft.rfinish;
            break;
            case 2:
            xI = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreMiddle.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreMiddle.yinit;
            rI = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreMiddle.rinit;
            xM = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreMiddle.xmiddle;
            yM = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreMiddle.ymiddle;
            rM = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreMiddle.rmiddle;
            xF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreMiddle.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreMiddle.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreMiddle.rfinal;
            xFF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreMiddle.xfinish;
            yFF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreMiddle.yfinish;
            rFF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreMiddle.rfinish;
            break;
            case 3:
            xI = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreRight.xinit;
            yI = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreRight.yinit;
            rI = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreRight.rinit;
            xM = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreRight.xmiddle;
            yM = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreRight.ymiddle;
            rM = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreRight.rmiddle;
            xF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreRight.xfinal;
            yF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreRight.yfinal;
            rF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreRight.rfinal;
            xFF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreRight.xfinish;
            yFF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreRight.yfinish;
            rFF = frc.robot.Constants.Limelight.Coords.Blue.Score8.ScoreRight.rfinish;
            break;
          }
          break;
          default:
          xI = 0;
          yI = 0;
          rI = 0;
          xM = 0;
          yM = 0;
          rM = 0;
          xF = 0;
          yF = 0;
          rF = 0;
          break;
        }
    if (isHigh){
      pose1 = new double[] {xI, yI, rI};
      pose2 = new double[] {xM, yM, rM};
      pose3 = new double[] {xF, yF, rF};
      pose4 = new double[] {xFF, yFF, rFF};
    } else {
      pose1 = new double[] {xI, yI, 180};
      pose2 = new double[] {xM, yM, 180};
      pose3 = new double[] {xF+0.75, yF, 180};
      pose4 = new double[] {xFF+0.75, yFF, 180};  
    }
    if (stage == 1){
      System.out.println("Initial: " + xI + ", " + yI);
      return pose1;
    } else if (stage == 2){
      System.out.println("Middle: " + xM + ", " + yM);
      return pose2;
    } else if (stage == 3){
      System.out.println("Final: " + xF + ", " + yF);
      return pose3;
    } else if (stage == 4){
      System.out.println("Finish: " + xFF + ", " + yFF);
      return pose4;
    } else {
      return pose1;
    }
  }


  public static double[] getCubeDistance(){
    double[] cubeCoordinates = new double[2];
    double cubeX = 0;
    double cubeY = 0;
    double botYaw = getBotPoseAdjusted()[2];
    YDegrees = limelighttable.getEntry("ty").getDouble(0.0) < 0 ? limelighttable.getEntry("ty").getDouble(0.0) - camAngleDegrees : 0;
    distfrombot = (camFromCubeMeters/(Math.tan(YDegrees)));

    cubeX = distfrombot*Math.cos(botYaw);
    cubeY = distfrombot*Math.sin(botYaw);

    cubeCoordinates[0] = cubeX;
    cubeCoordinates[1] = cubeY;

    Logger.getInstance().recordOutput("CubeX", cubeX);
    Logger.getInstance().recordOutput("CubeY", cubeY);
    return cubeCoordinates;
  }


  public static double[] getCubeCoordsUpdated(){
    double[] cubeCoordinates = new double[2];
    double[] botPose = getBotPoseAdjusted();
    double botX = botPose[0];
    double botY = botPose[1];
    double botYaw = botPose[2];
    double cubeX = 0;
    double cubeY = 0;

    if ((botYaw > 0 && botYaw <= 90) || (botYaw >= 270 && botYaw < 360)){
      isForward = true;
    } else if (botYaw > 90 && botYaw < 270){
      isForward = false;
    }

    YDegrees = limelighttable.getEntry("ty").getDouble(0.0) < 0 ? limelighttable.getEntry("ty").getDouble(0.0) - camAngleDegrees : 0;
    distfrombot = (camFromCubeMeters/(Math.tan(YDegrees)));

    if (isForward){
      cubeX = botX + distfrombot*Math.sin(botYaw)*Constants.Limelight.cubeFloorMultiplierX;
      cubeY = botY + distfrombot*Math.cos(botYaw)*Constants.Limelight.cubeFloorMultiplierY;
    }
    else {
      cubeX = botX + distfrombot*Math.sin(botYaw)*Constants.Limelight.cubeFloorMultiplierX;
      cubeY = botY + distfrombot*Math.cos(botYaw)*Constants.Limelight.cubeFloorMultiplierY;  
    }

    if (((cubeX > 15 || cubeX < 0 || (cubeX < 1 && cubeX > -1))) || ((cubeY > 15 || cubeY < 0) || (cubeY < 1 && cubeY > -1))){
      cubeX = 0;
      cubeY = 0;
    }

    cubeCoordinates[0] = cubeX;
    cubeCoordinates[1] = cubeY;

    Logger.getInstance().recordOutput("IsForward", isForward);
    Logger.getInstance().recordOutput("CubeX", cubeX);
    Logger.getInstance().recordOutput("CubeY", cubeY);
    return cubeCoordinates;
  }


  public static boolean ATDetected(){
    return (limelighttable.getEntry("tid").getDouble(0) != -1.0);
  }
  
  public static boolean ATwithinDistance(){
    if (ATDetected()){
      rawbotpose = limelighttable.getEntry("botpose").getDoubleArray(new double[6]);
      if ((rawbotpose[0] - frc.robot.Constants.Limelight.xOffSet < 0.6) || (rawbotpose[1] - frc.robot.Constants.Limelight.yOffSet < 0.05)){
        return false;
      } if (rawbotpose[0] - frc.robot.Constants.Limelight.xOffSet > 4.1 && rawbotpose[0] - frc.robot.Constants.Limelight.xOffSet < 15.8){
        return false;
      } else if (rawbotpose[0] - frc.robot.Constants.Limelight.xOffSet < 4.1 || rawbotpose[0] - frc.robot.Constants.Limelight.xOffSet > 15.8){
        return true;
      }
    }
    return false;
  }


  public Pose2d getBotPosePose(){
    ctr = 0;
    rawbotpose = limelighttable.getEntry("botpose").getDoubleArray(new double[6]);
    botpose = new double[3];
    for (double posepoint : rawbotpose){
      if (ctr==0) {
          if (posepoint == 0){
            botpose[ctr] = posepoint + frc.robot.Constants.Limelight.xOffSet;
          } else {
            if (ATwithinDistance()){
              botpose[ctr] = posepoint - frc.robot.Constants.Limelight.xOffSet;
            } else {
              posepoint = 0;
              botpose[ctr] = posepoint + frc.robot.Constants.Limelight.xOffSet;
            }
          }
      } else if (ctr==1) {
          if (posepoint == 0){
            botpose[ctr] = posepoint + frc.robot.Constants.Limelight.yOffSet;
          } else {
            if (ATwithinDistance()){
              botpose[ctr] = posepoint - frc.robot.Constants.Limelight.yOffSet;
            } else {
              posepoint = 0;
              botpose[ctr] = posepoint + frc.robot.Constants.Limelight.yOffSet;
            }
          }
      } else if (ctr==2) {
        botpose[ctr] = swerve.getNominalYaw();
      }
      ctr+=1;
    }
    if (((botpose[0] != frc.robot.Constants.Limelight.xOffSet) && (botpose[1] != frc.robot.Constants.Limelight.yOffSet)) && ATwithinDistance()){
      swerve.resetOdometry(new Pose2d(new Translation2d(botpose[0], botpose[1]), swerve.getYaw()));
    } else {
      swerve.swerveOdometry.update(swerve.getYaw(), swerve.getModulePositions());
    }
    if (!ATDetected() || !ATwithinDistance()){
      botpose[0] = swerve.swerveOdometry.getPoseMeters().getX();
      botpose[1] = swerve.swerveOdometry.getPoseMeters().getY();
    }
    Pose2d robotPose = new Pose2d(new Translation2d(botpose[0], botpose[1]), new Rotation2d(Units.degreesToRadians(botpose[2])));
    return robotPose;
  }


  public static double[] getBotPoseAdjusted(){
    ctr = 0;
    rawbotpose = limelighttable.getEntry("botpose").getDoubleArray(new double[6]);
    botpose = new double[6];
    for (double posepoint : rawbotpose){
      if (ctr == 5) {
        break;
      } if (ctr==0) {
          if (posepoint == 0){
            botpose[ctr] = posepoint + frc.robot.Constants.Limelight.xOffSet;
          } else {
            if (ATwithinDistance()){
              botpose[ctr] = posepoint - frc.robot.Constants.Limelight.xOffSet;
            } else {
              posepoint = 0;
              botpose[ctr] = posepoint + frc.robot.Constants.Limelight.xOffSet;
            }
          }
      } else if (ctr==1) {
          if (posepoint == 0){
            botpose[ctr] = posepoint + frc.robot.Constants.Limelight.yOffSet;
          } else {
            if (ATwithinDistance()){
              botpose[ctr] = posepoint - frc.robot.Constants.Limelight.yOffSet;
            } else {
              posepoint = 0;
              botpose[ctr] = posepoint + frc.robot.Constants.Limelight.yOffSet;
            }
          }
      } else if (ctr==2) {
        botpose[ctr] = swerve.getNominalYaw();
      } else {
        botpose[ctr] = posepoint;
      }
      ctr+=1;
    }
    if (((botpose[0] != frc.robot.Constants.Limelight.xOffSet) && (botpose[1] != frc.robot.Constants.Limelight.yOffSet)) && ATwithinDistance()){
      swerve.resetOdometry(new Pose2d(new Translation2d(botpose[0], botpose[1]), swerve.getYaw()));
    } else {
      swerve.swerveOdometry.update(swerve.getYaw(), swerve.getModulePositions());
    }
    if (!ATDetected() || !ATwithinDistance()){
      botpose[0] = swerve.swerveOdometry.getPoseMeters().getX();
      botpose[1] = swerve.swerveOdometry.getPoseMeters().getY();
    }
    Logger.getInstance().recordOutput("Field Pose", botpose);
    return botpose;
  }


  public static double[] getBotPose(){
    ctr = 0;
    rawbotpose = limelighttable.getEntry("botpose").getDoubleArray(new double[6]);
    botpose = new double[6];
    for (double posepoint : rawbotpose){
      if (ctr == 5) {
        break;
      } if (ctr==0) {
          if (posepoint == 0){
            botpose[ctr] = posepoint + frc.robot.Constants.Limelight.xOffSet;
          } else {
            botpose[ctr] = posepoint - frc.robot.Constants.Limelight.xOffSet;
          }
      } else if (ctr==1) {
          if (posepoint == 0){
            botpose[ctr] = posepoint + frc.robot.Constants.Limelight.yOffSet;
          } else {
            botpose[ctr] = posepoint - frc.robot.Constants.Limelight.yOffSet;
          }
      } else if (ctr==2) {
        botpose[ctr] = swerve.getNominalYaw();
      } else {
        botpose[ctr] = posepoint;
      }
      ctr+=1;
    }
    if ((botpose[0] != frc.robot.Constants.Limelight.xOffSet) && (botpose[1] != frc.robot.Constants.Limelight.yOffSet)){
      swerve.resetOdometry(new Pose2d(new Translation2d(botpose[0], botpose[1]), swerve.getYaw()));
    }
    else{
      swerve.swerveOdometry.update(swerve.getYaw(), swerve.getModulePositions());
    }
    if (limelighttable.getEntry("tid").getDouble(0) == -1.0){
      botpose[0] = swerve.swerveOdometry.getPoseMeters().getX();
      botpose[1] = swerve.swerveOdometry.getPoseMeters().getY();
    }
    Logger.getInstance().recordOutput("Field Pose", botpose);
    return botpose;
  }



  @Override
  public void periodic() {
    getBotPoseAdjusted();
    CubeCoordinates = getCubeCoordsUpdated();
    CubeX = CubeCoordinates[0];
    CubeY = CubeCoordinates[1];
    double result2 = limelighttable.getEntry("tx").getDouble(0);
    cords[0] = botpose[0];
    cords[1] = botpose[1];
    cords[2] = botpose[2];
    Yaw = result2;
    aprilid = limelighttable.getEntry("tid").getDouble(0);
    Logger.getInstance().recordOutput("distX", distX);
    Logger.getInstance().recordOutput("distY", distY);
    Logger.getInstance().recordOutput("ATDETECTED", ATDetected());
    Logger.getInstance().recordOutput("ATWITHINDISTANCE", ATwithinDistance());
    Logger.getInstance().recordOutput("RawX", limelighttable.getEntry("botpose").getDoubleArray(new double[6])[0]);
  }
}
