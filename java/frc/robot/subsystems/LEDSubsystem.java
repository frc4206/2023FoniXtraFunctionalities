package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  public enum State {
    PARTY,
    NORMAL,
    CONE,
    CUBE,
    OFF,
    CLAWRUNNING,
    EXTENDING,
    ATVALID,
    ATNOTVALID
  }

  protected final CANdle candle;
  private State state = State.NORMAL;

  public LEDSubsystem() {
    candle = new CANdle(Constants.LEDStrip.candleID, Constants.Canivore1);

    CANdleConfiguration config = new CANdleConfiguration();
    config.v5Enabled = false;
    config.disableWhenLOS = true;
    config.statusLedOffWhenActive = false;
    config.vBatOutputMode = CANdle.VBatOutputMode.Off;

    candle.configAllSettings(config);
    state = State.OFF;
  }
  
  public void animateOneLED(int red, int green, int blue, int white, int LEDSpot){
    candle.animate(new StrobeAnimation(red, green, blue, white, 1, Constants.LEDStrip.singleLED, LEDSpot));
  }

  public void partyAnim() {
    candle.animate(new RainbowAnimation(1.0, 1.0, Constants.LEDStrip.numLEDs, false, 0));
  }
    
  public void breathingWhite() {
    candle.animate(new LarsonAnimation(0, 0, 0, 255, 0.03, Constants.LEDStrip.numLEDs, LarsonAnimation.BounceMode.Center, 2, 0));
  }
  
  public void ATvalid(){
    candle.animate(new LarsonAnimation(50, /*50*/0, 0, 0, 0.03, Constants.LEDStrip.numLEDs, LarsonAnimation.BounceMode.Center, 2, 0));
  }

  public void ATnotvalid(){
    candle.animate(new LarsonAnimation(50, 0, 0, 0, 0.03, Constants.LEDStrip.numLEDs, LarsonAnimation.BounceMode.Center, 2, 0));
  }

  public void armExtending(){
    candle.animate(new StrobeAnimation(50, 50, 50, 0, 0.25, Constants.LEDStrip.numLEDs, 0));
  }

  public void clawRunning(){
    candle.animate(new StrobeAnimation(50, 0, 0, 0, 0.25, Constants.LEDStrip.numLEDs, 0));
  }

  public void allianceAnim(int LEDSpot, int numLEDs) {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      candle.animate(
          new StrobeAnimation(
            50,
            0,
            0,
            0,
            1,
            numLEDs,
            LEDSpot)
      );
    } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue){
      candle.animate(
          new StrobeAnimation(
            0,
            0,
            50,
            0,
            1,
            numLEDs,
            LEDSpot)
      );
    }
  }
  
  public void allianceAnimStatus(int LEDSpot, int numLEDs) {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      if (DriverStation.getLocation() == 1){
        candle.animate(
          new StrobeAnimation(
            50,
            0,
            0,
            0,
            0.01,
            numLEDs,
            LEDSpot)
        );
      } else if (DriverStation.getLocation() == 2){
        candle.animate(
          new StrobeAnimation(
            50,
            0,
            0,
            0,
            0.01,
            numLEDs,
            LEDSpot)
        );
      } else if (DriverStation.getLocation() == 3){
        candle.animate(
          new StrobeAnimation(
            50,
            0,
            0,
            0,
            0.01,
            numLEDs,
            LEDSpot)
        );
      }
    } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue){
      if (DriverStation.getLocation() == 1){ 
        candle.animate(new StrobeAnimation(
          0,
          0,
          50,
          0,
          0.01,
          numLEDs,
          LEDSpot)
        );
      } else if (DriverStation.getLocation() == 2){ 
        candle.animate(new StrobeAnimation(
          0,
          0,
          50,
          0,
          0.01,
          numLEDs,
          LEDSpot)
        );
      } else if (DriverStation.getLocation() == 3){ 
        candle.animate(new SingleFadeAnimation(
          0,
          0,
          50,
          0,
          0.01,
          numLEDs,
          LEDSpot)
        );
      }
    }
  }

  public void fireAnimation() {
    candle.animate(new FireAnimation(0.8, 1.0, Constants.LEDStrip.numLEDs * 3, 1.0, 1.0, false, 0));
  }
  
  public void coneModeAnimation(int numLEDs, int LEDSpot) {
    candle.animate(
      new StrobeAnimation(
        255/4,
        180/4,
        0,
        0,
        1,
        numLEDs,
        LEDSpot)
    );
  }
  
  public void cubeModeAnimation(int numLEDs, int LEDSpot) {
    candle.animate(
      new StrobeAnimation(
        128,
        0,
        128,
        0,
        1,
        numLEDs,
        LEDSpot)
    );
  }

  public void cyclePieceMode(){
    if (state == State.CONE){
      state = State.CUBE;
    } else if (state == State.CUBE){
      state = State.OFF;
    } else if (state == State.OFF){
      state = State.CONE;
    } else {
      state = State.CONE;
    }
  }

  public void cycleLEDMode(){
    if (state == State.PARTY){
      state = State.NORMAL;
      return;
    } else if (state == State.NORMAL){
      state = State.CUBE;
      return;
    } else if (state == State.CUBE){
      state = State.CONE;
      return;
    } else if (state == State.CONE){
      state = State.PARTY;
      return;
    }
  }

  @Override
  public void periodic() {
    // if (ElevatorSubsystem.elevatorMotor.getOutputCurrent() > 3){
    //   state = State.EXTENDING;
    // }
    // if (Limelight.limelighttable.getEntry("tid").getDouble(-1) != -1){
    //   state = State.ATVALID;
    // }
    // switch (state) {
    //   case NORMAL:
    //     if (DriverStation.isDisabled()) {
    //       allianceAnim(0, Constants.LEDStrip.numLEDs);
    //     } else if (DriverStation.isAutonomousEnabled()) {
    //       fireAnimation();
    //     } else {
    //       breathingWhite();
    //     }
    //     break;
    //   case PARTY:
    //     partyAnim();
    //     break;
    //   case CONE:
    //     coneModeAnimation(Constants.LEDStrip.numLEDs, 0);
    //     break;
    //   case CUBE:
    //     cubeModeAnimation(Constants.LEDStrip.numLEDs, 0);
    //     break;
    //   case CLAWRUNNING:
    //     clawRunning();
    //     break;
    //   case EXTENDING:
    //     armExtending();
    //     break;
    //   case ATVALID:
    //     ATvalid();
    //     break;
    //   case ATNOTVALID:
    //     ATnotvalid();
    //     break;
    //   default:
    //     breathingWhite();
    //     break;
    // } 
    cubeModeAnimation(33, 0);
  }
}