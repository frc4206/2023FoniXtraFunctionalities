package frc.robot.commands.Swerve;

import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.HeadingState;
import edu.wpi.first.wpilibj.Joystick;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private SwerveSubsystem s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    private double[] botposearray;
    private double yawSet;
    private double errorYaw;
    private double outputYaw;
    private PIDController pidyawi = new PIDController(Constants.Limelight.RPID.kPi, Constants.Limelight.RPID.kIi, Constants.Limelight.RPID.kDi);
    private PIDController pidyaw = new PIDController(Constants.Limelight.RPID.kP, Constants.Limelight.RPID.kI, Constants.Limelight.RPID.kD);
    private double rAxis;

    /**
     * Driver control
     */
    public TeleopSwerve(SwerveSubsystem s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        for (SwerveModule mod : s_Swerve.mSwerveMods){
            mod.mDriveMotor.setNeutralMode(NeutralMode.Coast);
        }
    }

    @Override
    public void execute() {
        botposearray = Limelight.getBotPoseAdjusted();

        switch (s_Swerve.headingState){
            case FORWARD:
                if (botposearray[2] > 0 && botposearray[2] < 10){
                    yawSet = 0;
                }
                else if (botposearray[2] < 360 && botposearray[2] > 350){
                    yawSet = 360;
                }
                break;
            case BACKWARD:
                yawSet = 180;
                break;
            case FREE:
                yawSet = 0;
                break;
            default: 
                yawSet = 0;
                break;
        }

        errorYaw = botposearray[2] - yawSet;
        if (Math.abs(errorYaw) > 1.5) {
          outputYaw = pidyaw.calculate(botposearray[2], yawSet);
        } else {
          outputYaw = pidyawi.calculate(botposearray[2], yawSet);
        }

        double yAxis = (-controller.getRawAxis(translationAxis)*Constants.Swerve.translationMultiplier) < 0 ? (-controller.getRawAxis(translationAxis)*Constants.Swerve.translationMultiplier) + Constants.stickDeadband : (-controller.getRawAxis(translationAxis)*Constants.Swerve.translationMultiplier) - Constants.stickDeadband; //-Constants.stickDeadband;
        double xAxis = (-controller.getRawAxis(strafeAxis)*Constants.Swerve.translationMultiplier) < 0 ? (-controller.getRawAxis(strafeAxis)*Constants.Swerve.translationMultiplier) + Constants.stickDeadband : (-controller.getRawAxis(strafeAxis)*Constants.Swerve.translationMultiplier) - Constants.stickDeadband; //-Constants.stickDeadband;
        Logger.getInstance().recordOutput("translationAxis", -controller.getRawAxis(translationAxis));
        Logger.getInstance().recordOutput("strafeAxis", -controller.getRawAxis(strafeAxis));
        switch (s_Swerve.headingState){
            case FORWARD:
                rAxis = outputYaw;
                break;
            case BACKWARD:
                rAxis = outputYaw;
                break;
            case FREE:
                rAxis = -controller.getRawAxis(rotationAxis)*Constants.Swerve.rotationMultiplier;
                break;
            default:
                rAxis = -controller.getRawAxis(rotationAxis)*Constants.Swerve.rotationMultiplier;
                break;
        }
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        if (s_Swerve.headingState == HeadingState.FREE){
            rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;
        }

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        Logger.getInstance().recordOutput("translationX", translation.getX());
        Logger.getInstance().recordOutput("translationY", translation.getY());
        
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
