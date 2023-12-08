package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public static Pigeon2 gyro;
    public HeadingState headingState = HeadingState.FREE;
    boolean hasResetOdometry = false;
    public double[] ypr;
    double nomYaw = 0;
    double realYaw = 0;
    double rotations = 0;
    double pitch;
    double roll;

    Rotation2d noRotation2d = new Rotation2d(0,0);
    public double[] convertedCords = {0,0};
    public Pose2d AprilCords = new Pose2d(0,0, noRotation2d);
    double[] botpose;

    //Used in calculation of swerveruntime
    double currX;
    double currY;
    double currDist;
    double initialX = 0;
    double initialY = 0;
    double initialDist = 0;

    static double avgMotorTemp;
    
    public enum HeadingState{
        FORWARD,
        BACKWARD,
        FREE
    }

    public SwerveSubsystem() {
        ypr = new double[3];
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Canivore1);
        gyro.configFactoryDefault();
        zeroGyro();
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        
    }
    
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation, 
                    getYaw()
                    )
                    : new ChassisSpeeds(
                        translation.getX(), 
                        translation.getY(), 
                        rotation)
                        );
                        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
                        
                                    for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }
    
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }
    
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public SwerveModulePosition[] getModulePositionsInverted(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPositionInverted();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }
    
    public static void setGyro(double degrees){
        gyro.setYaw(degrees);
    }

    public double getNominalYaw(){
        realYaw = getYaw().getDegrees();
        rotations = Math.round((realYaw/360));
        if (realYaw < 0){
            nomYaw = -realYaw;
            if (nomYaw > 360){
                nomYaw += (360*rotations);
            }
            if (nomYaw < 0){
                nomYaw += 360;
            }
            nomYaw = 360 - nomYaw;
        } else {
            nomYaw = realYaw - (360*rotations);
            if (nomYaw < 0){
                nomYaw += 360;
            }
        }
        return nomYaw;
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    public Pose2d getPose(){
        return swerveOdometry.getPoseMeters();
    }
    
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void setHeadingState(){
        if (headingState == HeadingState.FORWARD){
            headingState = HeadingState.BACKWARD;
            return;
        } else if (headingState == HeadingState.BACKWARD){
            headingState = HeadingState.FREE;
            return;
        } else if (headingState == HeadingState.FREE){
            headingState = HeadingState.FORWARD;
            return;
        }
    }

    public void freeHeadingState(){
        headingState = HeadingState.FREE;
    }

    public void toggleCoast(){
        if (GlobalVariables.isCoast){
            for (SwerveModule mod : mSwerveMods){
                mod.mDriveMotor.setNeutralMode(NeutralMode.Brake);
            }
            GlobalVariables.isCoast = false;
        } else {
            for (SwerveModule mod : mSwerveMods){
                mod.mDriveMotor.setNeutralMode(NeutralMode.Coast);
            }
            GlobalVariables.isCoast = true;
        }
    }

    public void BalanceBrake(){
        Translation2d translation = new Translation2d(0, 0.05).times(Constants.Swerve.maxSpeed);
        double rotation = 0;
        drive(translation, rotation, true, true);
    }

    public void AutoBalanceClose(){
        double kP = 0.06;
        double kF = 0.02;

        double errorP = pitch - 2.25;
        double outputP = errorP*kP + kF;

        double errorR = roll - 2.25;
        double outputR = errorR*kP + kF;
        
        while (!(pitch < 6 && pitch > -6)){
            Translation2d translation = new Translation2d(outputP, 0).times(Constants.Swerve.maxSpeed-4);
            double rotation = 0;
            drive(translation, rotation, true, true);
            return;
        }

        while (!(roll < 6 && roll > -6)) {
            Translation2d translation = new Translation2d(outputR, 0).times(Constants.Swerve.maxSpeed-4);
            double rotation = 0;
            drive(translation, rotation, true, true);    
            return;
        }

        drive(new Translation2d(0,0), 0, true, true);
    }

    public void AutoBalanceFar(){
        double kP = 0.06;
        double kF = 0.02;

        double errorP = pitch - 3.5;
        double outputP = errorP*kP + kF;

        double errorR = roll - 3.5;
        double outputR = errorR*kP + kF;
        
        while (!(pitch < 6 && pitch > -6)){
            Translation2d translation = new Translation2d(-outputP, 0).times(Constants.Swerve.maxSpeed-4);
            double rotation = 0;
            drive(translation, rotation, true, true);
            return;
        }

        while (!(roll < 6 && roll > -6)) {
            Translation2d translation = new Translation2d(-outputR, 0).times(Constants.Swerve.maxSpeed-4);
            double rotation = 0;
            drive(translation, rotation, true, true);    
            return;
        }

        drive(new Translation2d(0,0), 0, true, true);
        for (SwerveModule mod : mSwerveMods){
            mod.mDriveMotor.setNeutralMode(NeutralMode.Brake);
        }
    }
    
    
    @Override
    public void periodic(){
        gyro.getYawPitchRoll(ypr);
        pitch = ypr[1];
        roll = ypr[2];
        swerveOdometry.update(getYaw(), getModulePositions());
        resetOdometry(getPose());

        Logger.getInstance().recordOutput("SwerveStates", getStates());
        Logger.getInstance().recordOutput("Odometry", getPose());
        Logger.getInstance().recordOutput("Yaw", getYaw().getDegrees());
        currX = getPose().getX();
        currY = getPose().getY();

        currDist = Math.sqrt(currX*currX + currY*currY);
        if ((currX - initialX) >= 0.05 || (currY - initialY) >= 0.05 || (currDist - initialDist) >= 0.05){
            GlobalVariables.swerveRuntime += 0.03;
        }
        initialX = currX;
        currX = 0;
        initialY = currY;
        currY = 0;
        initialDist = currDist;
        currDist = 0;

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            avgMotorTemp += mod.mDriveMotor.getTemperature();
            avgMotorTemp += mod.mAngleMotor.getTemperature();
        }
        avgMotorTemp /= 8;
        GlobalVariables.avgMotorTemp = avgMotorTemp;
    }
}