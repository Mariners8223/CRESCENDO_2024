// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.subsystem.Arm.Arm.ArmPostion;
import frc.util.PIDFGains;

/** Add yo
import frc.util.PIDFGains;

/** Add your docs here. */
public class Constants {

    // public static final Translation2d[][] robotZones = new Translation2d[][]{
    //     //TODO: add robot zones
    // };

    public static final List<Translation2d> robotZones = new ArrayList<Translation2d>() {
        //TODO: add robot zones
    };

    public static final Pose2d AmpPose = new Pose2d(3, 8, Rotation2d.fromDegrees(90));
    public static final Translation3d SpeakerTranslation = new Translation3d( Units.inchesToMeters(-1.5), 5, 3);

    public static final class Logger{
        public static final String save_location = ""; //add save lcoation (probably a usb stick so /u)
    }

    public static final class Controllers{
        public static final double joyStickDeadBand = 0.05;
        public static final double triggerDeadBand = 0.1;
    }

    public static final class Vision{
        public static final Transform3d cameraToRobotCenter = new Transform3d(
            -0.45,
            0.001,
            0.2,
            new Rotation3d(
            0,
            20,
            -180));

        public static final class PhotonVision{
            public static final String cameraName = "name";
        }

        public static final class LimeLight{
            public static final String limeLightName = "";
        }
    }

    public static final class ArmConstants {
        public static final double armLengthMeters = 0.46;
        public static final double shooterAndIntakeLengthMeters = 0.361;

        public static final double mainPivotDistanceFromCenterMeters = 0.113;
        public static final double armHeightFromFrameMeters = 0.245;
        public static final double SecondaryMotorDistanceFromShooterMeters = 0.06;

        public static final ArmPostion FloorPosition = new ArmPostion(0, 0, 0); // In radians
        public static final ArmPostion SourcePosition = new ArmPostion(0, 0, 0);
    
        public static final double RobotHightFromGround = 0;
        public static final double SpeakerLength = 1.05;
        public static final double SpeakerMidlleLocationY = Units.inchesToMeters(218.42);
        public static final double SpeakerBottomLocationY = Units.inchesToMeters(218.42) - SpeakerLength/2;
        public static final double FieldYLength = Units.inchesToMeters(323.00);
        public static final double SpeakerIsCenterRatioReverse = 1 - SpeakerLength/(2*(FieldYLength - SpeakerLength/2 - SpeakerMidlleLocationY));
        public static final double SpeakerCenterLocationX = Units.inchesToMeters(-1.5);// - 0.75;
        public static final double SpeakerHight = 2.31;//meter
        public static final double RatioFieldToSpeakerReverse = 1 - (SpeakerLength/FieldYLength);
        public static final double SpeakerIsCenterRatioBottomLocation = FieldYLength - 2*(FieldYLength - SpeakerLength/2 - SpeakerMidlleLocationY);

        public static final ArmPostion AmpArmPosition = new ArmPostion();
        
        public static final ArmPostion ShootingPositionSniper = new ArmPostion();
        public static final ArmPostion ShootingPositionDunker = new ArmPostion();
        public static final Translation2d DunkingZone = new Translation2d();
        public static final Translation2d SnipingZone = new Translation2d();
        public static final Translation2d[] ArmPositionInSpeakerZone = new Translation2d[]{DunkingZone, SnipingZone};


        public static class Shooter{
            public static final int shooterMotor1ID = 0;
            public static final int shooterMotor2ID = 1;

            public static final PIDFGains shooter1PID = new PIDFGains(0.5, 0, 0);
            public static final PIDFGains shooter2PID = new PIDFGains(0.5, 0, 0);

            public static final boolean shooter1Inverted = false;
            public static final boolean shooter2Inverted = false;

            public static final double shooterMaxPower = 1;
            
            public static final PIDFGains shooterPIDGains = new PIDFGains(0, 0, 0, 0, 0, 0);

            public static final double wheelRadius = 0.0508;

            public static final double frictionPowerParameterForGPVelocity = 1;//get from exp

            public static final double ShootToAmpPower = 0.4;
            public static final double ShootToAmpTime = 2;
        }

        public static final class IntakeConstants{
            public static final int intakeMotorID = 0;
            public static final boolean intakeMotorIsInverted = false;

            public static final double intakeMotorSpeed = 0.1;

            public static final double StallCurrent = 50;
            public static final int MaxStallTime = 100;

            public static final I2C.Port ColorSensorPort = I2C.Port.kOnboard;
            public static final int CloseProximity = 1500;
        }

        public static class MotorConstants{
            public static final int mainMotorID = 0;
            public static final int seconderyMotorID = 1;

            public static final PIDFGains mainPID = new PIDFGains(0.5, 0, 0);
            public static final PIDFGains seconderyPID = new PIDFGains(0.5, 0, 0);

            public static final boolean mainInverted = false;
            public static final boolean seconderyInverted = false;

            public static final double mainZeroOffset = 0;
            public static final double seconderyZeroOffset = 0;

            public static final double mainConvertionFactor = 1;
            public static final double seconderyConvecrtionFactor = 1;

            public static final double[] mainSoftLimits = new double[]{1, -1};
            public static final double[] seconderySoftLimits = new double[]{1, -1};

            public static final double mainMotortolarance = 0.1;
            public static final double seconderyMotorTolarance = 0.1;
        }
    }

    public static final class DriveTrain{
        /**
         * the name of the swerve modules by order
         */
        public static enum ModuleName{
            Front_Left,
            Front_Right,
            Back_Left,
            Back_Right
        }

        public static final class Global{
            public static final double maxAcceleration = 40; //the max xy acceleration of the robot in meter per second squared
            public static final double maxAccelerationRotation = 40; //the max rotation acceleration of the robot in omega radians per second squard

            public static final PIDFGains thetaCorrectionPID = new PIDFGains(0.4, 0.0, 0.0);

            public static final double chassisSpeedsDeadZone = 0.05;

            public static final double distanceBetweenWheels = 0.55; // the distance between each wheel in meters

            public static final double maxAllowableErrorInPostionMeters = 0; //the max value before pathplanner replans the path for the robot
            public static final double maxAllowableErrorSpikeInMeters = 100; //the max value of a pose spike before replan

            public static final double maxRotationSpeed = 6.27; //the max speed the robot can rotate in in radains per second
        }

        public static final class Drive{
            public static final PIDFGains driveMotorPID = new PIDFGains(0.4, 0.001, 0.001, 0.0, 0.22, 0); //the pid gains for the PID Controller of the drive motor, units are in 2048 per rotation
            public static final double freeWheelSpeedMetersPerSec = 4.75; //the max speed of the drive wheel in meters per second

            public static final double driveMotorMaxAcceleration = 4; //the max Acceleration of the wheel in meters / second squard
            public static final double driveMotorMaxJerk = 4.4; //the max jerk of the wheel in meters / second cubed

            public static final double wheelRadiusMeters = 0.0508; //the radius of the drive wheel in meters
            public static final double wheelCircumferenceMeters = wheelRadiusMeters * 2 * Math.PI; //the circumference of the drive wheel in meters
 
            public static final double driveGearRatio = 6.75; //the gear ratio between the drive motor and the wheel

        }

        public static final class Steer{
            public static final double steerGearRatio = 12.5; //the gear ratio between the steer motor and the module itself
            public static final PIDFGains steerMotorPID = new PIDFGains(0.4, 0, 0.1, 0, 0.0005, 0); //the pid gains for the PID Controller of the steer motor, units are in rotations

            public static final double maxVelocity = 1; //the max velocity of the modules steer aspect in module rotations per minute
            public static final double minVelocity = 0.1; //the min velocity of the modules steer aspect in module rotation per minute
            public static final double maxAcceleration = 1; //the max acceleration of the modules steer apsect in module rotations per minute per second


            public static final double front_left_absoluteEncoderZeroOffset = -130.95703125; // the offset between the absolute encoder reading on the front left module, in degrees
            public static final double front_right_absoluteEncoderZeroOffset = -153.193359375; // the offset between the absolute encoder on the front left module, in degrees
            public static final double back_left_absoluteEncoderZeroOffset = -56.07421875; // the offset between the absolute encoder on the back left module, in degrees
            public static final double back_right_absoluteEncoderZeroOffset = 104.326171875; // the offset between the absolute encoder on the back right module, in degrees

            // public static final double front_left_absoluteEncoderZeroOffset = 0; // use this to calibrate zero offsets
            // public static final double front_right_absoluteEncoderZeroOffset = 0; // use this to calibrate zero offsets
            // public static final double back_left_absoluteEncoderZeroOffset = 0; // use this to calibrate zero offsets
            // public static final double back_right_absoluteEncoderZeroOffset = 0; // use this to calibrate zero offsets

        }

        public static final class PathPlanner{
            public static final boolean planPathTostartingPointIfNotAtIt = true; //if pathplanner should plan a path to the starting point if the robot is not there
            public static final boolean enableDynamicReplanning = true; //if pathplanner should replan the path if the robot is beyond the tolarance or if the spike is too big
            public static final double pathErrorTolarance = 0.1; //the max error in position before pathPlaneer replans the path in meters
            public static final double pathErrorSpikeTolarance = 1; //the max postion spike before path planner replans the path

            public static final PIDFGains thetaPID = new PIDFGains(5, 0.0, 0.0); //the pid gains for the PID Controller of the robot angle, units are radians
            public static final PIDFGains XYPID = new PIDFGains(5, 0.0, 1); //the pid gains for the pid controller of the robot's postion (xy)
        }

        public static final class Vision{
            public static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(3));
            public static final Vector<N3> visionStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(50));
        }

        public static class SwerveModule{
            public static Translation2d[] moduleTranslations = new Translation2d[]
                {new Translation2d(Global.distanceBetweenWheels / 2, Global.distanceBetweenWheels / 2), new Translation2d(Global.distanceBetweenWheels / 2, -Global.distanceBetweenWheels / 2),
                 new Translation2d(-Global.distanceBetweenWheels / 2, Global.distanceBetweenWheels / 2), new Translation2d(-Global.distanceBetweenWheels / 2, -Global.distanceBetweenWheels / 2)};

            // public static Translation2d[] moduleTranslations = new Translation2d[]
            //     {new Translation2d(Global.distanceBetweenWheels / 2, -Global.distanceBetweenWheels / 2), new Translation2d(Global.distanceBetweenWheels / 2, Global.distanceBetweenWheels / 2),
            //      new Translation2d(-Global.distanceBetweenWheels / 2, -Global.distanceBetweenWheels / 2), new Translation2d(-Global.distanceBetweenWheels / 2, Global.distanceBetweenWheels / 2)};
            // the lcoation of the moudles by enum order comperd to the center of the robot

            public ModuleName moduleName; //the name of the moudle (enum)

            public int driveMotorID; // the CAN ID of the drive motor
            public int steerMotorID; // the CAN ID of the steer motr
            public int absoluteEncoderID; // the CAN ID of the absolute encoder (this case CanCoder)

            public boolean isSteerInverted; //wheter the steer motor output should be revesed
            public boolean isDriveInverted; //wheter the drive motor output should be reversed

            public double absoluteEncoderZeroOffset; //the offset between the cancoder and the module zero angle in degrees

            public Translation2d moduleTranslation; //the translation of the module realetive to the center of the robot

            public SwerveModule(ModuleName moduleName, int driveMotorID, int steerMotorID, int absoluteEncoderID, double absoluteEncoderZeroOffset, boolean isSteerInverted, boolean isDriveInverted){
                this.moduleName = moduleName;

                this.driveMotorID = driveMotorID;
                this.steerMotorID = steerMotorID;
                this.absoluteEncoderID = absoluteEncoderID;

                this.isDriveInverted = isDriveInverted;
                this.isSteerInverted = isDriveInverted;

                this.absoluteEncoderZeroOffset = absoluteEncoderZeroOffset;

                this.moduleTranslation = moduleTranslations[moduleName.ordinal()];
            }
        }

        public static final SwerveModule front_left = new SwerveModule(ModuleName.Front_Left, 2, 3, 10, Steer.front_left_absoluteEncoderZeroOffset, false, false);
        //^the constants of the front left module
        public static final SwerveModule front_right = new SwerveModule(ModuleName.Front_Right, 4, 5, 11, Steer.front_right_absoluteEncoderZeroOffset, false, false);
        //^the constants of the front right module
        public static final SwerveModule back_left = new SwerveModule(ModuleName.Back_Left, 6, 7, 12, Steer.back_left_absoluteEncoderZeroOffset, false, false);
        //^the constants of the back left module
        public static final SwerveModule back_right = new SwerveModule(ModuleName.Back_Right, 8, 9, 13, Steer.back_right_absoluteEncoderZeroOffset, false, false);
        //^the constants of the back right module
        
    }

    
}
