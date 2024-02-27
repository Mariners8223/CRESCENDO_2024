// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.subsystem.Arm.Arm.ArmPosition;
import frc.util.PIDFGains;

/** Add yo
import frc.util.PIDFGains;

/** Add your docs here. */
public class Constants {
    public static void SwapToRed(){
        Speaker.SpeakerTranslation = new Translation3d(Speaker.FieldXLength - Speaker.SpeakerTranslation.getX(),
         Speaker.SpeakerTranslation.getY(), Speaker.SpeakerTranslation.getZ());
        Speaker.ampTranslation = new Translation3d(Speaker.FieldXLength - Speaker.SpeakerTranslation.getX(), 
         Speaker.SpeakerTranslation.getY(), Speaker.SpeakerTranslation.getZ());
        //TODO: switch climb related positions - x = fieldx - x, rotation = 180 - rotation
        //and zones
    }
    public static final List<Translation2d> robotZones = new ArrayList<Translation2d>() {
        //TODO: add robot zones
    };
    public static final double gGravity_phisics = 9.81;

    public static final Pose2d AmpPose = new Pose2d(1.829, 8.46, Rotation2d.fromDegrees(90));
    public static final Pose2d RobotShootingToAMPPosition = new Pose2d(1.83, 7.71, Rotation2d.fromDegrees(-90));
    // public static final Translation3d SpeakerTranslation = new Translation3d(Units.inchesToMeters(-1.5), 5.55, 2.03);

    public static final class Logger{
        public static final String save_location = ""; //add save lcoation (probably a usb stick so /u)
    }

    public static final class Controllers{
        public static final double joyStickDeadBand = 0.05;
        public static final double triggerDeadBand = 0.1;
    }

    public static final class Vision{

        // public static final Translation2d[] rubbishTranslation = {new Translation2d(-20, -20)};
        public static final double[] rubbishDistance = new double[]{-1};
        public static final double[] rubbishAngle = new double[]{-1000};
        public static final Pose3d rubbishPose = new Pose3d(new Pose2d(new Translation2d(-20, -20), new Rotation2d(69)));

        public static final PIDFGains aimToRingPID = new PIDFGains(0.5, 0, 0, 0, 0, 0);
        public static final double aimToRingToleranceDegrees = 2;

        public static final int numberOfCameras = 2;

            public static final Transform3d[][] cameraLocations = Constants.createCameraTransforms();

            public static final double gamePieceHeight = 0.05; //the height of the game piece in meters

        public static final class PhotonVision{
            public static final String rightCameraName = "Right Camera"; //the name of the right camera
            public static final String leftCameraName = "Left Camera"; //the name of the left camera

        }

        public static final class LimeLight{
            public static final String frontCameraName = "Front Camera"; //the name of the front camera
            public static final String backCameraName = "Back Camera"; //the name of the back camera
        }
    }
    
    // public static class AutoConstants{
    //     public static ArmPostion FastShootPose = new ArmPostion();

    public static final class Speaker{
            public static final double FieldYLength = Units.inchesToMeters(323.00);
            public static final double FieldXLength = 16.45;

            public static final double SpeakerLength = 1.05;
            public static final double SpeakerMiddleLocationY = Units.inchesToMeters(218.42);
            public static final double SpeakerBottomLocationY = Units.inchesToMeters(218.42) - SpeakerLength/2;
            public static final double SpeakerIsCenterRatioReverse = 1 - SpeakerLength/(2*(FieldYLength - SpeakerMiddleLocationY));
            public static final double SpeakerCenterLocationX = Units.inchesToMeters(-1.5);// - 0.75;
            public static final double RatioFieldToSpeakerReverse = 1 - (SpeakerLength/FieldYLength);
            public static final double SpeakerIsCenterRatioBottomLocation = FieldYLength - 2*(FieldYLength - SpeakerMiddleLocationY);

            public static Translation3d ampTranslation = new Translation3d(3, 8, 0);
            public static Translation3d SpeakerTranslation = new Translation3d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), 2.03);//z = 2.03
    }

    public static final class Zone1{//DIS NOT TRUE
        public static final double maxAngle = 90;
        public static final double minAngle = 60;
    }

    // public static final class Arm {
    //     public static Pose2d MiddleNote = new Pose2d(2.52, 5.56, Rotation2d.fromDegrees(0));
    //     public static Pose2d UpperNote = new Pose2d(2.89, 6.61, Rotation2d.fromDegrees(90));
    //     public static Pose2d LowerNote = new Pose2d(2.52, 4.1, Rotation2d.fromDegrees(-31.35));
    // }

    public static final class Arm{
        public static final double armLengthMeters = 0.46;
        public static final double shooterAndIntakeLengthMeters = 0.361;

        public static final double ShootingPowerToSpeaker = 0.5;

        public static final double mainPivotDistanceFromCenterMeters = 0.113;
        public static final double armHeightFromFrameMeters = 0.245;
        public static final double SecondaryMotorDistanceFromShooterMeters = 0.06;
    
        public static final double SpeakerLength = 1.10;//done
        public static final double SpeakerBottomLocationY = 5.00;//done
        public static final double SpeakerMidlleLocationY = 5.55;//done
        public static final double FieldYLength = Units.inchesToMeters(323.00);//a little more or a little less than 8.2 meters
        public static final double SpeakerIsCenterRatio = 11.0/53.0;//SpeakerLength/(2*(FieldYLength - SpeakerMidlleLocationY));
        public static final double SpeakerIsCenterRatioBottomLocation = 2.9;//FieldYLength - 2*(FieldYLength - SpeakerMidlleLocationY);
        public static final ArmPosition FloorPosition = new ArmPosition(0, 0, 0); // In radians
        public static final ArmPosition SourcePosition = new ArmPosition(0, 0, 0);

        public static final ArmPosition freeMovementPosition = new ArmPosition(0, Math.sin(Units.rotationsToRadians(0.13)) * armLengthMeters, 0);

        // public static final double SpeakerHeight = 2.31;//meter

        public static final ArmPosition AmpArmPosition = new ArmPosition();
        
        //TODO: defult ArmPositions for aiming
        public static final ArmPosition Zone1_ArmPosition = new ArmPosition(0, 0, 0);//main motor location for zone 1
        public static final ArmPosition Zone2_ArmPosition = new ArmPosition(0, 0, 0);//main motor location for zone 2
        public static final ArmPosition QuikShotPosition = new ArmPosition(0, 0, 0);//main motor location at flor
        public static final double EndOfZone1 = 3.2;//TODO: the distince from the speaker right before the lazer equasion is not relevent

        public static class Motors{
            public static final int mainMotorID = 15;
            public static final int secondaryMotorID = 16;

            public static final PIDFGains mainPID = new PIDFGains(10, 0.02, 0, 0, 0.005, 0.02);
            public static final PIDFGains secondaryPID = new PIDFGains(3.5, 0, 0, 0, 0.01, 0.002);

            public static final boolean mainInverted = false;
            public static final boolean secondaryInverted = false;

            public static final double mainZeroOffset = 0.4647;
             public static final double secondaryZeroOffset = 0.1473289;
            // public static final double mainZeroOffset = 0;
            // public static final double secondaryZeroOffset = 0;

            public static final double[] mainSoftLimits = new double[]{0.3206787109375, -0.0224609375};//was 0.35, -0.04
            public static final double[] secondarySoftLimits = new double[]{0.5560302734375, 0.01};

            public static final double[] mainMaxOutputs = new double[]{0.15, -0.05};
            public static final double[] secondaryMaxOutputs = new double[]{0.1, -0.05};

            public static final double mainConversionFactor = 150;
            public static final double secondaryConversionFactor = 121.5;
            public static final boolean mainEncoderInverted = false;
            public static final boolean secondaryEncoderInverted = false;

        }
    }

    public static class Shooter {
        public static final int shooterMotor1ID = 19;
        public static final int shooterMotor2ID = 18;

        public static final PIDFGains shooter2PID = new PIDFGains(0.001, 0, 0.0005, 0.00015, 0, 0); //TODO: get the real value
        public static final PIDFGains shooter1PID = new PIDFGains(0.001, 0, 0.001, 0.00015, 0.0, 0);

        public static final boolean shooter1Inverted = true;
        public static final boolean shooter2Inverted = false;

        public static final double shooterMaxPower = 0.9;

        public static final double wheelRadius = 0.0508;

        public static double frictionPowerParameterForGPVelocity = 0.6;//get from exp

        public static final double ShootToAmpPower = 0.4;
        public static final double ShootToAmpTime = 2;
        // public static final double RPMforShooterZone1 = 4000;
        // public static final double RPMforShooterZone2 = 5000;
        public static double GPAirTimeZone1 = 0.105;
        public static double GPAirTimeZone2 = 0.22;
    }

    public static final class Intake{
        public static final int intakeMotorID = 17;
        public static final boolean intakeMotorIsInverted = true;

        public static final double intakeMotorSpeed = 0.8;

        public static final double StallCurrent = 15;
        public static final int MaxStallTime = 30;

        public static final I2C.Port ColorSensorPort = I2C.Port.kMXP;
        public static final int CloseProximity = 30;

        public static final double secondaryIntakeAngle = 0.36;
        public static final double mainIntakeAngle = -0.03;
    }

    public static class Elevator{
        public static final double chainHeight = 72;

        public static final int railMotorID = 20;
        public static final int rollerMotorID = 21;

        public static final PIDFGains railMotorPIDF = new PIDFGains(0.2, 0.02, 0.02);

        public static final PIDFGains rollerMotorPIDF = new PIDFGains(0.1, 0.1, 0.1); //TODO: get the real value

        public static final double railMotorConvertionFactor = 3 * 5 * 9 * 2/ (4.65 * Math.PI); // 90 / (3 * Math.PI)
        public static final double rollerMotorConvertionFactor = 1; //TODO: get the real value

        public static final double railMotorTolarance = 0.1;
        public static final double rollerMotorTolarance = 0.01;

        public static final boolean isRailMotorInverted = true;
        public static final boolean isRollerMotorInverted = false;

        public static final int ClimbingMotorID = 1;//TODO: climb motor and sliding motor IDs
        public static final int SlidingMotorID = 1;

        public static final double ClimbingMotorOffset = 0;//TODO: climbing and sliding motor offset
        public static final double SlidingMotorOffset = 0;

        public static final double AirialMetersToRopeLength = 2.52/2.65;
        public static final double RopeLengthToMotorRotaions = 8/84;

        public static class SlidingPositions{
            public static List<Translation2d> SlidingPositions_MiddleRope = new ArrayList<Translation2d>(
                Arrays.asList(new Translation2d(4.389, 4.880),//rope 1 - top rope
                new Translation2d(4.44, 3.26),//rope 2 - bottom rope
                new Translation2d(5.87, 4.110)//rope 3 - middle pointing rope
            ));
            public static List<Translation2d> SlidingPositions_RightEdgeRope = new ArrayList<Translation2d>(//TODO: find locations
                Arrays.asList(new Translation2d(3.607, 4.429),//rope 1 - top rope
                new Translation2d(5.217, 2.888),//rope 2 - bottom rope
                new Translation2d(5.87, 4.971)//rope 3 - middle pointing rope
            ));
            public static List<Translation2d> SlidingPositions_LeftEdgeRope = new ArrayList<Translation2d>(//TODO: find locations
                Arrays.asList(new Translation2d(5.171, 5.331),//rope 1 - top rope
                new Translation2d(3.655, 3.790),//rope 2 - bottom rope
                new Translation2d(5.87, 3.249)//rope 3 - middle pointing rope
            ));

            public static List<Pose2d> InStageMiddleLocations_POSE2D = new ArrayList<Pose2d>(
                Arrays.asList(new Pose2d(4.11, 5.25, Rotation2d.fromDegrees(-60)),//rope 1 - top rope
                new Pose2d(4.12, 2.94, Rotation2d.fromDegrees(60)),//rope 2 - bottom rope
                new Pose2d(6.27, 4.09, Rotation2d.fromDegrees(180))//rope 3 - middle pointing rope
            ));
            public static List<Pose2d> UnderRopeMiddleLocations_POSE2D = new ArrayList<>(
                Arrays.asList(new Pose2d(SlidingPositions_MiddleRope.get(0).getX(), SlidingPositions_MiddleRope.get(0).getY(), Rotation2d.fromDegrees(-60)),//rope 1 - top rope
                new Pose2d(SlidingPositions_MiddleRope.get(1).getX(), SlidingPositions_MiddleRope.get(1).getY(), Rotation2d.fromDegrees(60)),//rope 2 - bottom rope
                new Pose2d(SlidingPositions_MiddleRope.get(2).getX(), SlidingPositions_MiddleRope.get(2).getY(), Rotation2d.fromDegrees(180))//rope 3 - middle pointing rope
            ));
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

            public static final PIDFGains thetaCorrectionPID = new PIDFGains(3, 0.0, 0.15);

            public static final double chassisSpeedsDeadZone = 0.05;

            public static final double distanceBetweenWheels = 0.55; // the distance between each wheel in meters

            public static final double maxAllowableErrorInPostionMeters = 0; //the max value before pathplanner replans the path for the robot
            public static final double maxAllowableErrorSpikeInMeters = 100; //the max value of a pose spike before replan

            public static final double maxRotationSpeed = 6.27; //the max speed the robot can rotate in in radains per second

            public static final double RobotHeightFromGround = 0.155;
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
            public static final double steerGearRatio = 12.5 * 3; //the gear ratio between the steer motor and the module itself
            // public static final double newGearRatio = steerGearRatio * 3;
            public static final PIDFGains steerMotorPID = new PIDFGains(0.4, 0, 0.1, 0, 0.0005, 0); //the pid gains for the PID Controller of the steer motor, units are in rotations

            public static final double maxVelocity = 1; //the max velocity of the modules steer aspect in module rotations per minute
            public static final double minVelocity = 0.1; //the min velocity of the modules steer aspect in module rotation per minute
            public static final double maxAcceleration = 1; //the max acceleration of the modules steer apsect in module rotations per minute per second

            // public static final double front_left_absoluteEncoderZeroOffset = -100.458984375; // the offset between the absolute encoder reading on the front left module, in degrees
            // public static final double front_right_absoluteEncoderZeroOffset = -92.197265625; // the offset between the absolute encoder on the front left module, in degrees
            // public static final double back_left_absoluteEncoderZeroOffset = 94.39453125; // the offset between the absolute encoder on the back left module, in degrees
            // public static final double back_right_absoluteEncoderZeroOffset = 92.900390625; // the offset between the absolute encoder on the back right module, in degrees

            public static final double front_left_absoluteEncoderZeroOffset = 0; // use this to calibrate zero offsets
            public static final double front_right_absoluteEncoderZeroOffset = 0; // use this to calibrate zero offsets
            public static final double back_left_absoluteEncoderZeroOffset = 0; // use this to calibrate zero offsets
            public static final double back_right_absoluteEncoderZeroOffset = 0; // use this to calibrate zero offsets

        }

        public static final class PathPlanner{
            public static final boolean planPathTostartingPointIfNotAtIt = true; //if pathplanner should plan a path to the starting point if the robot is not there
            public static final boolean enableDynamicReplanning = true; //if pathplanner should replan the path if the robot is beyond the tolarance or if the spike is too big
            public static final double pathErrorTolerance = 0.1; //the max error in position before pathPlaneer replans the path in meters
            public static final double pathErrorSpikeTolerance = 1; //the max postion spike before path planner replans the path

            public static final PIDFGains thetaPID = new PIDFGains(5, 0.0, 0.0); //the pid gains for the PID Controller of the robot angle, units are radians
            public static final PIDFGains XYPID = new PIDFGains(5, 0.0, 1); //the pid gains for the pid controller of the robot's postion (xy)
        }

        public static class SwerveModule{
            public static Translation2d[] moduleTranslations = new Translation2d[]
                {new Translation2d(Global.distanceBetweenWheels / 2, Global.distanceBetweenWheels / 2), new Translation2d(Global.distanceBetweenWheels / 2, -Global.distanceBetweenWheels / 2),
                 new Translation2d(-Global.distanceBetweenWheels / 2, Global.distanceBetweenWheels / 2), new Translation2d(-Global.distanceBetweenWheels / 2, -Global.distanceBetweenWheels / 2)};

            public ModuleName moduleName; //the name of the moudle (enum)

            public int driveMotorID; // the CAN ID of the drive motor
            public int steerMotorID; // the CAN ID of the steer motr
            public int absoluteEncoderID; // the CAN ID of the absolute encoder (this case CanCoder)

            public boolean isSteerInverted; //wheter the steer motor output should be revesed
            public boolean isDriveInverted; //wheter the drive motor output should be reversed
            public boolean isAbsEncoderInverted; //wheter the absolute encoder output should be reversed

            public double absoluteEncoderZeroOffset; //the offset between the cancoder and the module zero angle in degrees

            public Translation2d moduleTranslation; //the translation of the module realetive to the center of the robot

            public SwerveModule(ModuleName moduleName, int driveMotorID, int steerMotorID, int absoluteEncoderID, double absoluteEncoderZeroOffset, boolean isSteerInverted, boolean isDriveInverted, boolean isAbsEncoderInverted){
                this.moduleName = moduleName;

                this.driveMotorID = driveMotorID;
                this.steerMotorID = steerMotorID;
                this.absoluteEncoderID = absoluteEncoderID;

                this.isDriveInverted = isDriveInverted;
                this.isSteerInverted = isSteerInverted;
                this.isAbsEncoderInverted = isAbsEncoderInverted;

                this.absoluteEncoderZeroOffset = absoluteEncoderZeroOffset;

                this.moduleTranslation = moduleTranslations[moduleName.ordinal()];
            }
        }

        public static final SwerveModule front_left = new SwerveModule(ModuleName.Front_Left, 2, 3, 10, Steer.front_left_absoluteEncoderZeroOffset, false, false, true);
        //^the constants of the front left module
        public static final SwerveModule front_right = new SwerveModule(ModuleName.Front_Right, 4, 5, 11, Steer.front_right_absoluteEncoderZeroOffset, false, false, true);
        //^the constants of the front right module
        public static final SwerveModule back_left = new SwerveModule(ModuleName.Back_Left, 6, 7, 12, Steer.back_left_absoluteEncoderZeroOffset, false, false, true);
        //^the constants of the back left module
        public static final SwerveModule back_right = new SwerveModule(ModuleName.Back_Right, 8, 9, 13, Steer.back_right_absoluteEncoderZeroOffset, false, false, true);
        //^the constants of the back right module
        
    }


    private static Transform3d[][] createCameraTransforms(){
        return new Transform3d[][]{
                {new Transform3d( //camera 1 aprilTag postion
                -0.27,
                -0.13,
                0.06,
                new Rotation3d(
                0,
                Units.degreesToRadians(19),
                Math.PI)),

                new Transform3d( //camera 1 ring postion
                -0.27,
                -0.13,
                0.06,
                new Rotation3d(
                0,
                Units.degreesToRadians(-14),
                Math.PI
                )
                )},

                {
                new Transform3d( //camera 2 aprilTag postion
                0.45,
                0.33
                ,0.18,
                new Rotation3d(
                0,
                Units.degreesToRadians(18),
                Units.degreesToRadians(-30))),
                
                new Transform3d( //camera 2 aprilTag postion
                0.45,
                0.33
                ,0.18,
                new Rotation3d(
                0,
                Units.degreesToRadians(18),
                Units.degreesToRadians(-30)))
                },
                
                {
                new Transform3d( //camera 3 aprilTag postion
                0.35,
                -0.35,
                0.15,
                new Rotation3d(
                0,
                Units.degreesToRadians(-15),
                Units.degreesToRadians(11))),
            
                new Transform3d( //camera 3 aprilTag postion
                0.35,
                -0.35,
                0.15,
                new Rotation3d(
                0,
                Units.degreesToRadians(-15),
                Units.degreesToRadians(11)))
                },

                {
                new Transform3d( //camera 4 aprilTag postion
                0,
                0,0,
                new Rotation3d(
                0,
                0,
                0)),

                new Transform3d( //camera 4 ring postion
                0,
                0,
                0,
                new Rotation3d(
                0,
                0,
                0))
            }
            };
        }

    
}
