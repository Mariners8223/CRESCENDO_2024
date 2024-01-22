// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.VisionSubSystem;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.subsystem.VisionSubSystem.Vision.CameraInterface;
import frc.util.LimelightHelpers;
import frc.util.LimelightHelpers.LimelightResults;

/** Add your docs here. */
public class LimeLightClass implements CameraInterface{
    private LimelightResults latestResults;

    private Servo servo;

    private CameraMode mode;
    private Transform3d cameraToRobot[];

    private LimelightInputsAutoLogged inputs;

    @AutoLog
    public static class LimelightInputs{
      public String cameraName;
      public String location;
      public String mode;

      public boolean hasServo;

      public Pose3d estimatedPose;
      public Translation2d[] objectsToRobot;

      public double timeStamp;
      public double latency;
    }

    public LimeLightClass(String cameraName, CameraLocation location, int servoPort) {
      inputs.cameraName = cameraName;
      latestResults = new LimelightResults();

      mode = CameraMode.AprilTags;

      cameraToRobot = Constants.Vision.cameraLocations[location.ordinal()];

      if(servoPort != -1){
        servo = new Servo(servoPort);
        servo.setPosition(0);
        inputs.hasServo = true;
      }
      else{
        servo = null;
        inputs.hasServo = false;
      }
        
      LimelightHelpers.setCameraPose_RobotSpace(
        cameraName,
        cameraToRobot[0].getX(),
        cameraToRobot[0].getY(),
        cameraToRobot[0].getZ(),
        cameraToRobot[0].getTranslation().getX(),
        cameraToRobot[0].getRotation().getY(),
        cameraToRobot[0].getRotation().getZ());

      inputs.estimatedPose = Constants.Vision.rubbishPose;
      inputs.objectsToRobot = Constants.Vision.rubbishTranslation;

      inputs.timeStamp = 0;
      inputs.latency = 0;

      Logger.processInputs(inputs.cameraName, inputs);
    }

    public LimeLightClass(String cameraName, CameraLocation location) {
      this(cameraName, location, -1);
    }

    @Override
    public Pose3d getPose() {
      if(mode == CameraMode.AprilTags)
        return inputs.estimatedPose;
      else return Constants.Vision.rubbishPose;
    }

    @Override
    public Translation2d getTranslationToBestTarget(){
      if(mode == CameraMode.Rings)
        return inputs.objectsToRobot[0];
      else return Constants.Vision.rubbishTranslation[0];
    }
    
    @Override
    public Translation2d[] getTranslationsToTargets() {
      if(mode == CameraMode.Rings)
        return inputs.objectsToRobot;
      else return Constants.Vision.rubbishTranslation;
    }

    @Override
    public double getServoAngle(){
      if(servo == null) return 0;
      return servo.getAngle();
    }

    @Override
    public boolean hasTarget() {
      return latestResults.targetingResults.valid;
    }

    @Override
    public void update() {
      latestResults = LimelightHelpers.getLatestResults(inputs.cameraName);

      if(!latestResults.targetingResults.valid){
        inputs.timeStamp = 0;
        inputs.latency = 0;
        inputs.estimatedPose = Constants.Vision.rubbishPose;
        inputs.objectsToRobot = Constants.Vision.rubbishTranslation;
        Logger.processInputs(inputs.cameraName, inputs);
        return;
      }

      inputs.timeStamp = latestResults.targetingResults.timestamp_RIOFPGA_capture;
      inputs.latency = latestResults.targetingResults.latency_capture * 100;

      if(mode == CameraMode.AprilTags)
        inputs.estimatedPose = latestResults.targetingResults.getBotPose3d();
      else{
        var targets = latestResults.targetingResults.targets_Retro;

        for(int i = 0; i < targets.length || i == 5; i++){

          inputs.objectsToRobot[i] = PhotonUtils.estimateCameraToTargetTranslation(PhotonUtils.calculateDistanceToTargetMeters(
          cameraToRobot[1].getZ(), Constants.Vision.gamePieceHeight / 2, cameraToRobot[1].getRotation().getY(), targets[i].tx),
          Rotation2d.fromDegrees(targets[i].ty)).plus(cameraToRobot[1].getTranslation().toTranslation2d()
          ).rotateBy(cameraToRobot[1].getRotation().toRotation2d());
      }
      }

      Logger.processInputs(inputs.cameraName, inputs);
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
      LimelightHelpers.setPipelineIndex(inputs.cameraName, pipeLineIndex);
      if(pipeLineIndex == 0){
        mode = CameraMode.AprilTags;
        inputs.objectsToRobot = Constants.Vision.rubbishTranslation;
        if(servo != null)
          servo.setAngle(0);
      }
      else{
        mode = CameraMode.Rings;
        inputs.estimatedPose = Constants.Vision.rubbishPose;
        if(servo != null)
          servo.setAngle(90);
      }
    }

    @Override
    public double getTimeStamp() {
      return inputs.timeStamp;
    }

    @Override
    public double getLatency() {
      return inputs.latency;
    }

    @Override
    public double getPoseAmbiguty() {
      return 1;
    }
  }
