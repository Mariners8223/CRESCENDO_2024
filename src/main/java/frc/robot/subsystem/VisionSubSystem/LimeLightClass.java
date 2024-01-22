// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.VisionSubSystem;

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
    private String cameraName;

    private Servo servo;

    private CameraMode mode;
    private Transform3d cameraToRobot[];

    private Pose3d estimatedPose;
    private Translation2d objectsToRobot[] = {new Translation2d()};

    private double timeStamp;
    private double latency;

    public LimeLightClass(String cameraName, CameraLocation location, int servoPort) {
      this.cameraName = cameraName;
      latestResults = new LimelightResults();

      mode = CameraMode.AprilTags;

      cameraToRobot = Constants.Vision.cameraLocations[location.ordinal()];

      if(servoPort != -1){
        servo = new Servo(servoPort);
        servo.setPosition(0);
      }
        
      LimelightHelpers.setCameraPose_RobotSpace(
        cameraName,
        cameraToRobot[0].getX(),
        cameraToRobot[0].getY(),
        cameraToRobot[0].getZ(),
        cameraToRobot[0].getTranslation().getX(),
        cameraToRobot[0].getRotation().getY(),
        cameraToRobot[0].getRotation().getZ());

      estimatedPose = new Pose3d();

      timeStamp = 0;
      latency = 0;
    }

    public LimeLightClass(String cameraName, CameraLocation location) {
      this(cameraName, location, -1);
    }

    @Override
    public Pose3d getPose() {
      if(mode == CameraMode.AprilTags)
        return estimatedPose;
      else return null;
    }

    @Override
    public Translation2d getTranslationToBestTarget(){
      if(mode == CameraMode.Rings)
        return objectsToRobot[0];
      else return null;
    }

    @Override
    public double getServoAngle(){
      return servo.getAngle();
    }

    @Override
    public boolean hasTarget() {
      return latestResults.targetingResults.valid;
    }

    @Override
    public void update() {
      latestResults = LimelightHelpers.getLatestResults(cameraName);

      if(!latestResults.targetingResults.valid){
        timeStamp = 0;
        latency = 0;
        estimatedPose = null;
        objectsToRobot = null;
        return;
      }

      timeStamp = latestResults.targetingResults.timestamp_RIOFPGA_capture;
      latency = latestResults.targetingResults.latency_capture * 100;

      if(mode == CameraMode.AprilTags)
        estimatedPose = latestResults.targetingResults.getBotPose3d();
      else{
        var targets = latestResults.targetingResults.targets_Retro;

        for(int i = 0; i < targets.length || i == 5; i++){

          objectsToRobot[i] = PhotonUtils.estimateCameraToTargetTranslation(PhotonUtils.calculateDistanceToTargetMeters(
          cameraToRobot[1].getZ(), Constants.Vision.gamePieceHeight / 2, cameraToRobot[1].getRotation().getY(), targets[i].tx),
          Rotation2d.fromDegrees(targets[i].ty)).plus(cameraToRobot[1].getTranslation().toTranslation2d()
          ).rotateBy(cameraToRobot[1].getRotation().toRotation2d());
      }
    }
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
      LimelightHelpers.setPipelineIndex(cameraName, pipeLineIndex);
      if(pipeLineIndex == 0){
        mode = CameraMode.AprilTags;
        objectsToRobot = null;
        if(servo != null)
          servo.setAngle(0);
      }
      else{
        mode = CameraMode.Rings;
        estimatedPose = null;
        if(servo != null)
          servo.setAngle(90);
      }
    }

    @Override
    public double getTimeStamp() {
      return timeStamp;
    }

    @Override
    public double getLatency() {
      return latency;
    }

    @Override
    public double getPoseAmbiguty() {
      return 1;
    }

    @Override
    public Translation2d[] getTranslationsToTargets() {
      if(mode == CameraMode.Rings)
        return objectsToRobot;
      else return null;
    }
  }
