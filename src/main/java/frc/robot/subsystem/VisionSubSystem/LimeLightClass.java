// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.VisionSubSystem;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystem.VisionSubSystem.Vision.CameraInterface;
import frc.util.LimelightHelpers;
import frc.util.LimelightHelpers.LimelightResults;

/** Add your docs here. */
public class LimeLightClass implements CameraInterface{
    private LimelightResults latestResults;
    private double[] pythonResults = new double[6];
    private String cameraName;

    private CameraMode mode;
    private Transform3d cameraToRobot[];

    private Pose3d estimatedPose;
    private Translation2d objectToRobot;

    private double timeStamp;
    private double latency;
    

    public LimeLightClass(String cameraName, CameraLocation location) {
      this.cameraName = cameraName;
      latestResults = new LimelightResults();

      mode = CameraMode.AprilTags;

      cameraToRobot = Constants.Vision.cameraLocations[location.ordinal()];

      LimelightHelpers.setCameraPose_RobotSpace(
        cameraName,
        cameraToRobot[0].getX(),
        cameraToRobot[0].getY(),
        cameraToRobot[0].getZ(),
        cameraToRobot[0].getTranslation().getX(),
        cameraToRobot[0].getRotation().getY(),
        cameraToRobot[0].getRotation().getZ());

      estimatedPose = new Pose3d();
      objectToRobot = new Translation2d();

      timeStamp = 0;
      latency = 0;

      for(int i = 0; i < 6; i++){
        pythonResults[i] = 0;
      }
    }

    @Override
    public Pose3d getPose() {
      if(mode == CameraMode.AprilTags)
        return estimatedPose;
      else return null;
    }

    @Override
    public Translation2d getTranslationToTarget(){
      if(mode == CameraMode.Rings)
        return objectToRobot;
      else return null;
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
        objectToRobot = null;
        return;
      }

      timeStamp = latestResults.targetingResults.timestamp_RIOFPGA_capture;
      latency = latestResults.targetingResults.latency_capture * 100;

      if(mode == CameraMode.AprilTags)
        estimatedPose = latestResults.targetingResults.getBotPose3d();
      else{
        pythonResults = LimelightHelpers.getPythonScriptData(cameraName);
        objectToRobot = PhotonUtils.estimateCameraToTargetTranslation(
        PhotonUtils.calculateDistanceToTargetMeters(
          cameraToRobot[0].getZ(),
          Constants.Vision.gamePieceHeight,
          -cameraToRobot[0].getRotation().getY(),
          pythonResults[5]),
        Rotation2d.fromDegrees(pythonResults[6]));
      }
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
      LimelightHelpers.setPipelineIndex(cameraName, pipeLineIndex);
      if(pipeLineIndex == 0){
        mode = CameraMode.AprilTags;
        objectToRobot = null;
      }
      else{
        mode = CameraMode.Rings;
        estimatedPose = null;
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
      return 0;
    }
  }
