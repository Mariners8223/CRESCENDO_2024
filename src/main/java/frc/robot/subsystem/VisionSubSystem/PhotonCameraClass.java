// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.VisionSubSystem;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.VisionSubSystem.Vision.CameraInterface;

/** Add your docs here. */
public class PhotonCameraClass implements CameraInterface{
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private PhotonPipelineResult latestResult;
    private CameraMode mode;
    private boolean fieldLoaded;

    private Servo servo;

    private Transform3d cameraToRobot[] = new Transform3d[2];
    private Translation2d objectToRobot;

    private Pose3d estimatedPose;
    private double poseConfidence;

    private double timeStamp;
    private double latency;

    public PhotonCameraClass(String cameraName, CameraLocation location, int servoPort) {
      camera = new PhotonCamera(cameraName);

      try {
        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile),
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.Vision.cameraLocations[location.ordinal()][0]);

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        fieldLoaded = true;
      } catch (IOException exception) {
        fieldLoaded = false;
      }

      if(servoPort != -1){
        servo = new Servo(servoPort);
        servo.setPosition(0);
      }
      else servo = null;

      latestResult = new PhotonPipelineResult();

      estimatedPose = new Pose3d();
      objectToRobot = new Translation2d();

      mode = CameraMode.AprilTags;

      timeStamp = 0;
      latency = 0;
      poseConfidence = 0;

      cameraToRobot = Constants.Vision.cameraLocations[location.ordinal()];
    }

    public PhotonCameraClass(String cameraName, CameraLocation location){
      this(cameraName, location, -1);
    }

    @Override
    public Pose3d getPose() {
      if(mode == CameraMode.AprilTags && fieldLoaded)
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
      return latestResult.hasTargets();
    }

    @Override
    public void update() {
      latestResult = camera.getLatestResult();

      if(!latestResult.hasTargets()){
        timeStamp = 0;
        latency = 0;
        estimatedPose = null;
        poseConfidence = 0;
        objectToRobot = null;
        return;
      }

      timeStamp = latestResult.getTimestampSeconds();
      latency = latestResult.getLatencyMillis() * 100;

      if(mode == CameraMode.AprilTags && fieldLoaded){
        poseEstimator.setReferencePose(RobotContainer.driveBase.getPose());
        estimatedPose = poseEstimator.update().get().estimatedPose;
        poseConfidence = latestResult.getBestTarget().getPoseAmbiguity();
      }
      else objectToRobot = PhotonUtils.estimateCameraToTargetTranslation(
        PhotonUtils.calculateDistanceToTargetMeters(
        cameraToRobot[1].getZ(),
        Constants.Vision.gamePieceHeight,
        -cameraToRobot[1].getRotation().getY(),
        Units.degreesToRadians(latestResult.getBestTarget().getPitch())),
        Rotation2d.fromDegrees(latestResult.getBestTarget().getYaw())
      );
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
    public void setPipeLine(int pipeLineIndex) {
      camera.setPipelineIndex(pipeLineIndex);
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
    public double getPoseAmbiguty() {
      if(mode == CameraMode.AprilTags && fieldLoaded)
        return poseConfidence;
      else return 0;
    }
  }