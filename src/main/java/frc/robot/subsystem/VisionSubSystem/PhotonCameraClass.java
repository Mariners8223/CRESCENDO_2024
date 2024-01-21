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
    private PhotonCamera camera; //the camera
    private PhotonPoseEstimator poseEstimator; //the pose estimator
    private PhotonPipelineResult latestResult; //the latest result from the camera
    private CameraMode mode; //the mode the camera is in
    private boolean fieldLoaded; //if the field is loaded

    private Servo servo; //the servo to move the camera

    private Transform3d cameraToRobot[] = new Transform3d[2]; //the transform from the camera to the robot
    private Translation2d objectToRobot[] = {new Translation2d()}; //the translation from the object to the robot

    private Pose3d estimatedPose; //the estimated pose of the robot
    private double poseConfidence; //the confidence of the pose

    private double timeStamp; //the time stamp of the latest result
    private double latency; //the latency of the latest result


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
    public Translation2d getTranslationToBestTarget(){
      if(mode == CameraMode.Rings)
        return objectToRobot[0];
      else return null;
    }

    @Override
    public Translation2d[] getTranslationsToTargets(){
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
      else{
        var targets = latestResult.getTargets();

        for(int i = 0; i < targets.size() || i == 5; i++){
          objectToRobot[i] = PhotonUtils.estimateCameraToTargetTranslation(PhotonUtils.calculateDistanceToTargetMeters(
          cameraToRobot[1].getZ(), Constants.Vision.gamePieceHeight / 2, cameraToRobot[1].getRotation().getY(), targets.get(i).getPitch()),
          Rotation2d.fromDegrees(targets.get(i).getYaw())).plus(cameraToRobot[1].getTranslation().toTranslation2d()
          ).rotateBy(cameraToRobot[1].getRotation().toRotation2d());
        }
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
    public double getServoAngle(){
      if(servo != null)
        return servo.getAngle();
      else return 0;
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
      camera.setPipelineIndex(pipeLineIndex);
      if(pipeLineIndex == 0){
        mode = CameraMode.AprilTags;
        objectToRobot = null;
        if(servo != null)
          servo.setAngle(Units.radiansToDegrees(Constants.Vision.cameraLocations[CameraLocation.Front.ordinal()][0].getRotation().getY()));
      }
      else{
        mode = CameraMode.Rings;
        estimatedPose = null;
        if(servo != null)
          servo.setAngle(Units.radiansToDegrees(Constants.Vision.cameraLocations[CameraLocation.Front.ordinal()][1].getRotation().getY()));
      }
    }

    @Override
    public double getPoseAmbiguty() {
      if(mode == CameraMode.AprilTags && fieldLoaded)
        return poseConfidence;
      else return 0;
    }
  }