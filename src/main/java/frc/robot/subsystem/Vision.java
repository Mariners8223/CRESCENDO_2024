// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystem.Vision.CameraInterface.CameraLocation;
import frc.util.LimelightHelpers;
import frc.util.LimelightHelpers.LimelightResults;

public class Vision extends SubsystemBase {

  CameraInterface[] cameras = new CameraInterface[4];

  /** Creates a new Vision. */
  public Vision() {
    cameras[0] = new LimeLightClass(Constants.Vision.LimeLight.frontCameraName, CameraLocation.Front);
    cameras[1] = new PhotonCameraClass(Constants.Vision.PhotonVision.rightCameraName, CameraLocation.Right);
    cameras[2] = new LimeLightClass(Constants.Vision.LimeLight.backCameraName, CameraLocation.Back);
    cameras[3] = new PhotonCameraClass(Constants.Vision.PhotonVision.leftCameraName, CameraLocation.Left);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public interface CameraInterface {
    public enum CameraLocation{
      Front, Right, Back, Left
    }

    public enum CameraMode{
      AprilTags, Rings
    }

    public Pose3d getPose();
    public boolean hasTarget();
    public void update();
    public void setPipeLine(int pipeLine);
    // public Transform3d getObjectToRobot();
    public double getTimeStamp();
    public double getLatency();
  }

  private class PhotonCameraClass implements CameraInterface{
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private PhotonPipelineResult latestResult;
    private CameraMode mode;

    private Pose3d estimatedPose;
    private Transform3d objectToRobot;

    private double timeStamp;
    private double latency;

    public PhotonCameraClass(String cameraName, CameraLocation location) {
      camera = new PhotonCamera(cameraName);

      try {
        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile),
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.Vision.cameraLocations[location.ordinal()]);

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
      } catch (Exception e) {
        // TODO: handle exception
      }

      latestResult = new PhotonPipelineResult();

      estimatedPose = new Pose3d();
      objectToRobot = new Transform3d();

      mode = CameraMode.AprilTags;

      timeStamp = 0;
      latency = 0;
    }

    @Override
    public Pose3d getPose() {
      return estimatedPose;
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
        objectToRobot = null;
        return;
      }

      timeStamp = latestResult.getTimestampSeconds();
      latency = latestResult.getLatencyMillis() * 100;

      if(mode == CameraMode.AprilTags)
        estimatedPose = poseEstimator.update().get().estimatedPose;
      else objectToRobot = latestResult.getBestTarget().getBestCameraToTarget();
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

    public Transform3d getObjectToRobot() {
      return objectToRobot;
    }

  }

  private class LimeLightClass implements CameraInterface{
    private LimelightResults latestResults;
    private String cameraName;

    private CameraMode mode;

    private Pose3d estimatedPose;
    private double[] objectToRobot;

    private double timeStamp;
    private double latency;
    

    public LimeLightClass(String cameraName, CameraLocation location) {
      this.cameraName = cameraName;
      latestResults = new LimelightResults();

      mode = CameraMode.AprilTags;

      Transform3d cameraToRobot = Constants.Vision.cameraLocations[location.ordinal()];
      LimelightHelpers.setCameraPose_RobotSpace(
        cameraName,
        cameraToRobot.getX(),
        cameraToRobot.getY(),
        cameraToRobot.getZ(),
        cameraToRobot.getTranslation().getX(),
        cameraToRobot.getRotation().getY(),
        cameraToRobot.getRotation().getZ());
    }

    @Override
    public Pose3d getPose() {
      return estimatedPose;
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
      else objectToRobot = LimelightHelpers.getPythonScriptData(cameraName);
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
  
    
  }
}
