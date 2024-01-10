// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;


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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystem.Vision.CameraInterface.CameraLocation;
import frc.util.LimelightHelpers;
import frc.util.LimelightHelpers.LimelightResults;

public class Vision extends SubsystemBase {

  CameraInterface[] cameras = new CameraInterface[4];
  Pose3d[] poses = new Pose3d[4];
  double[] timeStamps = new double[4];
  double[] latencies = new double[4];
  Translation2d[] translations = new Translation2d[4];

  /** Creates a new Vision. */
  public Vision() {
    cameras[0] = new LimeLightClass(Constants.Vision.LimeLight.frontCameraName, CameraLocation.Front);
    cameras[1] = new PhotonCameraClass(Constants.Vision.PhotonVision.rightCameraName, CameraLocation.Right);
    cameras[2] = new LimeLightClass(Constants.Vision.LimeLight.backCameraName, CameraLocation.Back);
    cameras[3] = new PhotonCameraClass(Constants.Vision.PhotonVision.leftCameraName, CameraLocation.Left);
    
  }

  public Pose3d getPose(CameraLocation location){
    return cameras[location.ordinal()].getPose();
  }

  public Pose3d[] getposes(){
    return poses;
  }

  public double[] getTimeStamps(){
    return timeStamps;
  }

  public double[] getLatencies(){
    return latencies;
  }

  public Translation2d[] getTranslations(){
    return translations;
  }
  
  @Override
  public void periodic() {
    for(int i = 0; i < 4; i++){
      cameras[i].update();
      poses[i] = cameras[i].getPose();
      timeStamps[i] = cameras[i].getTimeStamp();
      latencies[i] = cameras[i].getLatency();
      translations[i] = cameras[i].getTranslationToTarget();
    }
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
    public void setPipeLine(int pipeLineIndex);
    public Translation2d getTranslationToTarget();
    public double getTimeStamp();
    public double getLatency();
  }

  private class PhotonCameraClass implements CameraInterface{
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private PhotonPipelineResult latestResult;
    private CameraMode mode;

    private Transform3d cameraToRobot;
    private Translation2d objectToRobot;

    private Pose3d estimatedPose;

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
      objectToRobot = new Translation2d();

      mode = CameraMode.AprilTags;

      timeStamp = 0;
      latency = 0;

      cameraToRobot = Constants.Vision.cameraLocations[location.ordinal()];
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
      return latestResult.hasTargets();
    }

    @Override
    public void update() {
      latestResult = camera.getLatestResult();

      if(!latestResult.hasTargets()){
        timeStamp = 0;
        latency = 0;
        estimatedPose = null;
        return;
      }

      timeStamp = latestResult.getTimestampSeconds();
      latency = latestResult.getLatencyMillis() * 100;

      if(mode == CameraMode.AprilTags)
        estimatedPose = poseEstimator.update().get().estimatedPose;
      else objectToRobot = PhotonUtils.estimateCameraToTargetTranslation(
        PhotonUtils.calculateDistanceToTargetMeters(
        cameraToRobot.getZ(),
        Constants.Vision.gamePieceHeight,
        -cameraToRobot.getRotation().getY(),
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
  }

  private class LimeLightClass implements CameraInterface{
    private LimelightResults latestResults;
    private double[] pythonResults = new double[6];
    private String cameraName;

    private CameraMode mode;
    private Transform3d cameraToRobot;

    private Pose3d estimatedPose;
    private Translation2d objectToRobot;

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

      estimatedPose = new Pose3d();
      objectToRobot = new Translation2d();

      timeStamp = 0;
      latency = 0;

      cameraToRobot = Constants.Vision.cameraLocations[location.ordinal()];

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
          cameraToRobot.getZ(),
          Constants.Vision.gamePieceHeight,
          -cameraToRobot.getRotation().getY(),
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
  }
}
