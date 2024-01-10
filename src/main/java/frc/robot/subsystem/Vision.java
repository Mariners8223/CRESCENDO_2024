// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;


import java.io.IOException;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
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
import frc.robot.RobotContainer;
import frc.robot.subsystem.Vision.CameraInterface.CameraLocation;
import frc.util.LimelightHelpers;
import frc.util.LimelightHelpers.LimelightResults;

public class Vision extends SubsystemBase {

  @AutoLog
  public static class VisonOutputs{
    Pose3d[] poses = new Pose3d[Constants.Vision.numberOfCameras];
    double[] timeStamps = new double[Constants.Vision.numberOfCameras];
    double[] latencies = new double[Constants.Vision.numberOfCameras];
    Translation2d[] translations = new Translation2d[Constants.Vision.numberOfCameras];
    double[] poseConfidences = new double[Constants.Vision.numberOfCameras];
    boolean[] hasTargets = new boolean[Constants.Vision.numberOfCameras];
  }

  private static Vision instance;

  CameraInterface[] cameras = new CameraInterface[Constants.Vision.numberOfCameras];
  Pose3d[] poses = new Pose3d[Constants.Vision.numberOfCameras];
  double[] timeStamps = new double[Constants.Vision.numberOfCameras];
  double[] latencies = new double[Constants.Vision.numberOfCameras];
  Translation2d[] translations = new Translation2d[Constants.Vision.numberOfCameras];
  double[] poseConfidences = new double[Constants.Vision.numberOfCameras];
  boolean[] hasTargets = new boolean[Constants.Vision.numberOfCameras];

  VisonOutputsAutoLogged outputs;

  /**
   * creates a new Vision subsystem with 4 cameras
   */
  private Vision() {
    cameras[0] = new LimeLightClass(Constants.Vision.LimeLight.frontCameraName, CameraLocation.Front);
    cameras[1] = new PhotonCameraClass(Constants.Vision.PhotonVision.rightCameraName, CameraLocation.Right);
    cameras[2] = new LimeLightClass(Constants.Vision.LimeLight.backCameraName, CameraLocation.Back);
    cameras[3] = new PhotonCameraClass(Constants.Vision.PhotonVision.leftCameraName, CameraLocation.Left);

    outputs = new VisonOutputsAutoLogged();

    for(int i = 0; i < Constants.Vision.numberOfCameras; i++){
      poses[i] = new Pose3d();
      timeStamps[i] = 0;
      latencies[i] = 0;
      translations[i] = new Translation2d();
      poseConfidences[i] = 0;
      hasTargets[i] = false;
    }

    outputs.poses = poses;
    outputs.timeStamps = timeStamps;
    outputs.latencies = latencies;
    outputs.translations = translations;
    outputs.poseConfidences = poseConfidences;
    outputs.hasTargets = hasTargets;    
  }

  public static Vision getInstance(){
    if(instance == null) instance = new Vision();
    return instance;
  }

  /**
   * returns the detected pose of the robot from the specified camera (may be null if no target is detected or the camera is not in AprilTag mode)
   * @param location the location of the camera
   * @return the detected pose of the robot from the specified camera
   */
  public Pose3d getPose(CameraLocation location){
    return cameras[location.ordinal()].getPose();
  }

  /**
   * returns the timestamp of the specified camera (may be 0 if no target is detected)
   * @param location the location of the camera
   * @return the timestamp of the specified camera
   */
  public double getTimeStamp(CameraLocation location){
    return cameras[location.ordinal()].getTimeStamp();
  }

  /**
   * returns the latency of the specified camera (may be 0 if no target is detected)
   * @param location the location of the camera
   * @return the latency of the specified camera
   */
  public double getLatency(CameraLocation location){
    return cameras[location.ordinal()].getLatency();
  }

  /**
   * retruns the translation to the target from the specified camera (may be null if no target is detected or the camera is not in Rings mode)
   * @param location the location of the camera
   * @return the translation to the target from the specified camera
   */
  public Translation2d getTranslation(CameraLocation location){
    return cameras[location.ordinal()].getTranslationToTarget();
  }

  /**
   * returns if the specified camera has a target
   * @param location the location of the camera
   * @return if the specified camera has a target
   */
  public boolean hasTarget(CameraLocation location){
    return cameras[location.ordinal()].hasTarget();
  }

  /**
   * gets all the poses of the cameras (may be null if no target is detected or the cameras is not in AprilTag mode)
   * @return all the poses of the cameras
   */
  public Pose3d[] getposes(){
    return poses;
  }

  /**
   * gets all the timestamps of the cameras (may be 0 if no target is detected)
   * @return all the timestamps of the cameras
   */
  public double[] getTimeStamps(){
    return timeStamps;
  }

  /**
   * gets all the latencies of the cameras (may be 0 if no target is detected)
   * @return all the latencies of the cameras
   */
  public double[] getLatencies(){
    return latencies;
  }

  /**
   * gets all the translations to the target from the cameras (may be null if no target is detected or the cameras is not in Rings mode)
   * @return all the translations to the target from the cameras
   */
  public Translation2d[] getTranslations(){
    return translations;
  }

  public double[] getPoseAmbiguitys(){
    return poseConfidences;
  }
  
  @Override
  public void periodic() {
    for(int i = 0; i < Constants.Vision.numberOfCameras; i++){
      cameras[i].update();
      poses[i] = cameras[i].getPose();
      timeStamps[i] = cameras[i].getTimeStamp();
      latencies[i] = cameras[i].getLatency();
      translations[i] = cameras[i].getTranslationToTarget();
      hasTargets[i] = cameras[i].hasTarget();
      poseConfidences[i] = cameras[i].getPoseAmbiguty();

      if(poses[i] != null) outputs.poses[i] = poses[i];
      if(translations[i] != null) outputs.translations[i] = translations[i];
    }

    outputs.hasTargets = hasTargets;
    outputs.latencies = latencies;
    outputs.poseConfidences = poseConfidences;
    outputs.timeStamps = timeStamps;

    Logger.processInputs(getName(), outputs);
  }




  public interface CameraInterface {
    public enum CameraLocation{
      Front, Right, Back, Left
    }

    public enum CameraMode{
      AprilTags, Rings
    }

    /**
     * returns the pose of the robot from the camera (may be null if no target is detected or the camera is not in AprilTag mode)
     * @return the pose of the robot from the camera
     */
    public Pose3d getPose();

    /**
     * returns if the camera has a target
     * @return if the camera has a target
     */
    public boolean hasTarget();

    /**
     * updates the camera
     */
    public void update();

    /**
     * sets the pipeline of the camera
     * @param pipeLineIndex the index of the pipeline
     */
    public void setPipeLine(int pipeLineIndex);

    /**
     * returns the translation to the target from the camera (may be null if no target is detected or the camera is not in Rings mode)
     * @return the translation to the target from the camera
     */
    public Translation2d getTranslationToTarget();

    /**
     * returns the timestamp of the camera (may be 0 if no target is detected)
     * @return the timestamp of the camera
     */
    public double getTimeStamp();

    /**
     * returns the latency of the camera (may be 0 if no target is detected)
     * @return the latency of the camera
     */
    public double getLatency();

    /**
     * returns the pose confidence of the camera (may be 0 if no target is detected)
     * @return the pose confidence of the camera
     */
    public double getPoseAmbiguty();
  }

  private class PhotonCameraClass implements CameraInterface{
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private PhotonPipelineResult latestResult;
    private CameraMode mode;
    private boolean fieldLoaded;

    private Transform3d cameraToRobot;
    private Translation2d objectToRobot;

    private Pose3d estimatedPose;
    private double poseConfidence;

    private double timeStamp;
    private double latency;

    public PhotonCameraClass(String cameraName, CameraLocation location) {
      camera = new PhotonCamera(cameraName);

      try {
        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile),
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.Vision.cameraLocations[location.ordinal()]);

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        fieldLoaded = true;
      } catch (IOException exception) {
        fieldLoaded = false;
      }

      latestResult = new PhotonPipelineResult();

      estimatedPose = new Pose3d();
      objectToRobot = new Translation2d();

      mode = CameraMode.AprilTags;

      timeStamp = 0;
      latency = 0;
      poseConfidence = 0;

      cameraToRobot = Constants.Vision.cameraLocations[location.ordinal()];
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

    @Override
    public double getPoseAmbiguty() {
      if(mode == CameraMode.AprilTags && fieldLoaded)
        return poseConfidence;
      else return 0;
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

    @Override
    public double getPoseAmbiguty() {
      return 0;
    }
  }
}
