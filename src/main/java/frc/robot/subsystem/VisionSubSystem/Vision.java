// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.VisionSubSystem;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.VisionSubSystem.Vision.CameraInterface.CameraLocation;

public class Vision extends SubsystemBase {

  @AutoLog
  public static class VisonOutputs{
    Pose3d[] poses = new Pose3d[Constants.Vision.numberOfCameras];
    double[] timeStamps = new double[Constants.Vision.numberOfCameras];
    double[] latencies = new double[Constants.Vision.numberOfCameras];
    Translation2d[][] objectsToCameras = new Translation2d[Constants.Vision.numberOfCameras][];
    double[] poseConfidences = new double[Constants.Vision.numberOfCameras];
    boolean[] hasTargets = new boolean[Constants.Vision.numberOfCameras];
  }

  private static Vision instance;

  CameraInterface[] cameras = new CameraInterface[Constants.Vision.numberOfCameras];
  Pose3d[] poses = new Pose3d[Constants.Vision.numberOfCameras];
  double[] timeStamps = new double[Constants.Vision.numberOfCameras];
  double[] latencies = new double[Constants.Vision.numberOfCameras];
  Translation2d[][] objectsToCameras = new Translation2d[Constants.Vision.numberOfCameras][];
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
      objectsToCameras[i][0] = new Translation2d();
      poseConfidences[i] = 0;
      hasTargets[i] = false;
    }

    outputs.poses = poses;
    outputs.timeStamps = timeStamps;
    outputs.latencies = latencies;
    outputs.objectsToCameras = objectsToCameras;
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
  public Translation2d getbestObjectToCamera(CameraLocation location){
    return objectsToCameras[location.ordinal()][0];
  }

  public Translation2d[] getObjectsToCamera(CameraLocation location){
    return objectsToCameras[location.ordinal()];
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
      objectsToCameras[i] = cameras[i].getTranslationsToTargets();
      hasTargets[i] = cameras[i].hasTarget();
      poseConfidences[i] = cameras[i].getPoseAmbiguty();

      if(poses[i] != null){
        outputs.poses[i] = poses[i];
        RobotContainer.driveBase.addVisionMesrument(poses[i].toPose2d(), timeStamps[i]);
      }
      if(objectsToCameras[i] != null) outputs.objectsToCameras[i] = objectsToCameras[i];
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
    public Translation2d getTranslationToBestTarget();

    /**
     * returns the translations to the targets from the camera (may be null if no target is detected or the camera is not in Rings mode)
     * @return the translations to the targets from the camera
     */
    public Translation2d[] getTranslationsToTargets();

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

    /**
     * returns the servo angle of the camera (may be 0 if there is no servo)
     * @return the servo angle of the camera (in degrees)
     */
    public double getServoAngle();
  }

  

  
}
