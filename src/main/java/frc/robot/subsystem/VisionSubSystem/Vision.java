// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.VisionSubSystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.VisionSubSystem.Vision.CameraInterface.CameraLocation;

public class Vision extends SubsystemBase {
  private CameraInterface[] cameras = new CameraInterface[Constants.Vision.numberOfCameras];
  // private List<CameraInterface> cameras = new ArrayList<>(Constants.Vision.);
  private Pose3d[] poses = new Pose3d[Constants.Vision.numberOfCameras];
  private double[] timeStamps = new double[Constants.Vision.numberOfCameras];
  private double[] latencies = new double[Constants.Vision.numberOfCameras];
  // private Translation2d[][] objectsToRobot = new Translation2d[Constants.Vision.numberOfCameras][];
  private double[][] distanceTOObjects = new double[Constants.Vision.numberOfCameras][5];
  private double[][] angleToObjects = new double[Constants.Vision.numberOfCameras][5];

  /**
   * creates a new Vision subsystem with 4 cameras
   */
  public Vision() {
    cameras[0] = new PhotonCameraClass("back", CameraLocation.Back, 0, false, Constants.Vision.cameraLocations[0]);
    // cameras[1] = new LimeLightClass("limelight", CameraLocation.Front_Right, Constants.Vision.cameraLocations[1]);
    // cameras[1] = new PhotonCameraClass("front-right", CameraLocation.Front_Right, true, Constants.Vision.cameraLocations[2]);

    for(int i = 0; i < Constants.Vision.numberOfCameras; i++){
      poses[i] = Constants.Vision.rubbishPose;
      timeStamps[i] = 0;
      latencies[i] = 0;
      distanceTOObjects[i] = new double[5];
      angleToObjects[i] = new double[5];
    }
  }

  // public static Vision getInstance(){
  //   if(instance == null) instance = new Vision();
  //   return instance;
  //   // return null;
  // }

  /**
   * returns the detected pose of the robot from the specified camera (may be null if no target is detected or the camera is not in AprilTag mode)
   * @param location the location of the camera
   * @return the detected pose of the robot from the specified camera
   */
  public Pose3d getPose(CameraLocation location){
    // return cameras[location.ordinal()].getPose();
    for (CameraInterface camera : cameras) {
      if(camera != null)
      if(camera.getCameraLocation() == location) return camera.getPose();
    }
    return Constants.Vision.rubbishPose;
  }

  /**
   * returns the timestamp of the specified camera (may be 0 if no target is detected)
   * @param location the location of the camera
   * @return the timestamp of the specified camera
   */
  public double getTimeStamp(CameraLocation location){
    // return cameras[location.ordinal()].getTimeStamp();
    for (CameraInterface camera: cameras) {
      if(camera != null)
      if(camera.getCameraLocation() == location) return camera.getTimeStamp();
    }
    return 0;
  }

  /**
   * returns the latency of the specified camera (may be 0 if no target is detected)
   * @param location the location of the camera
   * @return the latency of the specified camera
   */
  public double getLatency(CameraLocation location){
    // return cameras[location.ordinal()].getLatency();
    for (CameraInterface camera : cameras) {
      if(camera != null)
      if(camera.getCameraLocation() == location) return camera.getLatency();
    }
    return 0;
  }

  // /**
  //  * retruns the translation to the target from the specified camera (may be null if no target is detected or the camera is not in Rings mode)
  //  * @param location the location of the camera
  //  * @return the translation to the target from the specified camera
  //  */
  // public Translation2d getbestObjectToCamera(CameraLocation location){
  //   // return cameras[location.ordinal()].getTranslationToBestTarget();
  //   for (CameraInterface camera : cameras) {
  //     if(camera != null)
  //     if(camera.getCameraLocation() == location) return camera.getTranslationToBestTarget();
  //   }
  //   return Constants.Vision.rubbishTranslation[0];
  // }

  public double getDistanceToBestObject(CameraLocation location){
    // return cameras[location.ordinal()].getDistanceToBestTarget();
    for (CameraInterface camera : cameras) {
      if(camera != null)
      if(camera.getCameraLocation() == location) return camera.getDistanceToBestTarget();
    }
    return -1000;
  }

  public double getAngleToBestObject(CameraLocation location){
    // return cameras[location.ordinal()].getAngleToBestTarget();
    for (CameraInterface camera : cameras) {
      if(camera != null)
      if(camera.getCameraLocation() == location) return camera.getAngleToBestTarget();
    }
    return -1000;
  }

  public double[] getDistanceToObjects(CameraLocation location){
    // return cameras[location.ordinal()].getDistanceToTargets();
    for (CameraInterface camera : cameras) {
      if(camera != null)
      if(camera.getCameraLocation() == location) return camera.getDistanceToTargets();
    }
    return Constants.Vision.rubbishDistance;
  }

  public double[] getAngleToObjects(CameraLocation location){
    // return cameras[location.ordinal()].getAngleToTargets();
    for (CameraInterface camera : cameras) {
      if(camera != null)
      if(camera.getCameraLocation() == location) return camera.getAngleToTargets();
    }
    return Constants.Vision.rubbishDistance;
  }

  /**
   * returns if the specified camera has a target
   * @param location the location of the camera
   * @return if the specified camera has a target
   */
  public boolean hasTarget(CameraLocation location){
    for (CameraInterface camera : cameras) {
      if(camera != null)
      if(camera.getCameraLocation() == location) return camera.hasTarget();
    }
    return false;
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

  public void setPipelineIndex(CameraLocation location, int index){
    for (CameraInterface camera : cameras) {
      if(camera != null)
      if(camera.getCameraLocation() == location) camera.setPipeLine(index);
    }
  }

  /**
   * gets all the latencies of the cameras (may be 0 if no target is detected)
   * @return all the latencies of the cameras
   */
  public double[] getLatencies(){
    return latencies;
  }
  
  @Override
  public void periodic() {
    // System.out.println("sss");
    for(int i = 0; i < Constants.Vision.numberOfCameras; i++){
      cameras[i].update();
      poses[i] = cameras[i].getPose();
      timeStamps[i] = cameras[i].getTimeStamp();
      latencies[i] = cameras[i].getLatency();
      distanceTOObjects[i] = cameras[i].getDistanceToTargets();
      angleToObjects[i] = cameras[i].getAngleToTargets();
      // System.out.println("safbjhsabgksabg sajhg");

      if(poses[i] != Constants.Vision.rubbishPose) RobotContainer.driveBase.addVisionMesrument(poses[i].toPose2d(), timeStamps[i]);
      
    }    
  }




  public interface CameraInterface {
    public enum CameraLocation{
      Front_Left,
      Front_Right,
      Right,
      Back,
      Left
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

    // /**
    //  * returns the translation to the target from the camera (may be null if no target is detected or the camera is not in Rings mode)
    //  * @return the translation to the target from the camera
    //  */
    // public Translation2d getTranslationToBestTarget();

    /**
     * gets the distance to the best target from the camera (may be -1000 if no target is detected)
     * @return the distance to the best target from the camera (in meters)
     */
    public double getDistanceToBestTarget();

    /**
     * gets the angle to the best target from the camera (may be -1000 if no target is detected)
     * @return the angle to the best target from the camera (in degrees)
     */
    public double getAngleToBestTarget();

    /**
     * gets the distance to the targets from the camera (may be null if no target is detected)
     * @return the distance to the targets from the camera (in meters)
     */
    public double[] getDistanceToTargets();

    /**
     * gets the angle to the targets from the camera (may be null if no target is detected)
     * @return the angle to the targets from the camera (in degrees)
     */
    public double[] getAngleToTargets();

    // /**
    //  * returns the translations to the targets from the camera (may be null if no target is detected or the camera is not in Rings mode)
    //  * @return the translations to the targets from the camera
    //  */
    // public Translation2d[] getTranslationsToTargets();

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
     * return the physical location of the camera
     * @return the physical location of the camera
     */
    public CameraLocation getCameraLocation();

    /**
     * returns the servo angle of the camera (may be 0 if there is no servo)
     * @return the servo angle of the camera (in degrees)
     */
    public double getServoAngle();
  }

  

  
}
