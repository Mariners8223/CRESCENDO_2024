// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.VisionSubSystem;

import java.io.IOException;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
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
    private Optional<EstimatedRobotPose> estimatedPose;
    private CameraMode mode; //the mode the camera is in

    private Servo servo; //the servo to move the camera

    private Transform3d cameraToRobot[] = new Transform3d[2]; //the transform from the camera to the robot

    private PhotonCameraInputsAutoLogged inputs;

    @AutoLog
    public static class PhotonCameraInputs{
      public String cameraName; //the name of the camera
      public String location; //the location of the camera 
      public String mode; //the mode of the camera (AprilTags or Rings)
      public boolean isModeSwitchable;

      public boolean isfieldLoaded; //if the field is loaded
      public boolean hasServo;

      public Pose3d estimatedPose; //the estimated pose of the robot
      public double[] distanceTobject = new double[5]; //the distance to the object
      public double[] angleToObjects = new double[5]; //the angle to the object

      public double poseConfidence; //the confidence of the pose
      public double timeStamp; //the time stamp of the camera
      public double latency; //the latency of the camera
    }


    public PhotonCameraClass(String cameraName, CameraLocation location, int servoPort, boolean onlyRing, Transform3d[] cameraToRobot) {
      camera = new PhotonCamera(cameraName);
      inputs = new PhotonCameraInputsAutoLogged();

      inputs.cameraName = cameraName;
      inputs.location = location.toString();

      try {
        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile),
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraToRobot[0]);

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        inputs.isfieldLoaded = true;
      } catch (IOException exception) {
        inputs.isfieldLoaded = false;
      }

      if(servoPort != -1){
        servo = new Servo(servoPort);
        servo.set(0.55);
        inputs.hasServo = true;
      }
      else{
        servo = null;
        inputs.hasServo = false;
      }

      latestResult = new PhotonPipelineResult();

      inputs.estimatedPose = Constants.Vision.rubbishPose;

      mode = CameraMode.AprilTags;

      if(onlyRing) mode = CameraMode.Rings;

      inputs.timeStamp = 0;
      inputs.latency = 0;
      inputs.poseConfidence = 0;

      this.cameraToRobot = cameraToRobot;
    }

    public PhotonCameraClass(String cameraName, CameraLocation location, boolean onlyRing, Transform3d[] cameraToRobot){
      this(cameraName, location, -1, onlyRing, cameraToRobot);
    }

    @Override
    public Pose3d getPose() {
      if(mode == CameraMode.AprilTags && inputs.isfieldLoaded)
        return inputs.estimatedPose;
      else return Constants.Vision.rubbishPose;
    }

    // @Override
    // public Translation2d getTranslationToBestTarget(){
    //   if(mode == CameraMode.Rings)
    //     return inputs.objectsToRobot[0];
    //   else return Constants.Vision.rubbishTranslation[0];
    // }

    // @Override
    // public Translation2d[] getTranslationsToTargets(){
    //   if(mode == CameraMode.Rings)
    //     return inputs.objectsToRobot;
    //   else return Constants.Vision.rubbishTranslation;
    // }

    @Override
    public double getDistanceToBestTarget(){
      if(mode == CameraMode.Rings)
        return inputs.distanceTobject[0];
      else return 0;
    }

    @Override
    public double getAngleToBestTarget(){
      if(mode == CameraMode.Rings)
        return inputs.angleToObjects[0];
      else return 0;
    }

    @Override
    public double[] getDistanceToTargets(){
      if(mode ==  CameraMode.Rings)
        return inputs.distanceTobject;
      else return Constants.Vision.rubbishDistance;
    }

    @Override
    public double[] getAngleToTargets(){
      if(mode ==  CameraMode.Rings)
        return inputs.angleToObjects;
      else return Constants.Vision.rubbishAngle;
    }

    @Override
    public boolean hasTarget() {
      return latestResult.hasTargets();
    }

    @Override
    public void update() {
      latestResult = camera.getLatestResult();

      if(!latestResult.hasTargets()){
        inputs.timeStamp = 0;
        inputs.latency = 0;
        inputs.estimatedPose = Constants.Vision.rubbishPose;
        inputs.poseConfidence = 0;
        inputs.angleToObjects = Constants.Vision.rubbishAngle;
        inputs.distanceTobject = Constants.Vision.rubbishDistance;
        Logger.processInputs(inputs.cameraName, inputs);
        return;
      }

      inputs.timeStamp = latestResult.getTimestampSeconds();
      inputs.latency = latestResult.getLatencyMillis() * 100;

      if(mode == CameraMode.AprilTags && inputs.isfieldLoaded){
        if(latestResult.getBestTarget().getFiducialId() < 1 || latestResult.getBestTarget().getFiducialId() > 16) return;

        poseEstimator.setReferencePose(RobotContainer.driveBase.getPose());
        estimatedPose = poseEstimator.update();
        if(estimatedPose.isPresent()){
          inputs.estimatedPose = estimatedPose.get().estimatedPose;
          inputs.poseConfidence = latestResult.getBestTarget().getPoseAmbiguity();
        }
        else{
          inputs.estimatedPose = Constants.Vision.rubbishPose;
          inputs.poseConfidence = 0;
        }
      }
      else{

        // calculateDistanceAndAngleFromOffset(
        // PhotonUtils.calculateDistanceToTargetMeters(cameraToRobot[1].getZ(), Constants.Vision.gamePieceHeight, cameraToRobot[1].getRotation().getY(), Units.degreesToRadians(latestResult.getBestTarget().getPitch())),
        // Units.degreesToRadians(latestResult.getBestTarget().getYaw()), 0);
        
        inputs.angleToObjects[0] = latestResult.getBestTarget().getYaw();

        // inputs.distanceTobject[0] =  PhotonUtils.calculateDistanceToTargetMeters(
        // cameraToRobot[1].getZ(), Constants.Vision.gamePieceHeight, cameraToRobot[1].getRotation().getY(), target.getPitch());

        // inputs.angleToObjects[0] = target.getYaw(); //- Units.radiansToDegrees(cameraToRobot[1].getRotation().getZ());
        // for(int i = 0; i < targets.size() && i == 5; i++){
        //   inputs.objectsToRobot[i] = PhotonUtils.estimateCameraToTargetTranslation(PhotonUtils.calculateDistanceToTargetMeters(
        //   cameraToRobot[1].getZ(), Constants.Vision.gamePieceHeight / 2, cameraToRobot[1].getRotation().getY(), targets.get(i).getPitch()),
        //   Rotation2d.fromDegrees(targets.get(i).getYaw())).plus(cameraToRobot[1].getTranslation().toTranslation2d()
        //   ).rotateBy(cameraToRobot[1].getRotation().toRotation2d());
        // }
        // for(int  i = 0; i < targets.size() && i <= 5; i++){
        //   inputs.distanceTobject[i] = PhotonUtils.calculateDistanceToTargetMeters(
        //     cameraToRobot[1].getZ(), Constants.Vision.gamePieceHeight, cameraToRobot[1].getRotation().getY(), targets.get(i).getPitch());
        //   inputs.angleToObjects[i] = targets.get(i).getYaw() + Units.radiansToDegrees(cameraToRobot[1].getRotation().getZ());
        // }
      }

      Logger.processInputs(inputs.cameraName, inputs);
    }

    private void calculateDistanceAndAngleFromOffset(double distance, double angle, int index){
      // inputs.distanceTobject[index] = Math.sqrt(distance * (distance * Math.pow(Math.cos(angle), 2) +
      // distance * Math.pow(Math.sin(angle), 2) - 2 * cameraToRobot[1].getY() * Math.sin(angle)) - Math.pow(cameraToRobot[1].getY(), 2));
      inputs.distanceTobject[index] = Math.sqrt(distance * (distance - 2 * cameraToRobot[1].getY() * Math.sin(angle)) - Math.pow(cameraToRobot[1].getY(), 2));

      inputs.angleToObjects[index] = Math.acos((distance * Math.cos(angle) / inputs.distanceTobject[index]));
    }

    @Override
    public double getTimeStamp() {
      return inputs.timeStamp;
    }

    @Override
    public CameraLocation getCameraLocation(){
      return CameraLocation.valueOf(inputs.location);
    }

    @Override
    public double getLatency() {
      return inputs.latency;
    }

    @Override
    public double getServoAngle(){
      if(servo != null)
        return servo.getAngle();
      else return 0;
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
      System.out.println("switch to" + pipeLineIndex);
      camera.setPipelineIndex(pipeLineIndex);
      if(pipeLineIndex == 0){
        mode = CameraMode.AprilTags;
        inputs.angleToObjects = Constants.Vision.rubbishAngle;
        inputs.distanceTobject = Constants.Vision.rubbishDistance;
        if(servo != null)
          servo.set(0.55);
      }
      else{
        mode = CameraMode.Rings;
        inputs.estimatedPose = Constants.Vision.rubbishPose;
        if(servo != null)
          servo.set(0.74);
      }
    }

    @Override
    public double getPoseAmbiguty() {
      if(mode == CameraMode.AprilTags && inputs.isfieldLoaded)
        return inputs.poseConfidence;
      else return 0;
    }
  }