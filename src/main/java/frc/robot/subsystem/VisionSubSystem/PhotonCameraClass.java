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
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
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
      public double[] angleToObjects = new double[5]; //the angle to the object

      public double poseConfidence; //the confidence of the pose
      public double timeStamp; //the time stamp of the camera
      public double latency; //the latency of the camera
    }


    public PhotonCameraClass(String cameraName, CameraLocation location, int servoPort, boolean onlyRing, Transform3d[] cameraToRobot) {
      camera = new PhotonCamera(cameraName);
      inputs = new PhotonCameraInputsAutoLogged();

      camera.setDriverMode(false);

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

    @Override
    public double getAngleToBestTarget(){
      if(mode == CameraMode.Rings)
        return inputs.angleToObjects[0];
      else return 0;
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
        inputs.angleToObjects[0] = -1000;
        Logger.processInputs(inputs.cameraName + " camera", inputs);
        return;
      }

      inputs.timeStamp = latestResult.getTimestampSeconds();
      inputs.latency = latestResult.getLatencyMillis() * 100;

      if(mode == CameraMode.AprilTags && inputs.isfieldLoaded){
        if(latestResult.getBestTarget().getFiducialId() < 1 || latestResult.getBestTarget().getFiducialId() > 16) return;

        poseEstimator.setReferencePose(RobotContainer.driveBase.getPose());
        estimatedPose = poseEstimator.update();
        if(estimatedPose.isPresent()){
          inputs.poseConfidence = latestResult.getBestTarget().getPoseAmbiguity();
          if(inputs.poseConfidence < Constants.Vision.confidanceThreshold) inputs.estimatedPose = estimatedPose.get().estimatedPose;
          else inputs.estimatedPose = Constants.Vision.rubbishPose;
        }
        else{
          inputs.estimatedPose = Constants.Vision.rubbishPose;
          inputs.poseConfidence = 0;
        }
      }
      else{   
        for(int i = 0; i < 5 && i < latestResult.getTargets().size(); i++){
          inputs.angleToObjects[i] = latestResult.getTargets().get(i).getYaw();
        }
      }

      Logger.processInputs(inputs.cameraName + " camera", inputs);
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