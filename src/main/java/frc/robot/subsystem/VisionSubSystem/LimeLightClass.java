// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.VisionSubSystem;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystem.VisionSubSystem.Vision.CameraInterface;
import frc.util.LimelightHelpers;
import frc.util.LimelightHelpers.LimelightResults;

/** Add your docs here. */
public class LimeLightClass implements CameraInterface{
    private LimelightResults latestResults;

    private LimelightInputsAutoLogged inputs;
    private NetworkTable table;

    @AutoLog
    public static class LimelightInputs{
      public String cameraName;
      public String location;

      public Pose3d estimatedPose;

      public double timeStamp;
      public double latency;
    }

    public LimeLightClass(String cameraName, CameraLocation location, int servoPort, Transform3d[] cameraToRobot) {
      inputs = new LimelightInputsAutoLogged();
      inputs.cameraName = cameraName;
      latestResults = new LimelightResults();

      this.inputs.location = location.name();
        
      LimelightHelpers.setCameraPose_RobotSpace(
        cameraName,
        cameraToRobot[0].getX(),
        cameraToRobot[0].getY(),
        cameraToRobot[0].getZ(),
        cameraToRobot[0].getRotation().getX(),
        cameraToRobot[0].getRotation().getY(),
        cameraToRobot[0].getRotation().getZ());

      inputs.estimatedPose = Constants.Vision.rubbishPose;

      inputs.timeStamp = 0;
      inputs.latency = 0;

      table = NetworkTableInstance.getDefault().getTable("limelight");

      Logger.processInputs(inputs.cameraName, inputs);
    }

    public LimeLightClass(String cameraName, CameraLocation location, Transform3d[] cameraToRobot) {
      this(cameraName, location, -1, cameraToRobot);
    }

    @Override
    public Pose3d getPose() {
      return inputs.estimatedPose;
    }

    @Override
    public double getAngleToBestTarget() {
      return -1000;
    }

    @Override
    public double getDistanceToBestTarget() {
      return -1000;
    }

    @Override
    public double[] getDistanceToTargets() {
      return Constants.Vision.rubbishDistance;
    }

    @Override
    public double[] getAngleToTargets() {
      return Constants.Vision.rubbishAngle;
    }

    @Override
    public double getServoAngle(){
      return 0;
    }

    @Override
    public boolean hasTarget() {
      return latestResults.targetingResults.valid;
    }

    @Override
    public void update() {
      latestResults = LimelightHelpers.getLatestResults(inputs.cameraName);

      if(!LimelightHelpers.getTV(inputs.cameraName)){
        inputs.timeStamp = 0;
        inputs.latency = 0;
        inputs.estimatedPose = Constants.Vision.rubbishPose;
        Logger.processInputs(inputs.cameraName, inputs);
        return;
      }

      // System.out.println("udpates");
      inputs.timeStamp = Timer.getFPGATimestamp() - (table.getEntry("tl").getDouble(0) - table.getEntry("cl").getDouble(0)) / 1000;
      inputs.latency = (table.getEntry("tl").getDouble(0) - table.getEntry("cl").getDouble(0)) / 1000;

      // inputs.timeStamp = latestResults.targetingResults.timestamp_RIOFPGA_capture;
      // inputs.latency = latestResults.targetingResults.latency_capture * 100;
      inputs.estimatedPose = toPose3d(table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]));

      // SmartDashboard.putNumber("timestamp", getTimeStamp());

      // inputs.estimatedPose = latestResults.targetingResults.getBotPose3d();

      Logger.processInputs(inputs.cameraName, inputs);
    }

    private Pose3d toPose3d(double[] arr){
      return new Pose3d(
        arr[0],
        arr[1],
        arr[2],
        new Rotation3d(
        arr[3],
        arr[4],
        arr[5]));
    }

    @Override
    public CameraLocation getCameraLocation(){
      return CameraLocation.valueOf(inputs.location);
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
    }

    @Override
    public double getTimeStamp() {
      return inputs.timeStamp;
    }

    @Override
    public double getLatency() {
      return inputs.latency;
    }

    @Override
    public double getPoseAmbiguty() {
      return 1;
    }
  }
