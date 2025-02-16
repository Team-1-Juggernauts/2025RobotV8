// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonVisionSubsystem extends SubsystemBase 
{
  

  /** Creates a new PhotonVisionSubsystem. */
  protected final PhotonCamera camera;

  private final Transform3d cameraToRobot;
  private final Transform3d robotToCamera;
  private PhotonPipelineResult result = new PhotonPipelineResult();
  private double angleToTarget;
  private double distanceToTarget;
  private double poseAmibiguity;
  private double targetID;
  private double area;
  private double skew;
  private double pitch;




  private BooleanLogEntry hasTargetLogger;
  private DoubleLogEntry distanceLogger;
  private DoubleLogEntry angleLogger;
  

  public PhotonVisionSubsystem(String frontcenter, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(frontcenter);
    this.cameraToRobot = robotToCamera.inverse();
    this.robotToCamera = robotToCamera;

    hasTargetLogger = new BooleanLogEntry(DataLogManager.getLog(), String.format("/%s/Has Target", frontcenter));
    distanceLogger = new DoubleLogEntry(DataLogManager.getLog(), String.format("/%s/Distance", frontcenter));
    angleLogger = new DoubleLogEntry(DataLogManager.getLog(), String.format("/%s/Angle", frontcenter));

  }

  @Override
  public void periodic() 
  {


    // This method will be called once per scheduler run
    PhotonPipelineResult currentResult = camera.getLatestResult();
    if (this.result.hasTargets() != currentResult.hasTargets()) 
    {
      hasTargetLogger.append(currentResult.hasTargets());

        var result = currentResult;
      
    }
 
    this.result = currentResult;

    if (hasTargets()) {
      PhotonTrackedTarget bestTarget = getBestTarget();
    if (bestTarget != null) {
        targetID = bestTarget.getFiducialId();
      targetID = bestTarget.getFiducialId();

      angleToTarget =- bestTarget.getYaw();
      pitch = bestTarget.getPitch();
      Transform3d bestTargetTransform = bestTarget.getBestCameraToTarget();
      distanceToTarget =bestTargetTransform.getZ();
      poseAmibiguity = bestTarget.getPoseAmbiguity();
      area = bestTarget.getArea();
      skew = bestTarget.getSkew();


      distanceLogger.append(distanceToTarget);
      angleLogger.append(angleToTarget);
    }
    }
    SmartDashboard.putNumber(String.format("Photon Vision %s Target Distance", camera.getName()), distanceToTarget);
    SmartDashboard.putNumber(String.format("Photon Vision %s Best Target ID", camera.getName()), targetID);
    SmartDashboard.putNumber(String.format("Photon Vision %s Target Distance", camera.getName()), distanceToTarget);
    SmartDashboard.putNumber(String.format("Photon Vision %s Target Yaw", camera.getName()), angleToTarget);
    SmartDashboard.putNumber(String.format("Photon Vision %s Pose Ambiguity", camera.getName()), poseAmibiguity);
    SmartDashboard.putNumber(String.format("Photon Vision %s Target Area", camera.getName()), area);
    SmartDashboard.putNumber(String.format("Photon Vision %s Target Skew", camera.getName()), skew);
    SmartDashboard.putNumber(String.format("Photon Vision %s Target Pitch", camera.getName()), pitch);
    
    
  }

  /**
   * Returns the latest vision result
   * 
   * @return latest vision result
   */
  protected PhotonPipelineResult getLatestResult(){
    return result;
  }

  /**
   * Return angle to specificed target
   * @param target Target
   * @return Angle to target
   */
  public static double calculateAngletToTarget(PhotonTrackedTarget target){
    return Math.toRadians(-target.getYaw());
  }

  /**
   * Return the transform from the camera to the center of the robot 
   * 
   * @return Return the transform from the camera to the center of the robot
   */

  public Transform3d getCameraToRobotTransform(){
    return cameraToRobot;
  }

  /***
   * Return the transform from the center of the robot to camera
   * 
   * @return Return the transform from center of the robot to camera
   */
  public Transform3d getRobotToCameraTransform(){
    return robotToCamera;
  }

  /***
   * Returns weather the result has targets
   * 
   * @return Boolean if result has targets
   */
  public boolean hasTargets(){
    return result.hasTargets();
  }

  /***
   * Return best target
   * 
   * @return best target
   */
  public PhotonTrackedTarget getBestTarget() 
  {
   return result.getBestTarget();
  }
   
  public double getDistanceToBestTarget() {
    return distanceToTarget;

  }

  public double getAngleToBestTarget() {
    return angleToTarget;
  }
  
  public double getAmibiguity() {
    return poseAmibiguity;
  }

  public double getTargetTimeStamp() {
    return result.getTimestampSeconds();
  }

  public List<PhotonTrackedTarget> getTargets() {
    return result.getTargets();
  }

}