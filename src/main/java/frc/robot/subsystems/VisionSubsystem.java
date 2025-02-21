// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase 
{
    private final PhotonCamera camera;
    private PhotonTrackedTarget bestTarget;
    private final PIDController turnPID;

    public VisionSubsystem(String FrontCenter) {
        camera = new PhotonCamera(FrontCenter);
        turnPID = new PIDController(VisionConstants.kP, VisionConstants.kI, VisionConstants.kD);
        turnPID.setTolerance(VisionConstants.angleTolerance);
    }

    public boolean hasValidTarget() {
        return bestTarget != null;
    }

    public double getRotationSpeed() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            bestTarget = result.getBestTarget();
            double yawError = bestTarget.getYaw();
            return turnPID.calculate(yawError, 0.0);
        }
        return 0.0;
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();
        bestTarget = result.hasTargets() ? result.getBestTarget() : null;
    }
}

