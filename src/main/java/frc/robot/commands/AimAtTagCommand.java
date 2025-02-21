// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class AimAtTagCommand {
    public static Command create(CommandSwerveDrivetrain swerve, VisionSubsystem vision) 
    {
        return new Command() {
            @Override
            public void execute() {
                if (vision.hasValidTarget()) {
                    double rotationSpeed = vision.getRotationSpeed();
                    swerve.drive(0, 0, rotationSpeed);
                } else {
                    swerve.stop();
                }
            }

            @Override
            public boolean isFinished() {
                return vision.hasValidTarget() && Math.abs(vision.getRotationSpeed()) < 0.02;
            }

            @Override
            public void end(boolean interrupted) {
                swerve.stop();
            }
        };
    }
}