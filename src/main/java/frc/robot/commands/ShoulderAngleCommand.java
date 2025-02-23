// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderAngleCommand extends Command 
{
  
  private final ShoulderSubsystem ShoulderSubsystem;
  private final double targetPosition;
  private final double shoulderTolerance;
      
  public ShoulderAngleCommand(ShoulderSubsystem shoulder, Double targetPosition, Double shoulderTolerance) 
  {
    this.ShoulderSubsystem = shoulder;
    this.targetPosition = targetPosition;
    this.shoulderTolerance = shoulderTolerance;    
    addRequirements(shoulder);
  }

  @Override
  public void initialize() 
  {
      ShoulderSubsystem.setShoulderBrake();   
      ShoulderSubsystem.setShoulderPosition(targetPosition);   
  }

  @Override
  public void execute() 
  {            
   ShoulderSubsystem.setShoulderPosition(targetPosition);    
  }  
    
  @Override
  public void end(boolean interrupted) 
  {
   ShoulderSubsystem.setShoulderBrake();;   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    double error = Math.abs(ShoulderSubsystem.getPosition() - targetPosition);
    System.out.println("Checking isFinished(): Error = " + error);
    return Math.abs(ShoulderSubsystem.getPosition() - targetPosition) < shoulderTolerance;
  }
}
