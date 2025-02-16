// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderAngleCommand extends Command 
{
  
  private final ShoulderSubsystem ShoulderSubsystem;
  private final double distance;
  private final double shoulderTolerance;
      
  public ShoulderAngleCommand(ShoulderSubsystem shoulder, Double distance, Double shoulderTolerance) 
  {
    this.ShoulderSubsystem = shoulder;
    this.distance = distance;
    this.shoulderTolerance = shoulderTolerance;    
    addRequirements(shoulder);
  }

  @Override
  public void initialize() 
  {
      ShoulderSubsystem.setShoulderBrake();      
  }

  @Override
  public void execute() 
  {            
   ShoulderSubsystem.setShoulderPosition(distance);    
  }  
    
  @Override
  public void end(boolean interrupted) 
  {
   ShoulderSubsystem.setShoulderCoast();;   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    double error = Math.abs(ShoulderSubsystem.getPosition() - distance);
    System.out.println("Checking isFinished(): Error = " + error);
    return error < shoulderTolerance;
  }
}
