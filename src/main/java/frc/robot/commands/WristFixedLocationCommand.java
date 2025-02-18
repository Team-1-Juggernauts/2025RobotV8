// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristFixedLocationCommand extends Command 
{
  
  private final WristSubsystem wristSubsystem;
  private final double  distance;
  private final double wristTolerance;
 
  public WristFixedLocationCommand(WristSubsystem wrist, Double distance, Double wristTolerance) 
  {
    this.wristSubsystem = wrist;
    this.distance = distance;        
    this.wristTolerance = wristTolerance;
    addRequirements(wrist);
  }

  @Override
  public void initialize() 
  {
    wristSubsystem.setWristBrake();         
  }

  @Override
  public void execute() 
  {          
    wristSubsystem.setWristPosition(distance);
    SmartDashboard.putNumber("Wrist Tolerance", wristTolerance);
  }

  @Override
  public void end(boolean interrupted) 
  {    
    wristSubsystem.setWristCoast();  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    
    double error = Math.abs(wristSubsystem.getPosition() - distance);
    System.out.println("Checking isFinished(): Error = " + error);
    return error < wristTolerance;

    }
  }
