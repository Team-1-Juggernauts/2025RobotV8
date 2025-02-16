// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorFixedLocationCommand extends Command 
{
  
  private final ElevatorSubsystem elevatorSubsystem;
  private final double  distance;
  private final double elevatorTolerance;
 
  public ElevatorFixedLocationCommand(ElevatorSubsystem elevator, Double distance, Double elevatorTolerance) 
  {
    this.elevatorSubsystem = elevator;
    this.distance = distance;        
    this.elevatorTolerance = elevatorTolerance;
    addRequirements(elevator);
  }

  @Override
  public void initialize() 
  {
    elevatorSubsystem.setElevatorBrake();         
  }

  @Override
  public void execute() 
  {          
    elevatorSubsystem.setElevatorPosition(distance);
  }

  @Override
  public void end(boolean interrupted) 
  {    
    elevatorSubsystem.setElevatorCoast();  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    
    double error = Math.abs(elevatorSubsystem.getPosition() - distance);
    System.out.println("Checking isFinished(): Error = " + error);
    return error < elevatorTolerance;

    }
  }
