// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorFixedLocationCommand extends Command 
{
  
  private final ElevatorSubsystem elevatorSubsystem;
  private final double  targetPosition;
  private final double elevatorTolerance;
 
  public ElevatorFixedLocationCommand(ElevatorSubsystem elevator, Double targetPosition, Double elevatorTolerance) 
  {
    this.elevatorSubsystem = elevator;
    this.targetPosition =targetPosition;        
    this.elevatorTolerance = elevatorTolerance;
    addRequirements(elevator);
  }

  @Override
  public void initialize() 
  {
    elevatorSubsystem.setElevatorBrake();   
    elevatorSubsystem.setElevatorPosition(targetPosition);      
  }

  @Override
  public void execute() 
  {          
    elevatorSubsystem.setElevatorPosition(targetPosition);
    SmartDashboard.putNumber("Elevator Tolerance", elevatorTolerance);
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
    
    double error = Math.abs(elevatorSubsystem.getPosition() - targetPosition);
    System.out.println("Checking isFinished(): Error = " + error);
    return Math.abs(elevatorSubsystem.getPosition() - targetPosition) < elevatorTolerance;

    }
  }
