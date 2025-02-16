// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GripperSubsystem;

public class GripperCommand  extends Command 
{
  private static final double GripperStopCurrent = 20.0; 
  private double GripperPercentOutput = 0.0; 
  private final GripperSubsystem GripperSubsystem;

  public GripperCommand(GripperSubsystem  GripperSubsystem,
                                            double GripperPercentOutput)
  {                                                 
   this.GripperSubsystem   = GripperSubsystem;
   this.GripperPercentOutput = GripperPercentOutput;   
    addRequirements(GripperSubsystem);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    
  }

  @Override
  public void execute() 
    {
      
      double currentDraw = GripperSubsystem.getStatorCurrent();;
      if (currentDraw > GripperStopCurrent)
        {
          GripperSubsystem.setGripperSpeed(0);
          cancel();          
        }
      else
      {
        GripperSubsystem.setGripperSpeed(GripperPercentOutput);
      }
    }

  @Override
  public void end(boolean interrupted) 
  {
         GripperSubsystem.setGripperSpeed(0.0);         
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

