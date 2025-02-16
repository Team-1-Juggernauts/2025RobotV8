// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoLoadCoralCommand  extends Command 
{
 
 private static final double GripperStopCurrent = 20.0;
  private double GripperAutoSecondsToRun;  
  double ExecuteTimeStamp = 0.0;
  double GripperPercentIn = 0.0;  
  private final GripperSubsystem GripperSubsystem;

  public AutoLoadCoralCommand( GripperSubsystem GripperSubsystem,
                                                          double GripperPercentIn,
                                                          double GripperAutoSecondstoRun)

  {      
    this.GripperSubsystem = GripperSubsystem;
    this.GripperPercentIn = GripperPercentIn;
    this.GripperAutoSecondsToRun = GripperAutoSecondstoRun;       
    addRequirements(GripperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    ExecuteTimeStamp = Timer.getFPGATimestamp();
    
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
        GripperSubsystem.setGripperSpeed(GripperPercentIn);
      }

  }

    @Override
  public void end(boolean interrupted) 
  {
       GripperSubsystem.setGripperSpeed(0);    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    if((Timer.getFPGATimestamp() -  ExecuteTimeStamp) > GripperAutoSecondsToRun) 
    {
      return true;
    } 
    return false;
  }
}
