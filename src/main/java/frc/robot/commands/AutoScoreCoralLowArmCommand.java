// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
 
public class AutoScoreCoralLowArmCommand  extends Command 
{
  double AutoSecondsToRun;  
  double shoulderTolerance;
  double ExecuteTimeStamp = 0.0;
  double CurrentTimeStamp = 0.0;
  double GripperPercentOut;
  double shoulderPosOut;  
  double _shoulderpos;
  boolean ShoulderDone=false;
  
  private final GripperSubsystem  GripperSubsystem;
  private final ShoulderSubsystem ShoulderSubsystem;

  public AutoScoreCoralLowArmCommand( ShoulderSubsystem ShoulderSubsystem,
                                                          double ShoulderPosOut,
                                      GripperSubsystem GripperSubsystem,
                                                          double GripperPowerOut,
                                                          double AutoSecondsToRun,
                                                          double shoulderTolerance)

  {      
   
    this.ShoulderSubsystem = ShoulderSubsystem;
    this.shoulderPosOut = shoulderPosOut;
    this.GripperSubsystem = GripperSubsystem;
    this.GripperPercentOut = GripperPowerOut;
    this.AutoSecondsToRun = AutoSecondsToRun;
    this.shoulderTolerance = shoulderTolerance;
    
    
    addRequirements(GripperSubsystem);
    addRequirements(ShoulderSubsystem);
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
      
      ShoulderSubsystem.setShoulderPosition(shoulderPosOut);
      _shoulderpos = ShoulderSubsystem.getPosition();

 if (_shoulderpos > shoulderPosOut-shoulderTolerance && _shoulderpos < shoulderPosOut+shoulderTolerance)
      {
         ShoulderDone=true;
        GripperSubsystem.setGripperSpeed(GripperPercentOut);
      }
  }

 
  @Override
  public void end(boolean interrupted) 
  {
    ShoulderSubsystem.setShoulderCoast();
    GripperSubsystem.setGripperSpeed(0); 
    ShoulderDone = false;
  }

  
  @Override
  public boolean isFinished()
  {
    if((Timer.getFPGATimestamp() -  ExecuteTimeStamp) > AutoSecondsToRun) 
    {
      ShoulderSubsystem.setShoulderCoast();
      ShoulderDone = false;
      return true;      
    } 
    return false;
  }
}
