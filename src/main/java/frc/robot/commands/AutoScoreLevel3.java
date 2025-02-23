// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
 
public class AutoScoreLevel3  extends Command 
{
  double AutoSecondsToRun;  
  double shoulderTolerance;
  double elevatorTolerance;
  double ExecuteTimeStamp = 0.0;
  double CurrentTimeStamp = 0.0;
  double GripperPercentOut;
  double targetPosition; 
  double eltargetPosition; 
  double _shoulderpos;
  double _elevatorpos;
  boolean ShoulderDone=false;
  boolean ElevatorDone=false;
  
  private final GripperSubsystem  GripperSubsystem;
  private final ShoulderSubsystem ShoulderSubsystem;
  private final ElevatorSubsystem ElevatorSubsystem;

  public AutoScoreLevel3( ShoulderSubsystem ShoulderSubsystem,
                                                  double targetPosition,
                                                  double shoulderTolerance,
                                      GripperSubsystem GripperSubsystem,
                                                  double GripperPowerOut,
                                                  double AutoSecondsToRun,
                                       ElevatorSubsystem ElevatorSubsystem,
                                                  double eltargetPosition,
                                                  double elevatorTolerance)

  {      
   
    this.ShoulderSubsystem = ShoulderSubsystem;
    this.targetPosition = targetPosition;
    this.shoulderTolerance = shoulderTolerance;
    this.GripperSubsystem = GripperSubsystem;
    this.GripperPercentOut = GripperPowerOut;
    this.AutoSecondsToRun = AutoSecondsToRun;
    this.ElevatorSubsystem = ElevatorSubsystem;
    this.eltargetPosition = eltargetPosition;
    this.elevatorTolerance = elevatorTolerance;
    
    
    addRequirements(GripperSubsystem);
    addRequirements(ShoulderSubsystem);
    addRequirements(ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    ExecuteTimeStamp = Timer.getFPGATimestamp();
    ShoulderSubsystem.setShoulderPosition(targetPosition);
    ElevatorSubsystem.setElevatorPosition(eltargetPosition);
  }

  @Override
  public void execute() 
  {
      
      ElevatorSubsystem.setElevatorPosition(eltargetPosition);
      _elevatorpos = ElevatorSubsystem.getPosition();

      if (_elevatorpos > eltargetPosition-elevatorTolerance && _elevatorpos < eltargetPosition+elevatorTolerance)
         {
          ElevatorDone=true;
      ShoulderSubsystem.setShoulderPosition(targetPosition);
      _shoulderpos = ShoulderSubsystem.getPosition();
         }
     if (_shoulderpos > targetPosition-shoulderTolerance && _shoulderpos < targetPosition+shoulderTolerance)
        {
         ShoulderDone=true;
        GripperSubsystem.setGripperSpeed(GripperPercentOut);
        }
  }

 
  @Override
  public void end(boolean interrupted) 
  {
    ShoulderSubsystem.setShoulderBrake();
    GripperSubsystem.setGripperSpeed(0); 
    ShoulderDone = false;
    ElevatorDone = false;
  }

  
  @Override
  public boolean isFinished()
  {
    if((Timer.getFPGATimestamp() -  ExecuteTimeStamp) > AutoSecondsToRun) 
    {
      ShoulderSubsystem.setShoulderBrake();
      ShoulderDone = false;
      ElevatorDone = false;
      return true;      
    } 
    return false;
  }
}
