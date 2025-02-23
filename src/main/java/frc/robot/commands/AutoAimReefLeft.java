// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAimReefLeft extends Command 
{

    double AutoSecondsToRun;  
    double ExecuteTimeStamp = 0.0;
    double CurrentTimeStamp = 0.0;
 
   private final VisionSubsystem VisionSubsystem;
   private final CommandSwerveDrivetrain CommandSwerveDrivetrain;

  public AutoAimReefLeft(VisionSubsystem VisionSubsystem,
                            CommandSwerveDrivetrain CommandSwerveDrivetrain)

  {      
    this.VisionSubsystem = VisionSubsystem;
    this.CommandSwerveDrivetrain = CommandSwerveDrivetrain;  
    addRequirements(VisionSubsystem);
    addRequirements(CommandSwerveDrivetrain);
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
      
            if (VisionSubsystem.hasValidTarget()) {
              double rotationSpeed = VisionSubsystem.getRotationSpeed();
              CommandSwerveDrivetrain.drive(0, 0, rotationSpeed);
          } else {
              CommandSwerveDrivetrain.stop();
          }
      }

      @Override
      public boolean isFinished() 
      
      {
          
        
        
        
        return VisionSubsystem.hasValidTarget() && Math.abs(VisionSubsystem.getRotationSpeed()) < 0.02;
      }

      @Override
      public void end(boolean interrupted) {
          CommandSwerveDrivetrain.stop();
      }
  }

