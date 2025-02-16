package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import org.photonvision.PhotonCamera;

import frc.robot.subsystems.Analog0Subsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class AlignWithTargetCommand extends Command
{
 
    private final PhotonVisionSubsystem PhotonVisionSubsystem;

     public AlignWithTargetCommand(PhotonVisionSubsystem  PhotonVisionSubsystem)

    {
    this.PhotonVisionSubsystem = PhotonVisionSubsystem;
    addRequirements(PhotonVisionSubsystem);
  
    }
    
  public void initialize() 
  {
   
  }

  @Override
  public void execute() 
   {
    PhotonVisionSubsystem.getAngleToBestTarget();
   }

  
  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
