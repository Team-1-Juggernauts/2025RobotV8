// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.MathUtil;

import org.photonvision.PhotonCamera;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.Analog0Subsystem;

import frc.robot.commands.ClimbStickCommand;
import frc.robot.commands.ShoulderAngleCommand;
import frc.robot.commands.ShoulderJoyStickCommand;
import frc.robot.commands.WristFixedLocationCommand;
import frc.robot.commands.ElevatorFixedLocationCommand;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.AutoScoreCoralLowArmCommand;
import frc.robot.commands.AutoScoreCoralLowCommand;
import frc.robot.commands.CANdle_Blue_Command;
import frc.robot.commands.AutoLoadCoralCommand;
import frc.robot.commands.AimAtTagCommand;

import frc.robot.commands.CANdle_Yellow_Command;
import frc.robot.commands.CANdle_Solid_White_Animation;
import frc.robot.commands.CANdle_Green_Command;
import frc.robot.commands.CANdle_Red_Command;

public class RobotContainer 
{

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController UpperController = new CommandXboxController(1);
    private final PhotonCamera camera = new PhotonCamera("FrontCenter"); 

   public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
   private final VisionSubsystem vision = new VisionSubsystem("FrontCenter");
    
    double ElevatorPosHigh = 100.0;  
    double ElevatorPosMid  = 20.0; 
    double ElevatorPosHome =  1.0; 
    double elevatorTolerance = 2.0;  
    
    double ShoulderHigh = 100.00;
    double ShoulderMed  = 20.00;
    double ShoulderHome = 1.0;
    double shoulderTolerance = 2.0;


    double WristLoadPosition = 0.0;
    double WristScorePosition = 90;
    double WristTolerance = 2.0;
    double WristHome = 1.0;

    double   GripperPowerIn      = 0.3;   
    double   GripperPowerOut      = -0.3;    
    //private boolean isAnalogActivated = false; 
    double GripperAutoSecondsToRun = 2.0;
    double Gripper_ArmAutoSecondsToRun = 3.0;

    private final   CANdleSubsystem       m_Candle           = new CANdleSubsystem();
    private final   ClimbSubsystem        ClimbSubsystem     = new ClimbSubsystem();
    private final   ShoulderSubsystem     ShoulderSubsystem  = new ShoulderSubsystem();
    //private final   VisionSubsystem photonVisionSubsystem  = new PhotonVisionSubsystem("Team_1", new Transform3d());
    private final   Analog0Subsystem      Analog0Subsystem   = new Analog0Subsystem ();  
    public  final   ElevatorSubsystem     ElevatorSubsystem   = new ElevatorSubsystem(); 
    public  final   WristSubsystem       WristSubsystem      = new WristSubsystem();
    public final    GripperSubsystem      GripperSubsystem    = new GripperSubsystem(); 

    
    private static final Command ClimbStickCommand = null;    
   
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() 
    {
        
      CommandScheduler.getInstance().setDefaultCommand(ClimbSubsystem, ClimbStickCommand);
      
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        NamedCommands.registerCommand("AutoCoralLow", new   AutoScoreCoralLowCommand ( GripperSubsystem,
                                                                                              GripperPowerOut,
                                                                                              GripperAutoSecondsToRun));

        NamedCommands.registerCommand("AutoCoralLowArm", new AutoScoreCoralLowArmCommand  ( ShoulderSubsystem,
                                                                                                        ShoulderMed,
                                                                                                GripperSubsystem,
                                                                                                     GripperPowerOut,
                                                                                                Gripper_ArmAutoSecondsToRun,
                                                                                                shoulderTolerance));                                                                                      
      
        NamedCommands.registerCommand("AutoLoadCoral", new   AutoLoadCoralCommand ( GripperSubsystem,
                                                                                              GripperPowerIn,
                                                                                              GripperAutoSecondsToRun));
                                                                                       
    }
 
    private void configureBindings() 
    {

       drivetrain.setDefaultCommand        
        (
            
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * Math.abs(joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * Math.abs(joystick.getLeftX())* MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * Math.abs(joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
         )
        );

       // SmartDashboard.putNumber("Vision Target Yaw", photonVisionSubsystem.getAngleToBestTarget());
       //SmartDashboard.putNumber("Vision Target Distance", photonVisionSubsystem.getDistanceToBestTarget());
        //SmartDashboard.putBoolean("Has Vision Target", photonVisionSubsystem.hasTargets());

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0).withVelocityY(0.5))
    );

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        m_Candle.setDefaultCommand(new CANdle_Yellow_Command(m_Candle));

    //******************************************************************/
    //                      PhotonVision Aim at Reef Target
    //*****************************************************************/

    new Trigger(() -> joystick.getRightTriggerAxis() > 0.5)
                .whileTrue(AimAtTagCommand.create(drivetrain, vision));

       joystick.rightTrigger()
                          .whileTrue(new CANdle_Red_Command(m_Candle));
   
   /****************************************************************/
   //                         Elevator fixed position
   //***************************************************************/ 

   UpperController
   .povUp()
   .onTrue(new  ElevatorFixedLocationCommand(ElevatorSubsystem, ElevatorPosHigh, elevatorTolerance));

   UpperController
   .povLeft()
   .onTrue(new  ElevatorFixedLocationCommand(ElevatorSubsystem, ElevatorPosMid, elevatorTolerance));

   UpperController
    .povDown()
    .onTrue(new  ElevatorFixedLocationCommand(ElevatorSubsystem, ElevatorPosHome, elevatorTolerance));

 //*******************************************************************/
 //                      Shoulder Fixed position
 //*******************************************************************/
   
 UpperController
    .y()
    .onTrue(new ShoulderAngleCommand(ShoulderSubsystem, ShoulderHigh, shoulderTolerance)); 
    
 UpperController
    .x()
    .onTrue(new ShoulderAngleCommand(ShoulderSubsystem, ShoulderMed, shoulderTolerance)); 

 UpperController
    .b()
   .onTrue(new ShoulderAngleCommand(ShoulderSubsystem, ShoulderHome, shoulderTolerance));  

 //******************************************************************/
 //                                Shoulder Joystick
 //*****************************************************************/
    UpperController.leftStick()
    .onTrue(new ShoulderJoyStickCommand(ShoulderSubsystem,
                                     () -> MathUtil.applyDeadband(UpperController.getLeftY(), 0.01) ));

//*****************************************************************/
//                            Wrist to Score Reef position 
//******************************************************************/

    new Trigger(() -> UpperController.getLeftTriggerAxis() >0.5)
        .onTrue( new WristFixedLocationCommand( WristSubsystem, WristScorePosition, WristTolerance));
                 
//*****************************************************************/
//                         Wrist to Player Load position
//******************************************************************/

UpperController.leftBumper()
.onTrue(new WristFixedLocationCommand(WristSubsystem, WristLoadPosition, WristTolerance));

 
//*****************************************************************/
//                        GripperForward (Load)
//******************************************************************/

    new Trigger(() -> UpperController.getRightTriggerAxis() >0.5)
        .whileTrue( new GripperCommand( GripperSubsystem,GripperPowerOut));

    new  Trigger(() -> UpperController.getRightTriggerAxis() >0.5)
        .whileTrue( new CANdle_Blue_Command(m_Candle));
                 
//*****************************************************************/
//                         GripperReverse (Score)
//******************************************************************/

UpperController.rightBumper()
.whileTrue(new GripperCommand(GripperSubsystem, GripperPowerIn));

 UpperController.rightBumper()
                   .whileTrue(new CANdle_Solid_White_Animation(m_Candle));

 //******************************************************************/
    //                        Climber Joystick
    //*****************************************************************/
    UpperController.rightStick()
    .onTrue(new ClimbStickCommand(ClimbSubsystem,
                                     () -> MathUtil.applyDeadband(-UpperController.getRightY(), 0.01) ));

    new  Trigger(() -> UpperController.getLeftY() >0.5) 
                                .whileTrue(new CANdle_Green_Command(m_Candle));

     new  Trigger(() -> UpperController.getLeftY() >0.5)
                                .whileTrue( new CANdle_Blue_Command(m_Candle));

                                


                             

}
    public Command getAutonomousCommand() 
    {
       return autoChooser.getSelected();
    }
}
