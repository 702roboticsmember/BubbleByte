package frc.robot;



import java.util.function.BooleanSupplier;


import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.Direction;
import frc.robot.Constants.ShooterSpeeds;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final XboxController driver = new XboxController(0);

    /**
     * Stows the arm mechanism
     */

    
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton fastMode = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton slowMode = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton align = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton follow = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    //private final JoystickButton AutoAmp = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    


    // private final POVButton increaseTopSpeed = new POVButton(master, Direction.UP.direction);
    // private final POVButton decreaseTopSpeed = new POVButton(master, Direction.DOWN.direction);

    // private final POVButton increaseBottomSpeed = new POVButton(master, Direction.RIGHT.direction);
    // private final POVButton decreaseBottomSpeed = new POVButton(master, Direction.LEFT.direction);

    public static double power = 1;
    public static boolean robotCentric = false;

    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> teamChooser;

    // private static final Orchestra orchestra = new Orchestra("mario.chrp");

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final LimelightSubsystem l_LimelightSubsystem = new LimelightSubsystem();

   

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */

     public Command AutoPickUp_Driver(double x, double z, double ry) {
        return new AlignCommand(() -> l_LimelightSubsystem.getTargetPos(0),
                        () -> l_LimelightSubsystem.getTargetPos(2),
                        () -> l_LimelightSubsystem.getTargetPos(4),
                        () -> l_LimelightSubsystem.IsTargetAvailable(), 
                        x, 
                        z, 
                        ry, 
                        s_Swerve); 
    }

    public Command AutoFollow_Driver(double turn) {
        return new AutoFollowCommand(()->l_LimelightSubsystem.getTargetX(), 
            ()->l_LimelightSubsystem.getTargetA(), 
            ()->l_LimelightSubsystem.IsTargetAvailable(), 
            s_Swerve, 
            turn);        
    }


    public RobotContainer() {
        Field2d field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });

        NamedCommands.registerCommand("align", AutoPickUp_Driver(0, .75, 0));

      
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, 
        ()-> -driver.getRawAxis(1), 
        ()-> -driver.getRawAxis(0),
        ()-> -driver.getRawAxis(4), 
        ()->robotCentric));

      

        configureButtonBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        teamChooser = new SendableChooser<>();
        

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Team Chooser", teamChooser);
    }

    /**
     * If the limit switch is pressed, we can assume that the ring is inside!
     * 
     * @return the value of the limit switch.
     */

    

    // private Command wrapSpeedChange(Runnable r) {
    //     return Commands.runOnce(() -> {
    //         r.run();
    //         RobotContainer.this.debugSpeeds();
    //     });
    // }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        /* Driver Buttons */
        zeroGyro.onTrue(new ParallelCommandGroup(new InstantCommand(() -> s_Swerve.zeroHeading()), new InstantCommand(()->s_Swerve.gyro.reset())));
        slowMode.onTrue(new InstantCommand(() -> RobotContainer.power = .333));
        fastMode.onTrue(new InstantCommand(() -> RobotContainer.power = 1));

        align.whileTrue(new SequentialCommandGroup(
                 new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0)), AutoPickUp_Driver(0, .75, 0)));

        align.onFalse(new ParallelCommandGroup(new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0))));
        follow.whileTrue(new SequentialCommandGroup(
            new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0)), AutoFollow_Driver(0.3)));

        follow.onFalse(new ParallelCommandGroup(new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0))));

 
    }
        

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup((new InstantCommand(() -> {
            
             s_Swerve.gyro.reset();
            // s_Swerve.zeroHeading();
        })), autoChooser.getSelected());
        // An ExampleCommand will run in autonomous
        // return new exampleAuto(s_Swerve);

    }
}
