package frc.robot;




import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AlgaeArmPID;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.AutoFollowCommand;
import frc.robot.commands.ClimbPID;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.FollowPath;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;
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
    private final XboxController codriver = new XboxController(1);

    private DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_INTAKE);

    

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton leftstation = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton rightstation = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton fastMode = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton slowMode = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton align = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton alignLeft = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton alignRight = new JoystickButton(driver, XboxController.Button.kRightBumper.value);


    

    /* CoDriver Buttons */
    private final JoystickButton algaeReefIntake = new JoystickButton(codriver, XboxController.Button.kA.value);
    private final JoystickButton climbPID = new JoystickButton(codriver, XboxController.Button.kB.value);
    private final JoystickButton coralOuttake = new JoystickButton(codriver, XboxController.Button.kX.value);
    private final JoystickButton coralIntake = new JoystickButton(codriver, XboxController.Button.kStart.value);
    private final JoystickButton nest = new JoystickButton(codriver, XboxController.Button.kY.value);
    private final JoystickButton algaeGroundIntake = new JoystickButton(codriver, XboxController.Button.kRightBumper.value);
    private final JoystickButton algaeOuttake = new JoystickButton(codriver, XboxController.Button.kLeftBumper.value);
    

    private final POVButton L1 = new POVButton(codriver, 90);
    private final POVButton L2 = new POVButton(codriver, 0);
    private final POVButton L3 = new POVButton(codriver, 270);
    private final POVButton L4 = new POVButton(codriver, 180);

    public static double power = 1;
    public static boolean robotCentric = false;
    public static boolean climbing = false;

    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> teamChooser;
  

    //private static final Orchestra orchestra = new Orchestra("mario.chrp");

    /* Subsystems */
    private final LimelightSubsystem l_LimelightSubsystem = new LimelightSubsystem();
    private final Swerve s_Swerve = new Swerve(l_LimelightSubsystem);
    private final ClimbSubsystem c_ClimbSubsystem = new ClimbSubsystem();
    private final AlgaeArmSubsystem a_AlgaeArmSubsystem = new AlgaeArmSubsystem();
    private final AlgaeIntakeSubsystem a_AlgaeIntakeSubsystem = new AlgaeIntakeSubsystem();
    private final CoralIntakeSubsystem c_CoralIntakeSubsystem = new CoralIntakeSubsystem();
    private final ElevatorSubsystem e_ElevatorSubsytem = new ElevatorSubsystem(); 
    public static Field2d field = new Field2d();

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */

    /* Commands */
    

     public Command Align_Driver(double x, double z, double ry) {
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

    public Command Nest() {
        return new ParallelCommandGroup(
            new AlgaeArmPID(a_AlgaeArmSubsystem, Constants.AlgaeArmConstants.DefaultPose),
            new ElevatorPID(e_ElevatorSubsytem, Constants.ElevatorConstants.DefaultPose),
            new ClimbPID(c_ClimbSubsystem, Constants.ClimberConstants.DefaultPose)
        );
    }
    public Command AlgaeReefIntake_coDriver(){
        return new ParallelCommandGroup(
            new AlgaeArmPID(a_AlgaeArmSubsystem, Constants.AlgaeArmConstants.ReefPose),
            a_AlgaeIntakeSubsystem.run(Constants.AlgaeIntakeConstants.IntakeSpeed)
        );
        
    }
    public Command AlgaeGroundIntake_coDriver(){
        return new ParallelCommandGroup(
            new AlgaeArmPID(a_AlgaeArmSubsystem, Constants.AlgaeArmConstants.GroundPose),
            a_AlgaeIntakeSubsystem.run(Constants.AlgaeIntakeConstants.IntakeSpeed)
        );
        
    }
    public Command AlgaeOuttake_coDriver(){
        return new SequentialCommandGroup(
            new AlgaeArmPID(a_AlgaeArmSubsystem, Constants.AlgaeArmConstants.OuttakePose),
            a_AlgaeIntakeSubsystem.run(Constants.AlgaeIntakeConstants.OuttakeSpeed)
        );
        
    }

    public Command AlgaeReefIntake(){
        return new ParallelCommandGroup(
            new AlgaeArmPID(a_AlgaeArmSubsystem, Constants.AlgaeArmConstants.ReefPose),
            new InstantCommand(()->a_AlgaeIntakeSubsystem.setSpeed(Constants.AlgaeIntakeConstants.IntakeSpeed))
        );
        
    }
    public Command AlgaeGroundIntake(){
        return new ParallelCommandGroup(
            new AlgaeArmPID(a_AlgaeArmSubsystem, Constants.AlgaeArmConstants.GroundPose),
            new InstantCommand(()->a_AlgaeIntakeSubsystem.setSpeed(Constants.AlgaeIntakeConstants.IntakeSpeed))
        );
        
    }
    public Command AlgaeStow(){
        return new SequentialCommandGroup(
            new InstantCommand(()->a_AlgaeArmSubsystem.setSpeed(0)),
            new AlgaeArmPID(a_AlgaeArmSubsystem, Constants.AlgaeArmConstants.DefaultPose)
        );
    }
    public Command AlgaeOuttake(){
        return new SequentialCommandGroup(
            new AlgaeArmPID(a_AlgaeArmSubsystem, Constants.AlgaeArmConstants.OuttakePose),
            new InstantCommand(()->a_AlgaeIntakeSubsystem.setSpeed(Constants.AlgaeIntakeConstants.OuttakeSpeed))
        );
        
    }
    public Command AlgaeOuttakeOff(){
        return new InstantCommand(()->a_AlgaeIntakeSubsystem.setSpeed(0));
    }
    public Command AlignRight_Driver(){
        return new AlignCommand(() -> l_LimelightSubsystem.getTargetPos(0),
                        () -> l_LimelightSubsystem.getTargetPos(2),
                        () -> l_LimelightSubsystem.getTargetPos(4),
                        () -> l_LimelightSubsystem.IsTargetAvailable(), 
                        Constants.AlignConstants.rightX,
                        Constants.AlignConstants.rightZ, 
                        Constants.AlignConstants.rightRY, 
                        s_Swerve); 
                    }
    public Command AlignLeft_Driver(){
        return new AlignCommand(() -> l_LimelightSubsystem.getTargetPos(0),
                        () -> l_LimelightSubsystem.getTargetPos(2),
                        () -> l_LimelightSubsystem.getTargetPos(4),
                        () -> l_LimelightSubsystem.IsTargetAvailable(), 
                        Constants.AlignConstants.leftX,
                        Constants.AlignConstants.leftZ, 
                        Constants.AlignConstants.leftRY, 
                        s_Swerve); 
    }


    public Command ClimbOutPID(){
        return new ClimbPID(c_ClimbSubsystem, Constants.ClimberConstants.OutPose);
    }

    public Command ClimbInPID(){
        return new ClimbPID(c_ClimbSubsystem, Constants.ClimberConstants.InPose);
    }
    public Command CoralOuttake_coDriver(){
        return c_CoralIntakeSubsystem.run(Constants.CoralIntakeConstants.OuttakeSpeed);
    }

    public Command CoralOuttake_coDriver(double speed){
        return c_CoralIntakeSubsystem.run(speed);
    }

    public Command CoralOuttake(){
        return new InstantCommand(()-> c_CoralIntakeSubsystem.setSpeed(Constants.CoralIntakeConstants.OuttakeSpeed));
    }

    public Command CoralIntake(){
        return Commands.either(CoralOuttake(), Coraloff(), ()-> !limitSwitch.get());
    }

    public Command CoralIntake_coDriver(double speed){
        return Commands.either(CoralOuttake_coDriver(speed), Coraloff(), ()-> !limitSwitch.get());
    }

    public Command Coraloff(){
        return new InstantCommand(()-> c_CoralIntakeSubsystem.setSpeed(0));
    }

    public Command LimitSwitchDeadline(){
        return Commands.waitUntil(()-> !limitSwitch.get());
    }

    public Command L1(){
        return new ElevatorPID(e_ElevatorSubsytem, Constants.ElevatorConstants.L1Pose);
    }

    public Command L2(){
        return new ElevatorPID(e_ElevatorSubsytem, Constants.ElevatorConstants.L2Pose);
    }

    public Command L3(){
        return new ElevatorPID(e_ElevatorSubsytem, Constants.ElevatorConstants.L3Pose);
    }

    public Command L4(){
        return new ElevatorPID(e_ElevatorSubsytem, Constants.ElevatorConstants.L4Pose);
    }

// Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.


// Since AutoBuilder is configured, we can use it to build pathfinding commands


    public RobotContainer() {
        //Pathfinding.setDynamicObstacles(null, null);
        //beamLED.set(true);
        
        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("robotposex", s_Swerve.getPose().getTranslation().getX());
        SmartDashboard.putNumber("robotposey", s_Swerve.getPose().getTranslation().getY());
        
        //Logging callback for current robot pose
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

        NamedCommands.registerCommand("align", Align_Driver(0, .75, 0));
        NamedCommands.registerCommand("Disconect", new FollowPath(s_Swerve, "P3Disconect"));
      
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, 
        ()-> -driver.getRawAxis(1) * power, 
        ()-> -driver.getRawAxis(0)* power,
        ()-> -driver.getRawAxis(4)* power, 
        ()->robotCentric));

        
        c_ClimbSubsystem.setDefaultCommand(c_ClimbSubsystem.run(codriver.getRawAxis(0)));
        c_CoralIntakeSubsystem.setDefaultCommand(CoralIntake_coDriver(codriver.getRawAxis(2)));
        e_ElevatorSubsytem.setDefaultCommand(e_ElevatorSubsytem.run(codriver.getRawAxis(5)));
        

        configureButtonBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        teamChooser = new SendableChooser<>();
        

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Team Chooser", teamChooser);
        //SmartDashboard.putNumber("gyroYaaaww", s_Swerve.getGyroYaw());
    }

    

    

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
        zeroGyro.onTrue(new SequentialCommandGroup(new InstantCommand(()->s_Swerve.gyro.reset()), new InstantCommand(() -> s_Swerve.zeroHeading())));
        slowMode.onTrue(new InstantCommand(() -> RobotContainer.power = .2));
        fastMode.onTrue(new InstantCommand(() -> RobotContainer.power = 1));

        align.whileTrue(new SequentialCommandGroup(
                 new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0)), Align_Driver(0, .75, 0)));

        align.onFalse(new ParallelCommandGroup(new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0))));
        // follow.whileTrue(new SequentialCommandGroup(
        //     new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0)), AutoFollow_Driver(0.3)));

        // follow.onFalse(new ParallelCommandGroup(new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0))));
        alignLeft.whileTrue(AlignLeft_Driver());
        alignRight.whileTrue(AlignRight_Driver());
        //resetpose.onTrue(new InstantCommand(()->field.setRobotPose(0, 0, s_Swerve.getHeading())));
        leftstation.whileTrue(new SequentialCommandGroup(new FollowPath(s_Swerve, "PathTest")));
        rightstation.whileTrue(new SequentialCommandGroup(new FollowPath(s_Swerve, "PathTest")));
        

        /*CoDriver Buttons*/
        
        nest.whileTrue(Nest());
        algaeReefIntake.whileTrue(AlgaeReefIntake_coDriver());
        algaeReefIntake.onFalse(AlgaeStow());
        algaeGroundIntake.whileTrue(AlgaeGroundIntake_coDriver());
        algaeGroundIntake.onFalse(AlgaeStow());
        algaeOuttake.whileTrue(AlgaeOuttake_coDriver());
        algaeOuttake.onFalse(AlgaeStow());
        climbPID.toggleOnTrue(ClimbOutPID());
        climbPID.toggleOnFalse(ClimbInPID());
        coralOuttake.whileTrue(CoralOuttake_coDriver());
        coralIntake.whileTrue(CoralIntake_coDriver(Constants.CoralIntakeConstants.IntakeSpeed));
        
        L1.onTrue(L1());
        L2.onTrue(L2());
        L3.onTrue(L3());
        L4.onTrue(L4());

    }
        

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(new InstantCommand(()-> s_Swerve.gyro.reset()), autoChooser.getSelected());
        
    }
}
