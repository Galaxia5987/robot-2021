package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commandgroups.FeedAndShoot;
import frc.robot.commandgroups.FeedAndShootWithoutVision;
import frc.robot.commandgroups.PickupBalls;
import frc.robot.subsystems.PTO.PTO;
import frc.robot.subsystems.PTO.commands.SwitchSubsystems;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.commands.PercentClimb;
import frc.robot.subsystems.climber.commands.SetStopper;
import frc.robot.subsystems.climber.commands.StartClimb;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedShooter;
import frc.robot.subsystems.conveyor.commands.MinimizeConveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.*;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.commands.StartFunnel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.subsystems.shooter.commands.ShootWithoutVision;
import frc.robot.utils.VisionModule;
import frc.robot.utils.vision_commands.ChangeLEDs;
import frc.robot.utils.vision_commands.ToggleLEDs;
import frc.robot.valuetuner.ValueTuner;
import org.techfire225.webapp.Webserver;
import frc.robot.valuetuner.WebConstant;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class RobotContainer {
    public static final PTO pto = new PTO();
    public static final WebConstant velocity = new WebConstant("velocity", 69);
    public static final VisionModule vision = new VisionModule();
    public Funnel funnel = new Funnel();
    public Shooter shooter = new Shooter();
    public Climber climber = new Climber();
    //    public VisionModule visionModule = new VisionModule();
    public static XboxController Xbox = new XboxController(2);
    public static XboxController XboxDriver = new XboxController(3);
    public JoystickButton a = new JoystickButton(Xbox, XboxController.Button.kA.value);
    public JoystickButton b = new JoystickButton(Xbox, XboxController.Button.kB.value);
    public JoystickButton x = new JoystickButton(Xbox, XboxController.Button.kX.value);
    public JoystickButton xDriver = new JoystickButton(XboxDriver, XboxController.Button.kX.value);
    // The robot's subsystems and commands are defined here...
    public Intake intake = new Intake();
    public JoystickButton BL = new JoystickButton(Xbox, XboxController.Button.kBumperLeft.value);
    public JoystickButton BR = new JoystickButton(Xbox, XboxController.Button.kBumperRight.value);
    public JoystickButton y = new JoystickButton(Xbox, XboxController.Button.kY.value);
    public Conveyor conveyor = new Conveyor();
    public JoystickButton LT = new JoystickButton(Xbox, XboxController.Button.kStickLeft.value);
    public JoystickButton start = new JoystickButton(Xbox, XboxController.Button.kStart.value);
    public JoystickButton stop = new JoystickButton(Xbox, XboxController.Button.kBack.value);

    public SwerveDrive swerveDrive = new SwerveDrive(true, false);

    public WebConstant visionOutput = new WebConstant("vision-output", 0);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        if (Robot.debug) {
            startValueTuner();
            startFireLog();
        }
//        swerveDrive.setDefaultCommand(new HolonomicDrive(swerveDrive));
//        swerveDrive.setDefaultCommand(new Rotate(swerveDrive));
//        swerveDrive.setDefaultCommand(new DriveForward(swerveDrive));
    }


    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

//        a.whileHeld(new PickupBalls(intake, funnel, conveyor, true));
        a.whenPressed(new ChangeLEDs(true));
        b.whenPressed(new ChangeLEDs(false));
//        a.whileHeld(new FeedShooter(conveyor, Constants.Conveyor.CONVEYOR_MOTOR_POWER));
//        b.whileHeld(new PickupBalls(intake, funnel, conveyor, false));
//        x.whileHeld(new FeedAndShoot(pto, conveyor, shooter, Constants.Conveyor.CONVEYOR_MOTOR_POWER));
//        y.whileHeld(new PickupBalls(intake, funnel, conveyor, true));
//        xDriver.whileHeld(new InstantCommand(() -> swerveDrive.lock()));
//        b.whileHeld(new FeedShooter(conveyor, Constants.Conveyor.CONVEYOR_MOTOR_POWER));
//        BR.whileHeld(new FeedShooter(conveyor, -Constants.Conveyor.CONVEYOR_MOTOR_POWER));
//        BL.whileHeld(new FeedAndShootWithoutVision(conveyor, shooter, Constants.Conveyor.CONVEYOR_MOTOR_POWER));
//        a.whenPressed(new InstantCommand(() -> intake.togglePiston()));
//        LT.whileHeld(new StartFunnel(funnel, false));
//        start.whenPressed(new StartClimb(pto, climber));
//        stop.whileHeld(new PercentClimb(climber, () -> Xbox.getY(GenericHID.Hand.kLeft)));
    }




    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }

    /**
     * Initiates the value tuner.
     */
    private void startValueTuner() {
        new ValueTuner().start();
    }

    /**
     * Initiates the port of team 225s Fire-Logger.
     */
    private void startFireLog() {
        try {
            new Webserver();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
