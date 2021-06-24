/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.ghrobotics.lib.debug.FalconDashboard;
import org.techfire225.webapp.FireLog;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static final AHRS navx = new AHRS(SPI.Port.kMXP);
    public static boolean debug = true;
    public static double startAngle;
    private final Compressor compressor = new Compressor(0);
    public PowerDistributionPanel pdp = new PowerDistributionPanel();
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        navx.reset();
        m_robotContainer = new RobotContainer();
        m_robotContainer.hood.resetPosition();
        startAngle = navx.getYaw();
        compressor.start();

    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        for (int i = 0; i < 4; i++) {
            RobotContainer.swerveDrive.getModule(i).setAngle(0);
        }
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        FalconDashboard.INSTANCE.setFollowingPath(true);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        FalconDashboard.INSTANCE.setPathX(Units.metersToFeet(RobotContainer.swerveDrive.getPoseForTrajectory().getX()));
        FalconDashboard.INSTANCE.setPathY(Units.metersToFeet(RobotContainer.swerveDrive.getPoseForTrajectory().getY()));
        FalconDashboard.INSTANCE.setPathHeading(RobotContainer.swerveDrive.getPoseForTrajectory().getRotation().getRadians());
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        navx.reset();
        RobotContainer.swerveDrive.resetAllEncoders();
        m_robotContainer.hood.resetPosition();
        FalconDashboard.INSTANCE.setFollowingPath(false);
    }


    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        FireLog.log("hood_velocity", m_robotContainer.hood.getVelocity());
        FireLog.log("hood_position", m_robotContainer.hood.getPosition());
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
