package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.Vector;
import edu.wpi.first.wpiutil.math.numbers.N1;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.UnitModel;
import frc.robot.subsystems.PTO.PTO;
import org.techfire225.webapp.FireLog;

import static frc.robot.Constants.Shooter.*;

/**
 * The shooter class represents the physical shooter.
 * The purpose of the flywheel is to provide a set of functions to handle a shooting situation in a game.
 *
 * @author Barel
 * @version 1.0
 * @using TalonFX
 * @since 2021
 */
public class Shooter extends SubsystemBase {
    private final PTO pto = new PTO();

    public Shooter() {}

    public void setVelocity(double velocity) {
        pto.getMaster().set(ControlMode.Velocity, velocity);
    }

    public void stop() {
        pto.getMaster().set(ControlMode.PercentOutput, 0);
    }

    public double getVelocity() {
        return pto.getMaster().getSelectedSensorVelocity();
    }



}


