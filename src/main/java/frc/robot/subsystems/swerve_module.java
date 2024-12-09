package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config;
import frc.robot.constants;

public class swerve_module {
    private final TalonFX drive_motor;
    private final TalonFX turn_motor;
    private final DutyCycleEncoder abs;
    private final PIDController turn_pid; 

    public swerve_module(
        int drive_motor_id, 
        int turn_motor_id, 
        int abs_encoder_id, 
        String module_name,
        double offset
    ) {
        drive_motor = new TalonFX(drive_motor_id, config.can_ivore);
        turn_motor = new TalonFX(turn_motor_id, config.can_ivore);

        drive_motor.getConfigurator().apply(config.swerve.drive_configs(InvertedValue.Clockwise_Positive));
        turn_motor.getConfigurator().apply(config.swerve.turn_configs(InvertedValue.Clockwise_Positive));

        abs = new DutyCycleEncoder(abs_encoder_id);
        abs.setDutyCycleRange(1.0 / 4096, 4095.0 / 4096);
        abs.setPositionOffset(get_abs_value() - offset);

        turn_pid = new PIDController(0.5, 0, 0); //TODO: adjust turn motor pid 
        turn_pid.enableContinuousInput(-Math.PI, Math.PI);

        SmartDashboard.putNumber("abs value " + module_name, get_abs_value());
    }

    public double get_drive_pos() {
        return drive_motor.getPosition().getValue();
    }

    public double get_turn_pos() {
        return turn_motor.getPosition().getValue();
    }

    public double get_drive_velocity() {
        return drive_motor.getVelocity().getValue();
    }

    public double get_turn_velocity() {
        return turn_motor.getVelocity().getValue();
    }

    public double get_abs_rad() {
        return abs.getDistancePerRotation() * Math.PI * 2;
    }
    
    public double get_abs_value() {
        return abs.getAbsolutePosition();
    }

    public Rotation2d get_rotation2d() {
        return Rotation2d.fromRadians(get_turn_pos());
    }

    public SwerveModuleState get_state() {
        return new SwerveModuleState(get_drive_velocity(), get_rotation2d());
    }

    public SwerveModulePosition get_position() {
        return new SwerveModulePosition(constants.swerve.wheel_circumference, get_rotation2d());
    }

    public void set_desired_state(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, get_state().angle);
        drive_motor.setControl(new PositionVoltage(state.speedMetersPerSecond / 2));
        turn_motor.setControl(new PositionVoltage(state.angle.getRotations()));

        SmartDashboard.putString("Swerve[" + abs.getSourceChannel() + "] state", state.toString());
    }

    public void stop() {
        drive_motor.set(0);
        turn_motor.set(0);
    }
}
