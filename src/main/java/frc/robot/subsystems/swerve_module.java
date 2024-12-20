package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config;

public class swerve_module {
    private final TalonFX drive_motor;
    private final TalonFX turn_motor;
    private final DutyCycleEncoder abs;
    private SwerveModuleState desired = new SwerveModuleState();

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
        abs.setPositionOffset(offset);

        SmartDashboard.putNumber("abs value " + module_name, abs.getAbsolutePosition());
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

    public SwerveModuleState get_state() {
        return new SwerveModuleState(get_drive_velocity(), Rotation2d.fromRotations(get_turn_pos()));
    }

    public SwerveModuleState get_desired_state() {
        return desired;
    }

    public void set_desired_state(SwerveModuleState state) {
        desired = SwerveModuleState.optimize(state, get_state().angle);
        drive_motor.setControl(new VelocityVoltage(state.speedMetersPerSecond));
        turn_motor.setControl(new PositionVoltage(state.angle.getRotations()));
        SmartDashboard.putString("Swerve[" + abs.getSourceChannel() + "] state", state.toString());
    }

    public void stop() {
        drive_motor.setVoltage(0);
        turn_motor.setVoltage(0);
    }
}
