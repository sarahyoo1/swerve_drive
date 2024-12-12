package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.swerve_subsystem;

public class swerve_joystick_commands extends Command {
    private final swerve_subsystem swerve_subsystem;
    private final Supplier<Double> left_x, left_y, right_x;
    private final SlewRateLimiter x_limiter, y_limiter, turn_limiter; 
    
    public swerve_joystick_commands(
        swerve_subsystem swerve_subsystem, 
        Supplier<Double> left_x, 
        Supplier<Double> left_y, 
        Supplier<Double> right_x 
    ) {
        this.swerve_subsystem = swerve_subsystem;
        this.left_x = left_x;
        this.left_y = left_y;
        this.right_x = right_x;
        x_limiter = new SlewRateLimiter(1); //TODO: set rate limit. (or do i rlly need it)
        y_limiter = new SlewRateLimiter(1);
        turn_limiter = new SlewRateLimiter(1);
        addRequirements(swerve_subsystem);
    }

    @Override
    public void execute() {
       double x_speed = -left_y.get();
       double y_speed = -left_x.get();
       double turn_speed = -right_x.get();
       
       double spd_factor = 0.4; //TODO: adjust speed factor
       x_speed = x_limiter.calculate(x_speed) * spd_factor;
       y_speed = y_limiter.calculate(y_speed) * spd_factor;
       turn_speed = turn_limiter.calculate(turn_speed) * spd_factor;
    
        ChassisSpeeds chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, turn_speed, swerve_subsystem.get_heading());
        SwerveModuleState[] module_states = constants.swerve.drive_kinematics.toSwerveModuleStates(chassis_speeds);
        swerve_subsystem.set_module_states(module_states);
    }

    @Override
    public void end(boolean interrupted) {
        swerve_subsystem.stop_modules();
    }
}
