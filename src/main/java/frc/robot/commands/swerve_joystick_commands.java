package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve_subsystem;

public class swerve_joystick_commands extends Command {
    private final swerve_subsystem swerve_subsystem;
    private final Supplier<Double> left_x, left_y, right_x;
    private final Supplier<Boolean> field_orientation_func;
    private final SlewRateLimiter x_limiter, y_limiter, turn_limiter; //TODO: What are slew rate limiter?
    
    public swerve_joystick_commands(
        swerve_subsystem swerve_subsystem, 
        Supplier<Double> left_x, 
        Supplier<Double> left_y, 
        Supplier<Double> right_x, 
        Supplier<Boolean> field_orientation_func
    ) {
        this.swerve_subsystem = swerve_subsystem;
        this.left_x = left_x;
        this.left_y = left_y;
        this.right_x = right_x;
        this.field_orientation_func = field_orientation_func;
        x_limiter = new SlewRateLimiter(3);
        y_limiter = new SlewRateLimiter(3);
        turn_limiter = new SlewRateLimiter(3);
        addRequirements(swerve_subsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
       double x_speed = left_x.get();
       double y_speed = left_y.get();
       double turn_speed = right_x.get();

       x_speed = Math.abs(x_speed);
       y_speed = Math.abs(y_speed);
       turn_speed = Math.abs(turn_speed);
       
       double spd_factor = 0.5; //TODO: adjust speed
       x_speed = x_limiter.calculate(x_speed) * spd_factor;
       y_speed = y_limiter.calculate(y_speed) * spd_factor;
       turn_speed = turn_limiter.calculate(turn_speed) * spd_factor;
    
        ChassisSpeeds chassis_speeds;
        if (field_orientation_func.get()) {
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, turn_speed, swerve_subsystem.get_rotation2d());
        } else {
            chassis_speeds = new ChassisSpeeds(x_speed, y_speed, turn_speed);
        }
        
        SwerveModuleState[] module_states = Constants.swerve.drive_kinematics.toSwerveModuleStates(chassis_speeds);
        swerve_subsystem.set_module_states(module_states);
    }

    @Override
    public void end(boolean interrupted) {
        swerve_subsystem.stop_modules();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
}
