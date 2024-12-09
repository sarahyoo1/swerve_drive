package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config;
import frc.robot.constants;

public class turret extends SubsystemBase {
    private final TalonFX motor;
    private final DutyCycleEncoder mini, minier;
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.2, 0.02);
    private final PIDController pid = new PIDController(0.5, 0, 0);

    private double target_deg = 0, target_degps = 0;
   
    public turret() {
        motor = new TalonFX(constants.ids.turret_motor, config.can_ivore);
        mini = new DutyCycleEncoder(constants.ids.turret_gear_mini);
        minier = new DutyCycleEncoder(constants.ids.turret_gear_minier);
        //mini.setDistancePerRotation(-1);
        mini.setDutyCycleRange(1.0/4096, 4095.0/4096);
        minier.setDutyCycleRange(1.0/4096, 4095.0/4096);
    }

    @Override
    public void periodic() {
        motor.setVoltage(ff.calculate(target_deg) + pid.calculate(get_position_degrees(), target_degps));
        SmartDashboard.putNumber("abs_minier", minier.getAbsolutePosition());
        SmartDashboard.putNumber("abs_mini", mini.getAbsolutePosition());
        SmartDashboard.putNumber("turret_target_deg", target_deg);
        SmartDashboard.putNumber("turret_target_degps", target_degps);
    }

    public double get_position_degrees() {
        double mini_angle = mini.getAbsolutePosition(); //TODO: get offest
        double minier_angle = minier.getAbsolutePosition();
        double modulo = (double) constants.turret.mini_gear / (double) constants.turret.main_gear;
        double minier_ratio = (double) constants.turret.minier_gear / (double) constants.turret.main_gear;

        double base = mini_angle * modulo; //first turret angle candidate
        int max_candidates = (int) Math.ceil((double) constants.turret.main_gear / (double) constants.turret.mini_gear);
        double smallest_diff = Double.MAX_VALUE;
        double closest = 0;
        for (int i = 0; i < max_candidates; ++i) {
            double turret_candidate = base + i * modulo;
            //double corresponding_minier = turret_candidate % minier_ratio; maybe?
            double corresponding_minier = (double) turret_candidate / (double) minier_ratio;
            if (Math.abs(corresponding_minier) > 1) {
                corresponding_minier = Math.abs(corresponding_minier - (int)corresponding_minier);
            }
            //or...
            // double corresponding_minier = MathUtil.inputModulus(turret_candidate / minier_ratio, 0, 1);
            double diff = Math.abs(minier_angle - corresponding_minier);
            if (diff < smallest_diff) {
                smallest_diff = diff;
                closest = turret_candidate;
            }
        }

        return Units.rotationsToDegrees(closest);
    }

    //TODO: can i change target values in smartdashboard?
    public void set_target(double degrees, double degrees_per_sec) {
        target_deg = degrees;
        target_degps = degrees_per_sec;
    }
}
