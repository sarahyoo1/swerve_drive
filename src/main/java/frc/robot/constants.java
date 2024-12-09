package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class constants {

    public final class ids {
        public static final int can_swerve_fl_turn = 3;
        public static final int can_swerve_fr_turn = 9;
        public static final int can_swerve_bl_turn = 5;
        public static final int can_swerve_br_turn = 4;
        
        public static final int can_swerve_fl_drive = 6;
        public static final int can_swerve_fr_drive = 8;
        public static final int can_swerve_bl_drive = 7;
        public static final int can_swerve_br_drive = 2;
    
        public static final int dio_swerve_fl_abs = 8;
        public static final int dio_swerve_fr_abs = 6;
        public static final int dio_swerve_bl_abs = 0;
        public static final int dio_swerve_br_abs = 7;
        
        public static final int can_pigeon = 10;
        public static final int controller = 0;

        public static final int turret_motor = 14;
        public static final int turret_gear_mini = 2;
        public static final int turret_gear_minier = 3;
    }

    public final class swerve {
        public enum module_names {
            fr("fr"),
            fl("fl"),
            br("br"),
            bl("bl");

            public final String name;

            module_names(String name) {
                this.name = name;
            }
        }

        public enum module_e {
            mk4i_L1(8.14, 150.0/7.0, (25.0 / 19.0) * (15.0 / 45.0)),
            mk4i_L2(6.75, 150.0/7.0, (27.0 / 17.0) * (15.0 / 45.0)),
            mk4i_L3(6.12, 150.0/7.0, (28.0 / 16.0) * (15.0 / 45.0));
    
            public final double drive_ratio, steer_ratio, couple_ratio;

            module_e(double drive_ratio, double steer_ratio, double couple_ratio) {
                this.drive_ratio = drive_ratio;
                this.steer_ratio = steer_ratio;
                this.couple_ratio = couple_ratio;
            }
        }
        
        public static final double max_module_speed_mps = 4.572;
        public static final double max_speed_mps = 4.3;

        private static final double k = 1;
        public static final double wheel_diameter = Units.inchesToMeters(3.75) * k;
        public static final double wheel_radius = wheel_diameter / 2.0;
        public static final double wheel_circumference = wheel_diameter * Math.PI;
        public static final double track_width = Units.inchesToMeters(21);
        public static final SwerveDriveKinematics drive_kinematics = new SwerveDriveKinematics(
            new Translation2d(wheel_radius, track_width),
            new Translation2d(wheel_radius, track_width),
            new Translation2d(wheel_radius, track_width),
            new Translation2d(wheel_radius, track_width)
        );
    }

    public final class turret {
        public static final int main_gear = 46;
        public static final int mini_gear = 15;
        public static final int minier_gear = 11;
    }
}
