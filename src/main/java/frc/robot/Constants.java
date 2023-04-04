package frc.robot;

import java.util.Map;

import edu.wpi.first.math.controller.ArmFeedforward;

public class Constants {
  public static class Pivot {
    public static final int MASTER_MOTOR_ID = 5;
    public static final int SLAVE_MOTOR_ID = 6;
    public static final int CANCODER_ID = 0;
    public static final double GEAR_RATIO = 9;

    public static final double kP = 0.5;
    public static final double kI = 0;
    public static final double kD = 0;
    
    public static final Map<Double, ArmFeedforward> EXTEND_POSITION_TO_FEEDFORWARD_MAP =
        Map.of(
            0.0, new ArmFeedforward(0, 0, 0),
            20.0, new ArmFeedforward(0, 0, 0),
            40.0, new ArmFeedforward(0, 0, 0));

    public static final double MAX_VEL = 0;
    public static final double MAX_ACC = 0;

    public static final double DOWN_SETPOINT = GEAR_RATIO * 0;
    public static final double PICK_UP_SETPOINT = GEAR_RATIO * 30.0 / 360.0;
    public static final double SCORE_SETPOINT = GEAR_RATIO * 80.0 / 360.0;

    public static final double Y_SCALE = 0.025;

    public static final double BACK_HARD_LIMIT = GEAR_RATIO * 0.0 / 360.0;
    public static final double FRONT_HARD_LIMIT = GEAR_RATIO * 90.0 / 360.0;
  }

  public static class Extend {
    public static final double kP = 0.2;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double SETPOINT_TOLERANCE = 4;
    public static final double START_EXTENDING = 1;

    public static final int MOTOR_ID = 7; 

    // All voltage values are percent output
    public static final double BACK_VOLTAGE = 0.1; // Should be positive (has negative sign in code)
    public static final double BACK_TIME = 0.1; // Should be between 0 and SERVO_DELAY
    public static final double MAX_VOLTAGE_EXTEND = 0.5; // Value after comp; before it was 0.2
    public static final double MAX_VOLTAGE_RETRACT = 0.4; // Value after comp; before it was 0.28

    public static final int SERVO_PORT = 9;
    public static final double SERVO_DELAY = 0.7;
    public static final double RATCHET_ENGAGED = 115. / 180;
    public static final double RATCHET_DISENGAGED = 93. / 180;
    public static final double RATCHET_DELAY = 1;

    public static final double RETRACTED_SETPOINT = 0;
    public static final double MID_SETPOINT = 25.0;
    public static final double EXTENDED_SETPOINT = 43;
  }
}
