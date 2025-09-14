package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.lib.drivers.Phoenix6Util;

import java.util.HashMap;

public class TalonFXManager {
    /**
     * Holds all created TalonFX motor instances, keyed by a unique name
     */
    private static final Map<String, TalonFX> motors = new HashMap<>();

    /**
     * Creates and registers a motor with the given name, CAN ID, and applies a named default config.
     * @param name      Unique identifier for this motor
     * @param canId     CAN device ID
     * @param configKey Key of a default configuration to apply
     * @return The newly created TalonFX instance
     */
    public static TalonFX createMotor(String name, int canId, String configKey) {
        if (motors.containsKey(name)) {
            throw new IllegalArgumentException("Motor with name '" + name + "' already exists.");
        }
        TalonFX motor = new TalonFX(canId, "CV");
        TalonFXConfiguration config = DefaultConfigs.getConfig(configKey);
        Phoenix6Util.checkErrorAndRetry(()->
            motor.getConfigurator().apply(config, Constants.LONG_CANT_TIMEOUT_MS));
         motors.put(name, motor);
        return motor;
    }

    /**
     * Retrieves a previously registered motor by its name.
     * @param name Unique identifier
     * @return TalonFX instance
     */
    public static TalonFX getMotor(String name) {
        if (!motors.containsKey(name)) {
            throw new IllegalArgumentException("No motor registered under name '" + name + "'.");
        }
        return motors.get(name);
    }

    /**
     * Applies one of the default configs to an existing motor.
     * @param name      Registered motor name
     * @param configKey Key of the default configuration
     */
    public static void applyConfig(String name, String configKey) {
        TalonFX motor = getMotor(name);
        TalonFXConfiguration config = DefaultConfigs.getConfig(configKey);
        Phoenix6Util.checkErrorAndRetry(()->
            motor.getConfigurator().apply(config, Constants.LONG_CANT_TIMEOUT_MS));
    }

    /**
     * Static container for default motor configurations.
     */
    public static class DefaultConfigs {
        private static final Map<String, TalonFXConfiguration> configs = new HashMap<>();

        public static TalonFXConfiguration AngleMotorConfig () {
                TalonFXConfiguration ANGLE_CONFIG = new TalonFXConfiguration();
                ANGLE_CONFIG.Slot0.kP = Constants.Drive.ANGLE_MOTOR_PIDF_CONSTANTS.kP;
                ANGLE_CONFIG.Slot0.kI = Constants.Drive.ANGLE_MOTOR_PIDF_CONSTANTS.kI;
                ANGLE_CONFIG.Slot0.kD = Constants.Drive.ANGLE_MOTOR_PIDF_CONSTANTS.kD;
                ANGLE_CONFIG.Slot0.kS = 0.0;
                ANGLE_CONFIG.Slot0.kV = Constants.Drive.ANGLE_MOTOR_PIDF_CONSTANTS.kF;

                ANGLE_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
                ANGLE_CONFIG.CurrentLimits.StatorCurrentLimit = Constants.Drive.ANGLE_CURRENT_LIMIT;

                ANGLE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
                ANGLE_CONFIG.CurrentLimits.SupplyCurrentLimit = Constants.Drive.ANGLE_CURRENT_LIMIT;
                ANGLE_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = Constants.Drive.ANGLE_CURRENT_THRESHOLD;
                ANGLE_CONFIG.CurrentLimits.SupplyCurrentLowerTime = Constants.Drive.ANGLE_CURRENT_THRESHOLD_TIME;

                ANGLE_CONFIG.Voltage.PeakForwardVoltage = 12.0;
                ANGLE_CONFIG.Voltage.PeakReverseVoltage = -12.0;

                ANGLE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//                configs.put("Angle", ANGLE_CONFIG);
                return ANGLE_CONFIG;
            }

            public static TalonFXConfiguration DriveMotorConfig(){
                TalonFXConfiguration DRIVE_CONFIG = new TalonFXConfiguration();

                DRIVE_CONFIG.Slot0.kP = Constants.Drive.DRIVE_MOTOR_PIDF_CONSTANTS.kP;
                DRIVE_CONFIG.Slot0.kI = Constants.Drive.DRIVE_MOTOR_PIDF_CONSTANTS.kI;
                DRIVE_CONFIG.Slot0.kD = Constants.Drive.DRIVE_MOTOR_PIDF_CONSTANTS.kD;
                DRIVE_CONFIG.Slot0.kS = 0.1;
                DRIVE_CONFIG.Slot0.kV = Constants.Drive.DRIVE_MOTOR_PIDF_CONSTANTS.kF;

                DRIVE_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
                DRIVE_CONFIG.CurrentLimits.StatorCurrentLimit = Constants.Drive.DRIVE_CURRENT_LIMIT;

                DRIVE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
                DRIVE_CONFIG.CurrentLimits.SupplyCurrentLimit = Constants.Drive.DRIVE_CURRENT_LIMIT;
                DRIVE_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = Constants.Drive.DRIVE_CURRENT_THRESHOLD;
                DRIVE_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 0.5;

                DRIVE_CONFIG.Voltage.PeakForwardVoltage = 12.0;
                DRIVE_CONFIG.Voltage.PeakReverseVoltage = -12.0;

                DRIVE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                DRIVE_CONFIG.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Drive.OPEN_LOOP_RAMP;
                DRIVE_CONFIG.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Drive.OPEN_LOOP_RAMP;
                //configs.put("Drive", DRIVE_CONFIG);
                return DRIVE_CONFIG;
            }
            {
                TalonFXConfiguration DEFAULT_CONFIG = new TalonFXConfiguration();
                DEFAULT_CONFIG.Slot0.kP = Constants.Drive.ANGLE_MOTOR_PIDF_CONSTANTS.kP;
                DEFAULT_CONFIG.Slot0.kI = Constants.Drive.ANGLE_MOTOR_PIDF_CONSTANTS.kI;
                DEFAULT_CONFIG.Slot0.kD = Constants.Drive.ANGLE_MOTOR_PIDF_CONSTANTS.kD;
                DEFAULT_CONFIG.Slot0.kS = 0.0;
                DEFAULT_CONFIG.Slot0.kV = Constants.Drive.ANGLE_MOTOR_PIDF_CONSTANTS.kF;

                DEFAULT_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
                DEFAULT_CONFIG.CurrentLimits.StatorCurrentLimit = Constants.Drive.ANGLE_CURRENT_LIMIT;

                DEFAULT_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
                DEFAULT_CONFIG.CurrentLimits.SupplyCurrentLimit = Constants.Drive.ANGLE_CURRENT_LIMIT;
                DEFAULT_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = Constants.Drive.ANGLE_CURRENT_THRESHOLD;
                DEFAULT_CONFIG.CurrentLimits.SupplyCurrentLowerTime = Constants.Drive.ANGLE_CURRENT_THRESHOLD_TIME;

                DEFAULT_CONFIG.Voltage.PeakForwardVoltage = 12.0;
                DEFAULT_CONFIG.Voltage.PeakReverseVoltage = -12.0;

                DEFAULT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                configs.put("Default", DEFAULT_CONFIG);
            }
        
        /**
         * Retrieves a named default config, or throws if not found.
         */
        public static TalonFXConfiguration getConfig(String key) {
            if (key.matches("Angle")){
                return AngleMotorConfig();
            }else if (key.matches("Drive")){
                return DriveMotorConfig();
            }else{
                return null;
            }
            // TalonFXConfiguration cfg = configs.get(key);
            // if (cfg == null) {
            //     throw new IllegalArgumentException("No default config found for key '" + key + "'.");
            // }
            // return cfg;
        }

        /**
         * Allows registration of additional default configs at runtime.
         */
        public static void registerConfig(String key, TalonFXConfiguration config) {
            if (configs.containsKey(key)) {
                throw new IllegalArgumentException("Config key '" + key + "' already registered.");
            }
            configs.put(key, config);
        }
    }
}
