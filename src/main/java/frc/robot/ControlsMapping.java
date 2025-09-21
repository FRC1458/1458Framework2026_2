package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControlsMapping {
    public static final CommandXboxController controller = Robot.controller;
    public static void map() {
		controller.a().onTrue(Commands.runOnce(() -> DriverStationSim.setAllianceStationId(AllianceStationID.Blue1)));
    }
}
