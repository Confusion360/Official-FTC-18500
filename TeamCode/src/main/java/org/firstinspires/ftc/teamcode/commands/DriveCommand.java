package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class DriveCommand extends CommandBase {
	private final DriveTrain dt;

	public DriveCommand(DriveTrain dt_p) {
		dt = dt_p;
		addRequirements(dt);
	}

	@Override
	public void execute() {
		dt.update();
	}

	@Override
	public boolean isFinished() { return false; }
}
