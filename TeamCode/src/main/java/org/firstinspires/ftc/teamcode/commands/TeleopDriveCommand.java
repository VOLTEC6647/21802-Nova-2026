package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends CommandBase {

    private final MecanumDrive drivetrain;
    private final DoubleSupplier rightStickY,rightStickX,leftStickX, rot, multiplier;

    public TeleopDriveCommand(MecanumDrive d, DoubleSupplier rightStickY, DoubleSupplier rightStickX, DoubleSupplier leftStickX
                              ,
                              DoubleSupplier rot,
                              DoubleSupplier multiplier) {

        this.drivetrain = d;
        this.rightStickY = rightStickY;
        this.rightStickX = rightStickX;
        this.leftStickX = leftStickX;
        this.rot = rot;
        this.multiplier = multiplier;

        addRequirements(this.drivetrain);
    }



    @Override
    public void execute() {
        drivetrain.teleopDrive(
                rightStickY.getAsDouble(),
                rightStickX.getAsDouble(),
                leftStickX.getAsDouble(),
                multiplier.getAsDouble(),
                leftStickX.getAsDouble()
        );
    }
}
