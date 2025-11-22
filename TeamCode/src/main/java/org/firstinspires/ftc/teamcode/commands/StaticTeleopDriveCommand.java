package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

import java.util.function.DoubleSupplier;

public class StaticTeleopDriveCommand extends CommandBase {

    @Config
    public static class HeadingPID {
        public static double kp = 1.0;
        public static double ki = 0.0;
        public static double kd = 0.05;
        public static double kf = 0.0;
    }

    private static final double DEADBAND = 0.1;

    private final MecanumDrive drivetrain;
    private final Follower follower;
    private final DoubleSupplier rightStickY, rightStickX, leftStickX, rot, multiplier;

    private final PIDFController headingPID;
    private Double targetHeading = null; // null means no heading lock active

    public StaticTeleopDriveCommand(MecanumDrive drivetrain,
                                   Follower follower,
                                   DoubleSupplier rightStickY,
                                   DoubleSupplier rightStickX,
                                   DoubleSupplier leftStickX,
                                   DoubleSupplier rot,
                                   DoubleSupplier multiplier) {

        this.drivetrain = drivetrain;
        this.follower = follower;
        this.rightStickY = rightStickY;
        this.rightStickX = rightStickX;
        this.leftStickX = leftStickX;
        this.rot = rot;
        this.multiplier = multiplier;

        // Initialize PID controller for heading lock
        this.headingPID = new PIDFController(
                HeadingPID.kp,
                HeadingPID.ki,
                HeadingPID.kd,
                HeadingPID.kf
        );

        addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        // Update PID coefficients from FTC Dashboard
        headingPID.setPIDF(HeadingPID.kp, HeadingPID.ki, HeadingPID.kd, HeadingPID.kf);

        // Get rotation input from right stick
        double rotationInput = rot.getAsDouble();

        // Apply deadband to rotation input
        double rotationWithDeadband = MathUtils.applyDeadband(rotationInput, DEADBAND);

        double rotationOutput;

        if (Math.abs(rotationWithDeadband) > 0) {
            // Manual rotation control - right stick is being moved
            targetHeading = null; // Clear heading lock
            rotationOutput = rotationWithDeadband;
        } else {
            // Heading lock active - right stick is released
            double currentHeading = follower.getHeading();

            if (targetHeading == null) {
                // First frame of heading lock - save current heading
                targetHeading = currentHeading;
            }

            // Calculate heading error with angle wrapping
            double headingError = MathUtils.angleModulus(targetHeading - currentHeading);

            // Use PID to calculate rotation correction
            rotationOutput = headingPID.calculate(0, headingError);
        }

        // Execute drive with either manual rotation or PID-controlled heading lock
        drivetrain.teleopDrive(
                rightStickY.getAsDouble(),
                rightStickX.getAsDouble(),
                rotationOutput,
                multiplier.getAsDouble(),
                follower.getHeading()
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Clear heading lock when command ends
        targetHeading = null;
    }
}
