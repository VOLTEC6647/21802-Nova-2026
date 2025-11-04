package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Robot.startPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Turret implements Subsystem {

    private final DcMotorEx turret;
    private MultipleTelemetry telemetry;
    private final Vector2d BlueScore = new Vector2d(15.36, 129);
    private final double TICKS_PER_DEGREE = 1.49;
    private final Follower f;
    @Config
    public static class TurretPIDF{
        public static double kp = 1;
        public static double ki = 0;
        public static double kd = 0.05;
        public static double kf = -0.005;
    }
    private final PIDFController pid = new PIDFController(
            TurretPIDF.kp,
            TurretPIDF.ki,
            TurretPIDF.kd,
            TurretPIDF.kf
    );

    public Turret(HardwareMap h, Telemetry t) {
        this.telemetry = new MultipleTelemetry(t, FtcDashboard.getInstance().getTelemetry());


        turret = h.get(DcMotorEx.class,"turret");
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        f = Constants.createFollower(h);
        f.setStartingPose(startPose);


    }


    @Override
    public void periodic(){
        f.update();


        Vector2d targetVector = new Vector2d(
                BlueScore.getX() - f.getPose().getX(),
                BlueScore.getY() - f.getPose().getY()
        );

        double targetAngleRadians = targetVector.angle();

        double relativeAngleRadians = targetAngleRadians - f.getHeading();

        double relativeAngleDegrees = Math.toDegrees(relativeAngleRadians);


        while (relativeAngleDegrees > 180) relativeAngleDegrees -= 360;
        while (relativeAngleDegrees <= -180) relativeAngleDegrees += 360;

        // Can turn 135 deg left
        double TURRET_MIN_ANGLE_DEGREES = -135.0;
        // Can turn 135 deg right
        double TURRET_MAX_ANGLE_DEGREES = 135.0;

        double MaxAngleDegrees = Math.max(TURRET_MIN_ANGLE_DEGREES,
                Math.min(TURRET_MAX_ANGLE_DEGREES, relativeAngleDegrees));

        int targetPositionTicks = (int) (MaxAngleDegrees * TICKS_PER_DEGREE);

        turret.setTargetPosition(targetPositionTicks);

        double pidOutput = pid.calculate(turret.getCurrentPosition(), targetPositionTicks);

        turret.setPower(pidOutput);




    }
}