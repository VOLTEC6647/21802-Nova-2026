package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret implements Subsystem {

    private final DcMotorEx turret;
    private final MultipleTelemetry telemetry;
    private final Pose RedScore = new Pose(144, 136.5 );
    private final Pose BlueScore = new Pose(0, 140 );


    int targetPositionTicks = 0;
    @Config
    public static class TurretPIDF{
        public static double kp = 0.01;
        public static double ki = 0;
        public static double kd = 0;
        public static double kf = 0;
    }
    @Config
    public static class HoodRegression {
        public static double hoodMin = 0.0;
        public static double hoodMax = 1.0;
    }
    double TICKS_PER_DEGREE = 3.759555555555556;
    private final PIDFController pid = new PIDFController(
            TurretPIDF.kp,
            TurretPIDF.ki,
            TurretPIDF.kd,
            TurretPIDF.kf
    );
    double relativeAngleDegrees;
    double currentDistance = 0;
    Servo hood;

    public Turret(HardwareMap h, Telemetry t) {
        this.telemetry = new MultipleTelemetry(t, FtcDashboard.getInstance().getTelemetry());


        turret = h.get(DcMotorEx.class,"turret");
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hood = h.get(Servo.class, "hood");

        hood.setPosition(0);




    }
    public void resetEncoders(){
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    @Override
    public void periodic(){
        telemetry.addData("turret degrees", relativeAngleDegrees);
        telemetry.addData("current Degrees", turret.getCurrentPosition());
        double currentDegrees = turret.getCurrentPosition() / TICKS_PER_DEGREE;


        telemetry.addData("Current Degrees", currentDegrees);
        telemetry.addData("distance",currentDistance);


    }
    public void setTurretRED(double x, double y, double heading){
        currentDistance = Math.hypot(RedScore.getX() - x, RedScore.getY() - y);

        double targetServoPos = -1.907 + (0.499 * Math.log(currentDistance));
        double safeHoodPos = Range.clip(targetServoPos, HoodRegression.hoodMin, HoodRegression.hoodMax);

        hood.setPosition(safeHoodPos);




        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Heading", heading);


        pid.setPIDF(TurretPIDF.kp,
                TurretPIDF.ki,
                TurretPIDF.kd,
                TurretPIDF.kf);

        Vector2d targetVector = new Vector2d(
                RedScore.getX() - x,
                RedScore.getY() - y
        );

        double targetAngleRadians = targetVector.angle();
        double relativeAngleRadians = targetAngleRadians - heading;

        relativeAngleDegrees = Math.toDegrees(relativeAngleRadians);
        while (relativeAngleDegrees > 180) relativeAngleDegrees -= 360;
        while (relativeAngleDegrees <= -180) relativeAngleDegrees += 360;

        // Can turn 135 deg left
        double TURRET_MIN_ANGLE_DEGREES = -180.0;
        // Can turn 135 deg right
        double TURRET_MAX_ANGLE_DEGREES = 110.0;
        double MaxAngleDegrees = Math.max(TURRET_MIN_ANGLE_DEGREES,
                Math.min(TURRET_MAX_ANGLE_DEGREES, relativeAngleDegrees));
         //(384.5*3.52) / 360.00
        targetPositionTicks = (int) (MaxAngleDegrees * TICKS_PER_DEGREE);

        turret.setTargetPosition(targetPositionTicks);

        double pidOutput = pid.calculate(turret.getCurrentPosition(), targetPositionTicks);
        if (pidOutput > 1){
            pidOutput = 0.15;
        }

        if (pidOutput < -1){
            pidOutput = -0.15;
        }
        turret.setPower(pidOutput);


    }
    public void setTurretBLUE(double x, double y, double heading){
        currentDistance = Math.hypot(BlueScore.getX() - x, BlueScore.getY() - y);

        double targetServoPos = -1.907 + (0.499 * Math.log(currentDistance));
        double safeHoodPos = Range.clip(targetServoPos, HoodRegression.hoodMin, HoodRegression.hoodMax);

        hood.setPosition(safeHoodPos);

        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Heading", heading);


        pid.setPIDF(TurretPIDF.kp,
                TurretPIDF.ki,
                TurretPIDF.kd,
                TurretPIDF.kf);

        Vector2d targetVector = new Vector2d(
                BlueScore.getX() - x,
                BlueScore.getY() - y
        );

        double targetAngleRadians = targetVector.angle();

        double relativeAngleRadians = targetAngleRadians - heading;

        relativeAngleDegrees = Math.toDegrees(relativeAngleRadians);

        while (relativeAngleDegrees > 180) relativeAngleDegrees -= 360;
        while (relativeAngleDegrees <= -180) relativeAngleDegrees += 360;

        // Can turn 135 deg left
        double TURRET_MIN_ANGLE_DEGREES = -180.0;
        // Can turn 135 deg right
        double TURRET_MAX_ANGLE_DEGREES = 110.0;

        double MaxAngleDegrees = Math.max(TURRET_MIN_ANGLE_DEGREES,
                Math.min(TURRET_MAX_ANGLE_DEGREES, relativeAngleDegrees));

        //(384.5*3.52) / 360.00
        targetPositionTicks = (int) (MaxAngleDegrees * TICKS_PER_DEGREE);

        turret.setTargetPosition(targetPositionTicks);

        double pidOutput = pid.calculate(turret.getCurrentPosition(), targetPositionTicks);
        if (pidOutput > 1){
            pidOutput = 0.5;
        }

        if (pidOutput < -1){
            pidOutput = -0.5;
        }
        turret.setPower(pidOutput);


    }


}