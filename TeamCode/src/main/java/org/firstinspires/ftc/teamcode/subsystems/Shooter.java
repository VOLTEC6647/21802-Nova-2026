package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Bot;

public class Shooter implements Subsystem {

    private DcMotorEx motor1;
    double currentDistance;
    public double targetVelocity;
    @Config
    public static class ShooterPIDF{
        public static double kp = 320;
        public static double ki = 0.00;
        public static double kd = 0.00;
         public static double   kf = 13.428 ;
    }

    private final Pose RedScore = new Pose(144, 136.5 );
    private final Pose BlueScore = new Pose(0, 136.5 );
    private MultipleTelemetry telemetry;



    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        motor1 = hardwareMap.get(DcMotorEx.class, "shooter");

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);

        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pid = new PIDFCoefficients(
                ShooterPIDF.kp,
                ShooterPIDF.ki,
                ShooterPIDF.kd,
                ShooterPIDF.kf);


        motor1.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                pid
        );


    }

    @Override
    public void periodic(){
        double currentVelocity = motor1.getVelocity();

        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("shooter pw", motor1.getPower());


        PIDFCoefficients pid = new PIDFCoefficients(
                ShooterPIDF.kp,
                ShooterPIDF.ki,
                ShooterPIDF.kd,
                ShooterPIDF.kf);


        motor1.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                pid
        );


    }
    public double tarVelRED(double x, double y){
        currentDistance = Math.hypot(RedScore.getX() - x, RedScore.getY() - y);

        return targetVelocity = 918.39 * Math.exp(0.0035 * currentDistance);
    }
    public double tarVelBLUE(double x, double y){
        currentDistance = Math.hypot(BlueScore.getX() - x, BlueScore.getY() - y);

        return  targetVelocity = 899.23 * Math.exp(0.0036 * currentDistance);

    }
    public void setTargetVelocity(double vel){
        motor1.setVelocity(vel);
    }
    public void setVelocityRED(double x, double y){
        currentDistance = Math.hypot(RedScore.getX() - x, RedScore.getY() - y);

        targetVelocity = 918.39 * Math.exp(0.0035 * currentDistance);
        motor1.setVelocity(targetVelocity);
    }
    public void setVelocityBLUE(double x, double y){
        currentDistance = Math.hypot(BlueScore.getX() - x, BlueScore.getY() - y);

        targetVelocity = 899.23 * Math.exp(0.0036 * currentDistance);
        motor1.setVelocity(targetVelocity);
    }
    public double getCurrentVelocity(){
        return motor1.getVelocity();
    }

}