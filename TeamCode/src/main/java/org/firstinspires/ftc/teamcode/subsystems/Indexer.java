package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Bot;

public class Indexer implements Subsystem {

    private CRServo indexer;
    private CRServo indexer2;

    public double power = 0;
    private MultipleTelemetry telemetry;



    public Indexer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        indexer = hardwareMap.get(CRServo.class,"indexer");
        indexer2 = hardwareMap.get(CRServo.class,"indexer2");

        indexer.setDirection(DcMotorSimple.Direction.REVERSE);



    }


    @Override
    public void periodic(){

    }
    public void setPower(double power){
        power = power;
        indexer.setPower(power);
        indexer2.setPower(-power);



    }
}