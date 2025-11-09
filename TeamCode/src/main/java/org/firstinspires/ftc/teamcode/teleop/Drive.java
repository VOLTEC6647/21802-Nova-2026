package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp
public class Drive extends OpMode {
    Robot r;

    @Override
    public void init() {

        r = new Robot(hardwareMap,telemetry,gamepad1,gamepad2);
    }

    @Override
    public void start() {
        r.tStart();
    }

    @Override
    public void loop() {
        r.Controls();
        r.tPeriodic();
    }
}
