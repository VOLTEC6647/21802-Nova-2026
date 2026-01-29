
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

import java.io.File;

public class Robot {
    private HardwareMap h;
    private Telemetry t;
    double targetVelocity;
    private final Gamepad g1a;
    private final Gamepad g2a;
    private final Gamepad g1;
    private final Gamepad g2;
    private final Turret tU;
    private final Shooter s;
    private final Intake i;
    private final Stopper stopper;
    private final Hood hood;
    private final Indexer iN;
    private final Follower f;
    public File myFileName;
    public String team;

    @Config
    public static class ShooterConfigs {
        public static double TARGET_VEL = 800;
        public static double TARGET_HOOD = 800;


    }

    public static Pose startPoseRED = new Pose(105.5579, 132.711, Math.toRadians(0), PedroCoordinates.INSTANCE);
    public static Pose startPoseBLUE = new Pose(40, 58.500, Math.toRadians(180), PedroCoordinates.INSTANCE);

    public Robot(HardwareMap h, Telemetry t, Gamepad g1a, Gamepad g2a) {

        this.h = h;
        this.t = t;
        this.g1a = g1a;
        this.g2a = g2a;

        tU = new Turret(this.h, this.t);
        s = new Shooter(this.h, this.t);
        i = new Intake(this.h, this.t);
        iN = new Indexer(this.h, this.t);
        stopper = new Stopper(this.h, this.t);
        hood = new Hood(this.h, this.t);


        f = Constants.createFollower(this.h);

        try {
            myFileName = AppUtil.getInstance().getSettingsFile("team.txt");
            team = ReadWriteFile.readFile(myFileName).trim();
        } catch (Exception e) {
            team = "red";
        }

        if (PoseStorage.currentPose != null) {
            f.setStartingPose(PoseStorage.currentPose);


        } else {
            if (team.equals("blue")) {
                f.setStartingPose(startPoseBLUE);
            } else {
                f.setStartingPose(startPoseRED);
            }
        }

        this.g1 = new Gamepad();
        this.g2 = new Gamepad();


    }

    public void Controls() {

        g1.copy(g1a);
        g2.copy(g2a);

        boolean resetTurret = false;

        if (team.equals("blue")) {
            f.setTeleOpDrive(g1.left_stick_y, g1.left_stick_x, -g1.right_stick_x, false);
            s.setVelocityBLUE(f.getPose().getX(), f.getPose().getY());
            targetVelocity = s.tarVelBLUE(f.getPose().getX(), f.getPose().getY());




            if (resetTurret) {
                tU.setTurretToZero();
            } else {
                tU.setTurretBLUE(f.getPose().getX(), f.getPose().getY(), f.getPose().getHeading());
            }

        } else {
            f.setTeleOpDrive(-g1.left_stick_y, -g1.left_stick_x, -g1.right_stick_x, false);
            s.setVelocityRED(f.getPose().getX(), f.getPose().getY());
            targetVelocity = s.tarVelRED(f.getPose().getX(), f.getPose().getY());



            if (resetTurret) {
                tU.setTurretToZero();
            } else {
                tU.setTurretRED(f.getPose().getX(), f.getPose().getY(), f.getPose().getHeading());
            }
        }
        tPeriodic();

       /* if (g1.left_bumper){
            s.setTargetVelocity(ShooterConfigs.TARGET_VEL);
            hood.setPosition(ShooterConfigs.TARGET_HOOD);
        }*/

        // 1. Calculate if the shooter is at the target speed
        double currentVelocity = Math.abs(s.getCurrentVelocity());
        boolean isAtSpeed = (targetVelocity - currentVelocity < 40);

// 2. Control Logic
        if (g1.x && isAtSpeed) {
            // SHOOTING MODE: Only runs if button is held AND speed is reached
            i.setPower(0.95);
            stopper.setPosition(0); // Open stopper to fire

        } else if (g1.a) {
            // MANUAL INTAKE MODE: Runs regardless of shooter speed
            // This is your "other button" logic
            i.setPower(1);
            stopper.setPosition(1); // Keep stopper closed to hold rings

        } else {
            // IDLE MODE: No buttons pressed, or trying to shoot but not up to speed
            i.setPower(0);
            stopper.setPosition(1); // Keep stopper closed
        }


    }

    public void tPeriodic() {
        tU.periodic();
        iN.periodic();
        i.periodic();
        s.periodic();
        f.update();
        t.update();
        t.addData("TarVelR", s.tarVelRED(f.getPose().getX(), f.getPose().getY()));
        t.addData("TarVelB", s.tarVelRED(f.getPose().getX(), f.getPose().getY()));

        TelemetryPacket posePacket = new TelemetryPacket();
        posePacket.put("Pose x", f.getPose().getX());
        posePacket.put("Pose y", f.getPose().getY());
        posePacket.put("Pose heading", f.getPose().getHeading());
        FtcDashboard.getInstance().sendTelemetryPacket(posePacket);
        t.addData("Pose", f.getPose());
        //v.periodic();

    }

    public void tStart() {
        f.startTeleopDrive(true);
    }
}


