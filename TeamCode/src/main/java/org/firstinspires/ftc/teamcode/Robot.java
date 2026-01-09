package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

import java.io.File;

public class Robot {
    private HardwareMap h;
    private Telemetry t;
    private final Gamepad g1a;
    private final Gamepad g2a;
    private final Gamepad g1;
    private final Gamepad g2;
    private final Turret tU;
    private final Shooter s;
    private final Intake i;
    private final Indexer iN;
    private final Follower f;
    public File myFileName;
    public String team;
    public static Pose startPoseRED = new Pose(97.382, 100.4597,0, PedroCoordinates.INSTANCE);
    public static Pose startPoseBLUE = new Pose(46.618, 100.4597,Math.toRadians(180), PedroCoordinates.INSTANCE);


    public Robot(HardwareMap h, Telemetry t, Gamepad g1a, Gamepad g2a) {

        this.h = h;
        this.t = t;
        this.g1a = g1a;
        this.g2a = g2a;

        tU = new Turret(this.h, this.t);
        s = new Shooter(this.h,this.t);
        i = new Intake(this.h,this.t);
        iN = new Indexer(this.h,this.t);

        f = Constants.createFollower(this.h);

        try {
            myFileName = AppUtil.getInstance().getSettingsFile("team.txt");
            team = ReadWriteFile.readFile(myFileName).trim();
        } catch (Exception e) {
            team = "red";
        }

        if (PoseStorage.currentPose != null) {
            f.setStartingPose(PoseStorage.currentPose);


        }
        else {
            if (team.equals("blue")){
                f.setStartingPose(startPoseBLUE);
            } else {
                f.setStartingPose(startPoseRED);
            }
        }

        this.g1 = new Gamepad();
        this.g2 = new Gamepad();


    }

    public void Controls(){

        g1.copy(g1a);
        g2.copy(g2a);

        if (team.equals("blue")){
            tU.setTurretBLUE(f.getPose().getX(),f.getPose().getY(),f.getPose().getHeading());
            f.setTeleOpDrive(g1.left_stick_y, g1.left_stick_x,   -g1.right_stick_x, false);
            s.setVelocityBLUE(f.getPose().getX(),f.getPose().getY());

            if (g1.options){
                f.setPose(null);
            }

        } else {
            tU.setTurretRED(f.getPose().getX(),f.getPose().getY(),f.getPose().getHeading());
            f.setTeleOpDrive(-g1.left_stick_y, -g1.left_stick_x,   -g1.right_stick_x, false);
            s.setVelocityRED(f.getPose().getX(),f.getPose().getY());

            if (g1.options){
                f.setPose(null);
            }

        }


        if(g1.a){
            i.setPower(1);
        } else {
            i.setPower(0);
        }
        if (g1.x){
            iN.setPower(1);
        } else {
            iN.setPower(0);
        }


    }
    public void tPeriodic() {
        tU.periodic();
        iN.periodic();
        i.periodic();
        s.periodic();  
        f.update();
        t.update();
        TelemetryPacket posePacket = new TelemetryPacket();
        posePacket.put("Pose x", f.getPose().getX());
        posePacket.put("Pose y", f.getPose().getY());
        posePacket.put("Pose heading", f.getPose().getHeading());
        FtcDashboard.getInstance().sendTelemetryPacket(posePacket);
        t.addData("Pose",f.getPose());
        //v.periodic();

    }
    public void tStart() {
        f.startTeleopDrive(true);
    }
}


