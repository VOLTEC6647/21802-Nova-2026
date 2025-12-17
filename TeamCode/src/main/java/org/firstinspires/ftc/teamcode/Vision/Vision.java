
package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import lombok.Getter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Config
public class Vision extends SubsystemBase {

    private final Limelight3A camera;
    @Getter private LLResult result;

    public static double TURN_P = 0.05;
    private MultipleTelemetry telemetry;




    public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        camera = hardwareMap.get(Limelight3A.class, "limelight");

        pipeline(0);
        initializeCamera();

    }

    public void initializeCamera() {
        camera.setPollRateHz(100);
        camera.start();
    }
    public void pipeline(int switchPipeline){
        camera.pipelineSwitch(switchPipeline);
    }

    public double getTurnPower() {
        if (!result.isValid()) {
            return 0.0;
        }
        double tx = getTx();

        return -tx * TURN_P;
    }

    public double getTx() {
        if (result == null) {
            return 0;
        }
        return result.getTx();
    }

    public double getTy() {
        if (result == null) {
            return 0;
        }
        return result.getTy();
    }
    public Pose mt(double x, double y, double heading) {
        Pose jijija = new Pose(x,y,heading);
        if (result != null && result.isValid()) {
       Pose3D botpose = result.getBotpose();

        Position poseInches=botpose.getPosition().toUnit(DistanceUnit.INCH);

        jijija = new Pose(72 - poseInches.x, poseInches.y + 72 , heading, InvertedFTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);


        }


        return jijija;


    }

    public Pose getPedroPose() {
        LLResult result = camera.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();

            Position poseInches = botpose.getPosition().toUnit(DistanceUnit.INCH);

            Pose rawPose = new Pose(
                    72 - poseInches.x,
                    poseInches.y + 72,
                    botpose.getOrientation().getYaw(),
                    InvertedFTCCoordinates.INSTANCE
            );

            return rawPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        }

        // Return null (or a default Pose) if no valid data is found
        return null;

    }




    @Override
    public void periodic() {

        result = camera.getLatestResult();


        Pose posePedro;
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            Position poseInches=botpose.getPosition().toUnit(DistanceUnit.INCH);
            posePedro = new Pose(72 - poseInches.x, poseInches.y + 72 , botpose.getOrientation().getYaw(), InvertedFTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE.INSTANCE);
            telemetry.addData("pedroPoints", "x="+posePedro.getX()+";y="+posePedro.getY());
            telemetry.addData("Ty", getTy());
            telemetry.addData("Tx", getTx());
        }





    }
}
