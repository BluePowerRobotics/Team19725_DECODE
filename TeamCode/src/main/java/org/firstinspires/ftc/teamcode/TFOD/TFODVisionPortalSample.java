//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import java.util.List;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//
//@TeleOp
//public class TFODVisionPortalSample extends LinearOpMode {
//    private TfodProcessor tfod;
//    private VisionPortal visionPortal;
//
//    @Override
//    public void runOpMode() {
//        // 1. 创建TfodProcessor
//        tfod = new TfodProcessor.Builder()
//                //.setModelAssetName("YourModel.tflite") // 或者用.setModelFileName()
//                //.setModelLabels(new String[]{"Label1", "Label2"})
//                .build();
//
//        // 2. 创建VisionPortal并添加TfodProcessor
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(tfod)
//                .build();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // 3. 获取识别结果
//            List<Recognition> recognitions = tfod.getRecognitions();
//            for (Recognition rec : recognitions) {
//                telemetry.addData("Label", rec.getLabel());
//                telemetry.addData("Confidence", rec.getConfidence());
//            }
//            telemetry.update();
//        }
//
//        visionPortal.close();
//    }
//}