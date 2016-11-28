package bagus.wisuda;

import android.content.Context;
import android.content.pm.ActivityInfo;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.provider.ContactsContract;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.Menu;
import android.view.SurfaceView;
import android.view.View;
import android.view.Window;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.MAVLink.common.msg_rc_channels_override;
import com.MAVLink.common.msg_set_position_target_local_ned;
import com.MAVLink.enums.MAV_FRAME;
import com.o3dr.android.client.ControlTower;
import com.o3dr.android.client.Drone;
import com.o3dr.android.client.apis.ControlApi;
import com.o3dr.android.client.apis.VehicleApi;
import com.o3dr.android.client.interfaces.DroneListener;
import com.o3dr.android.client.interfaces.TowerListener;
import com.o3dr.services.android.lib.coordinate.LatLong;
import com.o3dr.services.android.lib.coordinate.LatLongAlt;
import com.o3dr.services.android.lib.drone.action.ControlActions;
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent;
import com.o3dr.services.android.lib.drone.attribute.AttributeType;
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter;
import com.o3dr.services.android.lib.drone.connection.ConnectionResult;
import com.o3dr.services.android.lib.drone.connection.ConnectionType;
import com.o3dr.services.android.lib.drone.property.Altitude;
import com.o3dr.services.android.lib.drone.property.Gps;
import com.o3dr.services.android.lib.drone.property.Home;
import com.o3dr.services.android.lib.drone.property.Speed;
import com.o3dr.services.android.lib.drone.property.State;
import com.o3dr.services.android.lib.drone.property.VehicleMode;
import com.o3dr.services.android.lib.model.SimpleCommandListener;
import com.o3dr.services.android.lib.model.action.Action;

import org.droidplanner.services.android.impl.core.MAVLink.MavLinkCommands;
import org.droidplanner.services.android.impl.core.MAVLink.MavLinkRC;
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;

import static com.o3dr.services.android.lib.drone.property.VehicleMode.COPTER_ALT_HOLD;
import static com.o3dr.services.android.lib.drone.property.VehicleMode.COPTER_GUIDED;

public class MainActivity extends AppCompatActivity implements DroneListener, TowerListener, CvCameraViewListener2 {
    private static final int MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE = ((1 << 0) | (1 << 1) | (1 << 2));
    private static final int MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE = ((1 << 3) | (1 << 4) | (1 << 5));
    private static final int MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE = ((1 << 6) | (1 << 7) | (1 << 8));

    private static final String TAG = MainActivity.class.getSimpleName();
    private Drone drone;
    private ControlTower controlTower;
    private final Handler handler = new Handler();
    private static final int DEFAULT_USB_BAUD_RATE = 57600;

//    private static final  String tag = MainActivity.class.getCanonicalName();
    private Mat     mRgba;
    private Mat     mHsv;
    private Mat     mHsvMask;
    private Mat     mDilated;
    private Mat     mEroded;
    Mat hierarchy;
    String taString;
    List<MatOfPoint> contours;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

        final Context context = getApplicationContext();
        this.controlTower = new ControlTower(context);
        this.drone = new Drone(context);

        mOpenCVCameraView = (JavaCameraView) findViewById(R.id.MainActivityCameraView);
        mOpenCVCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCVCameraView.setCvCameraViewListener(this);
        mOpenCVCameraView.setMaxFrameSize(320, 240);
    }
    @Override
    public void onStart() {
        super.onStart();
        this.controlTower.connect(this);
    }
    @Override
    public void onStop() {
        super.onStop();
        if (this.drone.isConnected()) {
            this.drone.disconnect();
            updateConnectedButton(false);
        }
        this.controlTower.unregisterDrone(this.drone);
        this.drone.disconnect();
    }

    // Drone Listener
    // ==========================================================
    /*@Override
    public void onDroneConnectionFailed(ConnectionResult result) {
        alertUser("Connection Failed:" + result.getErrorMessage());
    }*/


    @Override
    public void onDroneEvent(String event, Bundle extras) {
        switch (event) {
            case AttributeEvent.STATE_CONNECTED:
                alertUser("Drone Connected");
                updateConnectedButton(this.drone.isConnected());
                updateArmButton();
                break;

            case AttributeEvent.STATE_VEHICLE_MODE:
                updateMode();
                State vehicleState = this.drone.getAttribute(AttributeType.STATE);
                VehicleMode vehicleMode = vehicleState.getVehicleMode();
                  if (vehicleMode==COPTER_ALT_HOLD){
//                if (//vehicleMode==COPTER_GUIDED) {
                      int [] rcOutputs = new int []{1600,1600,1500,1500,0,0,0,0};
//                      int [] rcOutputs = new int []{1,2,3,4,5,6,7,8};
                      MavLinkRC.sendRcOverrideMsg((MavLinkDrone)this.drone,rcOutputs); // i want to send pwm to to channel 1-4 from program
//                    double x=0, y = 0, z = 0;
//                    final int testCount = 1000;
//                    final ControlApi controlApi = ControlApi.getApi(drone);
//                    for (int i = 0; i < testCount; i++) {
///                     final float randomX = (float) ((Math.random() * 2) - 1f);
//                     final float randomY = (float) ((Math.random() * 2) - 1f);
//                     final float randomZ = (float) ((Math.random() * 2) - 1f);
//                        controlApi.enableManualControl(true,null);
//                        final float randomX=(float)1;
//                        final float randomY=(float)0;
//                        final float randomZ=(float)0;
//                        controlApi.manualControl(randomX, randomY, randomZ, null);
//                        controlApi.manualControl(randomX, randomY, randomZ, null);
//                        controlApi.manualControl(randomX, randomY, randomZ, null);
//                        taString = "RanX:"+Float.toString(randomX)+", RanY:"+Float.toString(randomY)+", RanZ:"+Float.toString(randomZ)+System.getProperty("line.separator");


                     //   x=x+10;
                     //   MavLinkCommands.sendGuidedVelocity((MavLinkDrone)this.drone, x, y,z);
 //                       MavLinkCommands.setVelocityInLocalFrame((MavLinkDrone)this.drone,2,0,0,null);

//                    }
//                    taString = "X:" + Double.toString(x) + ", Y:" + Double.toString(y) + ", Z:" + Double.toString(z) + System.getProperty("line.separator");
//                    writeToFile(taString);
                }
            case AttributeEvent.STATE_DISCONNECTED:
                alertUser("Drone Disconnected");
                updateConnectedButton(this.drone.isConnected());
                updateArmButton();
                break;

            case AttributeEvent.STATE_UPDATED:

            case AttributeEvent.SPEED_UPDATED:
                updateSpeed();
                break;

            case AttributeEvent.ALTITUDE_UPDATED:
                updateAltitude();
                break;

            case AttributeEvent.HOME_UPDATED:
                updateDistanceFromHome();
                break;
            case AttributeEvent.STATE_ARMING:
                alertUser("ARM");
                updateArmButton();
                return;

            default:
             //   Log.i("DRONE_EVENT", event); //Uncomment to see events from the drone
                break;
        }
    }
    @Override
    public void onDroneServiceInterrupted(String errorMsg) {

    }

    // 3DR Services Listener
    // ==========================================================
    @Override
    public void onTowerConnected() {
        alertUser("3DR Services Connected");
        this.controlTower.registerDrone(this.drone, this.handler);
        this.drone.registerDroneListener(this);
    }
    @Override
    public void onTowerDisconnected() {
        alertUser("3DR Service Interrupted");
    }

    // UI Events
    // ==========================================================
    public void onBtnConnectTap(View view) {
        if (this.drone.isConnected()) {
            this.drone.disconnect();
        } else {
            Bundle extraParams = new Bundle();
            extraParams.putInt(ConnectionType.EXTRA_USB_BAUD_RATE, DEFAULT_USB_BAUD_RATE); // Set default baud rate to 57600
            ConnectionParameter connectionParams = new ConnectionParameter(ConnectionType.TYPE_USB, extraParams, null);
            this.drone.connect(connectionParams);
        }
    }
    public void onArmButtonTap(View view) {
        State vehicleState = this.drone.getAttribute(AttributeType.STATE);

        if(vehicleState.isArmed()){
            VehicleApi.getApi(this.drone).arm(false);
        }else if (!vehicleState.isConnected()) {
            // Connect
            alertUser("Connect to a drone first");

        }else {
            // Connected but not Armed
//            VehicleApi.getApi(this.drone).arm(true);
            VehicleApi.getApi(this.drone). arm(true, false, new SimpleCommandListener() {
                @Override
                public void onError(int executionError) {
                    alertUser("Unable to arm vehicle.");
                }
                @Override
                public void onTimeout() {
                    alertUser("Arming operation timed out.");
                }
            });

        }
    }

    // UI updating
    // ==========================================================
    protected void updateConnectedButton(Boolean isConnected) {
        Button connectButton = (Button) findViewById(R.id.btnConnect);
        if (isConnected) {
            connectButton.setText("Disconnect");
        } else {
            connectButton.setText("Connect");
        }
    }
    protected void updateArmButton() {
        State vehicleState = this.drone.getAttribute(AttributeType.STATE);
        Button armButton = (Button) findViewById(R.id.btnArmTakeOff);

        if (!this.drone.isConnected()) {
            armButton.setVisibility(View.VISIBLE);
        } else if (vehicleState.isArmed()) {
            // Take off
            armButton.setText("DISARM");
        }else if (vehicleState.isConnected()) {
            // Connected but not Armed
            armButton.setText("ARM");
        }
    }
    protected void updateAltitude() {
        TextView altitudeTextView = (TextView) findViewById(R.id.altitudeValueTextView);
        Altitude droneAltitude = this.drone.getAttribute(AttributeType.ALTITUDE);
        altitudeTextView.setText(String.format("%3.1f", droneAltitude.getAltitude()) + "m");
    }
    protected void updateSpeed() {
        TextView speedTextView = (TextView) findViewById(R.id.speedValueTextView);
        Speed droneSpeed = this.drone.getAttribute(AttributeType.SPEED);
        speedTextView.setText(String.format("%3.1f", droneSpeed.getGroundSpeed()) + "m/s");
    }
    protected void updateDistanceFromHome() {
        TextView distanceTextView = (TextView) findViewById(R.id.distanceValueTextView);
        Altitude droneAltitude = this.drone.getAttribute(AttributeType.ALTITUDE);
        double vehicleAltitude = droneAltitude.getAltitude();
        Gps droneGps = this.drone.getAttribute(AttributeType.GPS);
        LatLong vehiclePosition = droneGps.getPosition();

        double distanceFromHome = 0;

        if (droneGps.isValid()) {
            LatLongAlt vehicle3DPosition = new LatLongAlt(vehiclePosition.getLatitude(), vehiclePosition.getLongitude(), vehicleAltitude);
            Home droneHome = this.drone.getAttribute(AttributeType.HOME);
            distanceFromHome = distanceBetweenPoints(droneHome.getCoordinate(), vehicle3DPosition);
        } else {
            distanceFromHome = 0;
        }

        distanceTextView.setText(String.format("%3.1f", distanceFromHome) + "m");
    }
    protected void updateMode(){
        TextView modeTextView = (TextView) findViewById(R.id.modeku);
        State vehicleState = this.drone.getAttribute(AttributeType.STATE);
        modeTextView.setText(String.format("%s",vehicleState.getVehicleMode()));
    }

    // Helper methods
    // ==========================================================
    protected void alertUser(String message) {
        Toast.makeText(getApplicationContext(), message, Toast.LENGTH_SHORT).show();
        Log.d(TAG, message);
    }
    protected double distanceBetweenPoints(LatLongAlt pointA, LatLongAlt pointB) {
        if (pointA == null || pointB == null) {
            return 0;
        }
        double dx = pointA.getLatitude() - pointB.getLatitude();
        double dy = pointA.getLongitude() - pointB.getLongitude();
        double dz = pointA.getAltitude() - pointB.getAltitude();
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    //PCD
    //===========================================================
    private BaseLoaderCallback mLoaderCallback= new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status){
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG,"OpenCV loaded successfuly");
                    mOpenCVCameraView.enableView();
                    break;

                }
                default:
                {
                    super.onManagerConnected(status);
                }
            }

        }
    } ;
    private JavaCameraView mOpenCVCameraView;
    @Override
    public boolean onCreateOptionsMenu(Menu menu){
        //getMenuInflater().inflate(R.menu.main, menu);
        //return true;
        return super.onCreateOptionsMenu(menu);

    }
    public void onResume(){
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, this, mLoaderCallback);
    }
    public void onDestroy(){
        super.onDestroy();
        if (mOpenCVCameraView !=null){
            mOpenCVCameraView.disableView();
        }
    }
    private void writeToFile(String data) {
        try {
            File sdCard = Environment.getExternalStorageDirectory();
            File directory = new File (sdCard.getAbsolutePath() + "/MyFiles");
            directory.mkdirs();
            boolean append =true;
            File file = new File(directory, "mysdfile.txt");
            FileOutputStream fOut = new FileOutputStream(file, append);
            OutputStreamWriter osw = new OutputStreamWriter(fOut);
            osw.write(data);
            osw.close();
        }
        catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }
    @Override
    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mHsv = new Mat(height,width,CvType.CV_8UC3);
        hierarchy = new Mat();
        mHsvMask = new Mat();
        mDilated = new Mat();
        mEroded = new Mat();
    }
    @Override
    public void onCameraViewStopped() {
        mRgba.release();
        mHsv.release();
        mHsvMask.release();
        mDilated.release();
        hierarchy.release();

    }
    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        long start = System.nanoTime();

        mRgba =inputFrame.rgba();
        contours = new ArrayList<MatOfPoint>();
        hierarchy =new Mat();
        mHsv = new Mat();
        mHsvMask =new Mat();

        Imgproc.cvtColor(mRgba, mHsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowerThreshold = new Scalar ( 0, 0, 0 ); // Blue color – lower hsv values
        Scalar upperThreshold = new Scalar ( 179, 255, 10 ); // Blue color – higher hsv values
        Core.inRange ( mHsv, lowerThreshold , upperThreshold, mHsvMask );

        Imgproc.dilate ( mHsvMask, mDilated, new Mat() );
        Imgproc.erode(mDilated,mEroded,new Mat());

        Imgproc.findContours(mDilated, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        double maxVal = 0;
        int maxValIdx = 0;
        for ( int contourIdx=0; contourIdx < contours.size(); contourIdx++ ) {
            double contourArea = Imgproc.contourArea(contours.get(contourIdx));
            if (maxVal < contourArea) {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }

            MatOfPoint2f approxCurve = new MatOfPoint2f();
            MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(maxValIdx).toArray());

            double approxDistance = Imgproc.arcLength(contour2f,true)*0.02;
            Imgproc.approxPolyDP(contour2f,approxCurve,approxDistance,true);

            MatOfPoint point = new MatOfPoint(approxCurve.toArray());
            Rect rect = Imgproc.boundingRect(point);

            if(Imgproc.contourArea(contours.get(maxValIdx))>1000&&Imgproc.contourArea(contours.get(maxValIdx))<35000)
            {
                Imgproc.drawContours(mRgba, contours, maxValIdx, new Scalar(0,255,0), 1);
                Imgproc.rectangle(mRgba,new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(255, 0, 0, 255),1);
                Imgproc.circle(mRgba,new Point(rect.x+(rect.width/2),rect.y+(rect.height/2)),3,new Scalar(255,0,0,255));

                //int rowBS =0,rowBE=120,colS=110,colE=210;
                //int rowAS =121,rowAE=240;
                //int roll=0;
                //int pitch=0;
                //int landing=0;
                //int PoinX=rect.x+(rect.width/2);
                //int PoinY=rect.y+(rect.height/2);

                //Mat roiTengahbawah= mRgba.submat(rowBS,rowBE,colS,colE);
                //Mat roiTengahatas = mRgba.submat(rowAS,rowAE,colS,colE);

//                if (PoinX<=colS&&PoinY<=rowBE){//room1
//                    roll=roll+15;
//                    pitch=pitch+50;
//                }else if (colS<=PoinX&&PoinX<=colE&&PoinY<=rowBE){//room2
//                    pitch=pitch+50;
//                }else if (PoinX>=colE&&PoinY<=rowBE){//room3
//                    roll=roll-15;
//                    pitch=pitch+50;
//                }else if (PoinX<=colS&&PoinY>=rowBE){//room4
//                    roll=roll+15;
//                }else if (colS<=PoinX&&PoinX<=colE&&PoinY>=rowBE){//room5
//                   pitch=pitch+10;
//                }else if (PoinX>=colE&&PoinY>=rowBE){//room6
//                    roll=roll-15;
//                }else{//no room
//                    landing = 100;
//                }

//                taString = "land:"+Integer.toString(landing)+", X:"+Integer.toString(PoinX)+", Y:"+Integer.toString(PoinY)+", Yaw:"+Integer.toString(roll)+",Picth:"+Integer.toString(pitch)+",Area:"+Double.toString(maxVal)+","+System.getProperty("line.separator");
//                Log.i("TAG","Land?:"+landing+"Koor X: "+PoinX+ " , Koor Y: " +PoinY+"Yaw: "+roll+ " , pitch: " +pitch+"area kontur: "+maxVal+ " , kontur IDX: " +maxValIdx);
            }
        }
        long finish = System.nanoTime()-start;
      //  taString="time = "+Long.toString(finish)+System.getProperty("line.separator");
      //  writeToFile(taString);

        Log.i("TAG","timee" +String.valueOf(finish));
        return mRgba;
    }
/*
    public void my_ned(MavLinkDrone drone,float pitch, float roll, float yaw){

        msg_set_position_target_local_ned msg = new msg_set_position_target_local_ned();
        msg.target_system= drone.getSysid();
        msg.target_component=drone.getCompid();
        msg.time_boot_ms=0;
        msg.coordinate_frame= MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
        msg.type_mask= MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE | MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        msg.vx=pitch;
        msg.vy=roll;
        msg.vx=yaw;
        drone.getMavClient().sendMessage(msg,null);

    }*/
}
