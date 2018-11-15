package com.example.ashwinvasudevan_dev.egorosimage;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PixelFormat;
import android.graphics.PorterDuff;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.TextView;

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Subscriber;

import sensor_msgs.CompressedImage;

public class MainActivity extends RosActivity {

    private RosSurfaceView<CompressedImage> compressedImage;
    private SurfaceHolder compressedImageHolder;

    private SurfaceView shapeView;
    private SurfaceHolder shapeViewHolder;

    private Bundle extras;
    private String publisherTopic;
    private String subscribeTopic;

    private Canvas canvas;
    private Paint paint;

    private float RectLeft, RectTop, RectRight, RectBottom;

    private RectNode rectNode;


    public MainActivity() {
        super("EgoObjectGrasping", "EgoObjectGrasping");
    }

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Bundle extras = getIntent().getExtras();

        publisherTopic = extras.getString("publishTopic");
        subscribeTopic = extras.getString("subscribeTopic");


        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);


        setContentView(R.layout.activity_main);
        //Full Screen

        compressedImage = (RosSurfaceView<sensor_msgs.CompressedImage>) findViewById(R.id.rosImageView);
        shapeView = (SurfaceView) findViewById(R.id.shapeView);

        compressedImage.init();

        compressedImage.setTopicName(subscribeTopic);
        compressedImage.setMessageType(sensor_msgs.CompressedImage._TYPE);
        compressedImage.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        compressedImage.setZOrderOnTop(false);
        compressedImage.setOnTouchListener(onTouchListener);

        compressedImageHolder = compressedImage.getHolder();
        compressedImageHolder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);

        shapeView.setZOrderOnTop(true);
        shapeView.setZOrderMediaOverlay(true);

        shapeViewHolder = shapeView.getHolder();
        shapeViewHolder.setFormat(PixelFormat.TRANSPARENT);
        shapeViewHolder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);


    }


    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {

        rectNode = new RectNode();
        rectNode.setTopicName(publisherTopic);

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),
                getMasterUri());
        nodeConfiguration.setMasterUri(getMasterUri());

        nodeMainExecutor.execute(compressedImage, nodeConfiguration.setNodeName("android/video_view"));
        nodeMainExecutor.execute(rectNode, nodeConfiguration.setNodeName("android/rect_coord"));

    }

    private void DrawFocusRect(float RectLeft, float RectTop, float RectRight, float RectBottom, int color) {

        canvas = shapeViewHolder.lockCanvas();
        canvas.drawColor(0, PorterDuff.Mode.CLEAR);
        //border's properties
        paint = new Paint();
        paint.setStyle(Paint.Style.STROKE);
        paint.setColor(color);
        paint.setStrokeWidth(3);
        canvas.drawRect(RectLeft, RectTop, RectRight, RectBottom, paint);


        shapeViewHolder.unlockCanvasAndPost(canvas);
    }

    View.OnTouchListener onTouchListener = new View.OnTouchListener() {

        @Override
        public boolean onTouch(View v, MotionEvent event) {

            if (event.getPointerCount() == 2) {

                RectLeft = event.getX(0);
                RectTop = event.getY(0);

                RectRight = event.getX(1);
                RectBottom = event.getY(1);

                DrawFocusRect(RectLeft, RectTop, RectRight, RectBottom, Color.BLUE);

                //Normalised values are transmitted on the topic
                //Formula = (Value * Desired Range) / Actual Range
                rectNode.setCoordArray(
                        (RectLeft * compressedImage.getBitmapWidth()) / compressedImage.getPhoneWidth(),
                        (RectTop * compressedImage.getBitmapHeight()) / compressedImage.getPhoneHeight(),
                        (RectRight * compressedImage.getBitmapWidth()) / compressedImage.getPhoneWidth(),
                        (RectBottom * compressedImage.getBitmapHeight()) / compressedImage.getPhoneHeight());

            }
            return true;
        }
    };
}
