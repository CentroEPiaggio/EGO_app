package com.example.ashwinvasudevan_dev.egorosimage;

import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import org.ros.android.MessageCallable;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;

import android.graphics.Matrix;

import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

public class RosSurfaceView<T> extends SurfaceView implements NodeMain, SurfaceHolder.Callback {

    private String topicName;
    private String messageType;

    public int getBitmapWidth() {
        return bitmapWidth;
    }

    public int getBitmapHeight() {
        return bitmapHeight;
    }

    private int bitmapWidth;
    private int bitmapHeight;

    private int phoneWidth;

    public int getPhoneWidth() {
        return phoneWidth;
    }

    public int getPhoneHeight() {
        return phoneHeight;
    }

    private int phoneHeight;

    private DisplayMetrics displayMetrics;

    private MessageCallable<Bitmap, T> callable;
    private SurfaceHolder surfaceHolder = null;
    private Paint paint = new Paint();
    private Canvas canvas;

    public void init() {
        displayMetrics = new DisplayMetrics();
        ((MainActivity) getContext()).getWindowManager().getDefaultDisplay().getMetrics(displayMetrics);
        phoneWidth = displayMetrics.widthPixels;
        phoneHeight = displayMetrics.heightPixels;
    }

    public RosSurfaceView(Context context) {
        super(context);
    }

    public RosSurfaceView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public RosSurfaceView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
    }

    public void setTopicName(String topicName) {
        this.topicName = topicName;

    }


    public void setMessageType(String messageType) {
        this.messageType = messageType;

    }


    public void setMessageToBitmapCallable(MessageCallable<Bitmap, T> callable) {
        this.callable = callable;

    }

    public void drawRect(float x, float y, float height) {

    }


    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ros_image_view");
    }

    public void onStart(ConnectedNode connectedNode) {
        surfaceHolder = getHolder();

        paint.setColor(Color.BLUE);
        // Making drawing smooth.
        paint.setAntiAlias(true);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(8);
        paint.setStrokeJoin(Paint.Join.ROUND);
        paint.setStrokeCap(Paint.Cap.ROUND);


        Subscriber<T> subscriber = connectedNode.newSubscriber(topicName, messageType);
        subscriber.addMessageListener(new MessageListener<T>() {

            @Override
            public void onNewMessage(final T message) {
                if (surfaceHolder.getSurface().isValid()) {
                    canvas = surfaceHolder.lockCanvas();

                    //Message without scaling
                    Bitmap bm = callable.call(message);

                    bitmapWidth = bm.getWidth();
                    bitmapHeight = bm.getHeight();

                    //Message with scaling
                    Bitmap scaledBm = Bitmap.createScaledBitmap(bm, phoneWidth, phoneHeight, true);

                    if (bm != null) {

                        canvas.drawBitmap(scaledBm, 0, 0, paint);

                    }
                    surfaceHolder.unlockCanvasAndPost(canvas);


                }
            }
        });
    }

    @Override
    public void onShutdown(Node node) {

    }

    @Override
    public void onShutdownComplete(Node node) {

    }

    @Override
    public void onError(Node node, Throwable throwable) {

    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {

    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {

    }
}
