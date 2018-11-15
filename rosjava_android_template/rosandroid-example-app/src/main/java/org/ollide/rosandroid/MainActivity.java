/*
 * Copyright (C) 2014 Oliver Degener.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ollide.rosandroid;

import android.annotation.SuppressLint;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.CompoundButton;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.SeekBar;
import android.widget.Switch;
import android.widget.TextView;

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.OrientationPublisher;
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

import android.widget.RadioGroup;
import android.widget.RadioGroup.OnCheckedChangeListener;

import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;

import io.github.controlwear.virtual.joystick.android.JoystickView;
import sensor_msgs.CompressedImage;
import std_msgs.Float64;
import std_msgs.Int32;
import std_msgs.Int32MultiArray;


public class MainActivity extends RosActivity {

    public MainActivity() {
        super("RosAndroidExample", "RosAndroidExample");
    }

    private RosImageView<CompressedImage> cameraView;
    private TextView mTextViewAngleRight;
    private TextView mTextViewStrengthRight;
    private TextView mTextViewCoordinateRight;

    private JoystickView joystickRightXY;
    private JoystickView joystickLeftXY;
    private SeekBar seekBarRight;
    private SeekBar seekBarLeft;
    private Switch switchLeft;
    private Switch switchRight;

    private SeekBar graspLeft;
    private SeekBar graspRight;

    private RadioButton headButton;
    private RadioButton bodyButton;
    private RadioButton defaultButton;
    private RadioButton bodyJButton;

    private RadioGroup mRadioGroup;
    private OrientationPublisher morientationPublisher;
    private SensorManager mSensorManager;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        morientationPublisher = new OrientationPublisher(mSensorManager);
        joystickRightXY = (JoystickView) findViewById(R.id.joystickView_rightXY);
        joystickLeftXY = (JoystickView) findViewById(R.id.joystickView_leftXY);
        seekBarRight = (SeekBar) findViewById(R.id.seekBarView_rightZ);
        seekBarLeft = (SeekBar) findViewById(R.id.seekBarView_leftZ);
        switchLeft = (Switch) findViewById(R.id.switchLeft);
        switchRight = (Switch) findViewById(R.id.switchRight);

        graspLeft = (SeekBar) findViewById(R.id.graspLeft);
        graspRight = (SeekBar) findViewById(R.id.graspRight);

        seekBarLeft.setProgress(100);
        seekBarLeft.setMax(200);
        seekBarRight.setProgress(100);
        seekBarRight.setMax(200);

        mRadioGroup = (RadioGroup) findViewById(R.id.radiogroup);

        headButton = (RadioButton) findViewById(R.id.radio_head);
        bodyButton = (RadioButton) findViewById(R.id.radio_body);
        defaultButton = (RadioButton) findViewById(R.id.radio_default);
        bodyJButton = (RadioButton) findViewById(R.id.radio_body_joystick);

        mRadioGroup.check(R.id.radio_default);
        defaultButton.setChecked(true);

        cameraView = (RosImageView<sensor_msgs.CompressedImage>) findViewById(R.id.image);
        cameraView.setMessageType(sensor_msgs.CompressedImage._TYPE);
        cameraView.setMessageToBitmapCallable(new BitmapFromCompressedImage());

//        mTextViewAngleRight = (TextView) findViewById(R.id.textView_angle_right);
//        mTextViewStrengthRight = (TextView) findViewById(R.id.textView_strength_right);
//        mTextViewCoordinateRight = findViewById(R.id.textView_coordinate_right);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        cameraView.setTopicName("/compressed_image");
        NodeMain node = new SimplePublisher();

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());

        nodeMainExecutor.execute(node, nodeConfiguration);
        nodeMainExecutor.execute(cameraView,
                nodeConfiguration.setNodeName("android/camera_view"));

        morientationPublisher.setTitle("android/orientation");
        nodeMainExecutor.execute(morientationPublisher, nodeConfiguration.setNodeName("android/mpublisher"));


    }

    public class SimplePublisher extends AbstractNodeMain implements NodeMain {

        @Override
        public GraphName getDefaultNodeName() {
            return GraphName.of("SimplePublisher/TimeLoopNode");
        }

        @Override
        public void onStart(ConnectedNode connectedNode) {

            final Publisher<Int32MultiArray> publisherRightMultiArray = connectedNode.newPublisher(GraphName.of("reference_r"), std_msgs.Int32MultiArray._TYPE);
            final Publisher<Int32MultiArray> publisherLeftMultiArray = connectedNode.newPublisher(GraphName.of("reference_l"), std_msgs.Int32MultiArray._TYPE);
            final Publisher<Int32> publisherOrientationState = connectedNode.newPublisher(GraphName.of("orientation_state"), std_msgs.Int32._TYPE);
            final Publisher<Float64> publisherLeftGrasp = connectedNode.newPublisher(GraphName.of("left_grasp"), std_msgs.Float64._TYPE);
            final Publisher<Float64> publisherRightGrasp = connectedNode.newPublisher(GraphName.of("right_grasp"), std_msgs.Float64._TYPE);
            final Publisher<Int32MultiArray> publisherBodyJoystick = connectedNode.newPublisher(GraphName.of("body_joystick"), std_msgs.Int32MultiArray._TYPE);

            final CancellableLoop loop = new CancellableLoop() {

                //Right
                int xJoyStickRight = 0;
                int yJoyStickRight = 0;
                int zSliderRight = 0;
                int rollRight = 0;
                int pitchRight = 0;
                int yawRight = 0;
                int[] rightArray = {xJoyStickRight, yJoyStickRight, zSliderRight, rollRight, pitchRight, yawRight};

                //Left
                int xJoyStickLeft = 0;
                int yJoyStickLeft = 0;
                int zSliderLeft = 0;
                int rollLeft = 0;
                int pitchLeft = 0;
                int yawLeft = 0;
                int[] leftArray = {xJoyStickLeft, yJoyStickLeft, zSliderLeft, rollLeft, pitchLeft, yawLeft};

                int[] bodyJoystick = {0, 0};

                int stateValue = 0;
                float leftGrasp = 0;
                float rightGrasp = 0;
                //orientation

                std_msgs.Int32 orientationState = publisherOrientationState.newMessage();
                std_msgs.Int32MultiArray rightMultiArray = publisherRightMultiArray.newMessage();
                std_msgs.Int32MultiArray leftMultiArray = publisherRightMultiArray.newMessage();

                std_msgs.Int32MultiArray bodyMultiArray = publisherRightMultiArray.newMessage();

                std_msgs.Float64 leftGraspValue = publisherLeftGrasp.newMessage();
                std_msgs.Float64 rightGraspValue = publisherLeftGrasp.newMessage();


                @Override
                protected void loop() throws InterruptedException {

                    rightMultiArray.setData(rightArray);
                    leftMultiArray.setData(leftArray);
                    orientationState.setData(stateValue);
                    leftGraspValue.setData(leftGrasp);
                    rightGraspValue.setData(rightGrasp);

                    joystickRightXY.setOnMoveListener(new JoystickView.OnMoveListener() {
                        @SuppressLint("DefaultLocale")
                        @Override
                        public void onMove(int angle, int strength) {

                            xJoyStickRight = (int) Math.round(Math.cos(angle * Math.PI / 180) * strength);
                            yJoyStickRight = (int) Math.round(Math.sin(angle * Math.PI / 180) * strength);

                            if (!switchRight.isChecked()) {
                                rightArray[0] = xJoyStickRight;
                                rightArray[1] = yJoyStickRight;
                                rightMultiArray.setData(rightArray);
                                Log.d("ArrayValue", String.valueOf(rightArray[0]) + " " + String.valueOf(rightArray[1]) + " " + String.valueOf(rightArray[2]));
                                publisherRightMultiArray.publish(rightMultiArray);
                            } else {
                                rightArray[3] = xJoyStickRight;
                                rightArray[4] = yJoyStickRight;
                                rightMultiArray.setData(rightArray);
                                Log.d("ArrayValue", String.valueOf(rightArray[0]) + " " + String.valueOf(rightArray[1]) + " " + String.valueOf(rightArray[2]));
                                publisherRightMultiArray.publish(rightMultiArray);
                            }

                            if (bodyJButton.isChecked()) {
                                bodyJoystick[1] = xJoyStickRight;
                                bodyMultiArray.setData(bodyJoystick);
                                publisherBodyJoystick.publish(bodyMultiArray);
                            }

                        }
                    });


                    joystickLeftXY.setOnMoveListener(new JoystickView.OnMoveListener() {
                        @Override
                        public void onMove(int angle, int strength) {
                            xJoyStickLeft = (int) Math.round(Math.cos(angle * Math.PI / 180) * strength);
                            yJoyStickLeft = (int) Math.round(Math.sin(angle * Math.PI / 180) * strength);
                            if (!switchLeft.isChecked()) {
                                leftArray[0] = xJoyStickLeft;
                                leftArray[1] = yJoyStickLeft;
                                rightMultiArray.setData(leftArray);
                                Log.d("ArrayValue", String.valueOf(leftArray[0]) + " " + String.valueOf(leftArray[1]) + " " + String.valueOf(leftArray[2]));
                                publisherLeftMultiArray.publish(leftMultiArray);
                            } else {
                                leftArray[3] = xJoyStickLeft;
                                leftArray[4] = yJoyStickLeft;
                                rightMultiArray.setData(leftArray);
                                Log.d("ArrayValue", String.valueOf(leftArray[0]) + " " + String.valueOf(leftArray[1]) + " " + String.valueOf(leftArray[2]));
                                publisherLeftMultiArray.publish(leftMultiArray);
                            }
                            if (bodyJButton.isChecked()) {
                                bodyJoystick[0] = yJoyStickLeft;
                                bodyMultiArray.setData(bodyJoystick);
                                publisherBodyJoystick.publish(bodyMultiArray);
                            }

                        }


                    });

                    seekBarRight.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                        @Override
                        public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                            if (!switchRight.isChecked()) {
                                rightArray[2] = progress - 100;
                                rightMultiArray.setData(rightArray);
                                Log.d("ArrayValue", String.valueOf(rightArray[0]) + " " + String.valueOf(rightArray[1]) + " " + String.valueOf(rightArray[2]));
                                publisherRightMultiArray.publish(rightMultiArray);
                            } else {
                                rightArray[5] = progress - 100;
                                rightMultiArray.setData(rightArray);
                                Log.d("ArrayValue", String.valueOf(rightArray[0]) + " " + String.valueOf(rightArray[1]) + " " + String.valueOf(rightArray[2]));
                                publisherRightMultiArray.publish(rightMultiArray);
                            }

                        }

                        @Override
                        public void onStartTrackingTouch(SeekBar seekBar) {

                        }

                        @Override
                        public void onStopTrackingTouch(SeekBar seekBar) {
                            seekBar.setProgress(100);
                        }
                    });


                    seekBarLeft.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                        @Override
                        public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                            if (!switchLeft.isChecked()) {
                                leftArray[2] = progress - 100;
                                leftMultiArray.setData(leftArray);
                                Log.d("ArrayValue", String.valueOf(leftArray[0]) + " " + String.valueOf(leftArray[1]) + " " + String.valueOf(leftArray[2]));
                                publisherLeftMultiArray.publish(leftMultiArray);
                            } else {
                                leftArray[5] = progress - 100;
                                leftMultiArray.setData(leftArray);
                                Log.d("ArrayValue", String.valueOf(leftArray[0]) + " " + String.valueOf(leftArray[1]) + " " + String.valueOf(leftArray[2]));
                                publisherLeftMultiArray.publish(leftMultiArray);

                            }
                        }

                        @Override
                        public void onStartTrackingTouch(SeekBar seekBar) {

                        }

                        @Override
                        public void onStopTrackingTouch(SeekBar seekBar) {
                            seekBar.setProgress(100);
                        }
                    });

                    graspLeft.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                        @Override
                        public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                            leftGrasp = (float) progress;
                            leftGraspValue.setData(leftGrasp);
                            publisherLeftGrasp.publish(leftGraspValue);
                        }

                        @Override
                        public void onStartTrackingTouch(SeekBar seekBar) {

                        }

                        @Override
                        public void onStopTrackingTouch(SeekBar seekBar) {

                        }
                    });

                    graspRight.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                        @Override
                        public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                            rightGrasp = (float) progress;
                            rightGraspValue.setData(leftGrasp);
                            publisherRightGrasp.publish(rightGraspValue);
                        }

                        @Override
                        public void onStartTrackingTouch(SeekBar seekBar) {

                        }

                        @Override
                        public void onStopTrackingTouch(SeekBar seekBar) {

                        }
                    });

//                    headButton.setOnClickListener(new View.OnClickListener() {
//                        @Override
//                        public void onClick(View v) {
//                            stateValue = 1;
//                            orientationState.setData(stateValue);
//                            publisherOrientationState.publish(orientationState);
//                        }
//                    });
//
//                    bodyButton.setOnClickListener(new View.OnClickListener() {
//                        @Override
//                        public void onClick(View v) {
//                            stateValue = 2;
//                            orientationState.setData(stateValue);
//                            publisherOrientationState.publish(orientationState);
//                        }
//                    });
//
//                    defaultButton.setOnClickListener(new View.OnClickListener() {
//                        @Override
//                        public void onClick(View v) {
//                            stateValue = 0;
//                            orientationState.setData(stateValue);
//                            publisherOrientationState.publish(orientationState);
//
//
//                        }
//                    });

                    mRadioGroup.setOnCheckedChangeListener(new OnCheckedChangeListener() {
                        public void onCheckedChanged(RadioGroup group, int checkedId) {


                            if (headButton.isChecked()) {
                                joystickLeftXY.setButtonDirection(0);
                                joystickRightXY.setButtonDirection(0);
                                stateValue = 1;
                                orientationState.setData(stateValue);
                                publisherOrientationState.publish(orientationState);

                            } else if (bodyButton.isChecked()) {
                                joystickLeftXY.setButtonDirection(0);
                                joystickRightXY.setButtonDirection(0);
                                stateValue = 2;
                                orientationState.setData(stateValue);
                                publisherOrientationState.publish(orientationState);

                            } else if (defaultButton.isChecked()) {
                                joystickLeftXY.setButtonDirection(0);
                                joystickRightXY.setButtonDirection(0);
                                stateValue = 0;
                                orientationState.setData(stateValue);
                                publisherOrientationState.publish(orientationState);

                            } else if (bodyJButton.isChecked()) {
                                stateValue = 3;
                                orientationState.setData(stateValue);
                                publisherOrientationState.publish(orientationState);
                                joystickLeftXY.setButtonDirection(1);
                                joystickRightXY.setButtonDirection(-1);
                            }

                        }

                    });

                }
            };
            connectedNode.executeCancellableLoop(loop);
        }
    }
}

