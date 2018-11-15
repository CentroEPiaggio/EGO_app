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

package com.example.ashwinvasudevan_dev.egorosimage;

import android.graphics.Rect;
import android.util.Log;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import java.text.SimpleDateFormat;
import java.util.Date;

public class RectNode extends AbstractNodeMain implements NodeMain {

    private static final String TAG = RectNode.class.getSimpleName();

    private float RectLeft = 0;
    private float RectTop = 0;
    private float RectRight = 0;
    private float RectBottom = 0;

    private String topicName;

    public void setTopicName(String topicName) {
        this.topicName = topicName;
    }


    public void setCoordArray(float rectLeft, float rectTop, float rectRight, float rectBottom) {
        RectLeft = rectLeft;
        RectTop = rectTop;
        RectRight = rectRight;
        RectBottom = rectBottom;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("Rect/Coordinates");
    }


    @Override
    public void onStart(ConnectedNode connectedNode) {
        final Publisher<std_msgs.Float32MultiArray> publisher = connectedNode.newPublisher(GraphName.of(topicName), std_msgs.Float32MultiArray._TYPE);
        final float[] coordArray = {RectLeft, RectTop, RectRight, RectBottom};
        final CancellableLoop loop = new CancellableLoop() {

            @Override
            protected void loop() throws InterruptedException {

                std_msgs.Float32MultiArray coord32Array = publisher.newMessage();
                coord32Array.setData(new float[]{RectLeft, RectTop, RectRight, RectBottom});


                publisher.publish(coord32Array);
            }
        };
        connectedNode.executeCancellableLoop(loop);
    }


}
