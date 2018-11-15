package com.example.ashwinvasudevan_dev.egorosimage;

import android.content.Intent;
import android.os.Bundle;
import android.support.design.widget.FloatingActionButton;
import android.support.design.widget.Snackbar;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.AppCompatAutoCompleteTextView;
import android.support.v7.widget.Toolbar;
import android.view.View;
import android.widget.AutoCompleteTextView;
import android.widget.Button;

public class TopicActivity extends AppCompatActivity {

    private AutoCompleteTextView subscribeTextView;
    private AutoCompleteTextView publishTextView;
    private Button connectButton;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_topic);

        subscribeTextView = (AutoCompleteTextView) findViewById(R.id.subscribeTopicTV);
        publishTextView = (AutoCompleteTextView) findViewById(R.id.publishTopicTV);
        connectButton = (Button) findViewById(R.id.connectButton);

        connectButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(TopicActivity.this, MainActivity.class);
                intent.putExtra("subscribeTopic", subscribeTextView.getText().toString());
                intent.putExtra("publishTopic", publishTextView.getText().toString());
                startActivity(intent);
            }
        });


    }

}
