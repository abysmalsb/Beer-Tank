'use strict';

const Alexa = require('alexa-sdk');
const AWS = require('aws-sdk');

//Environment Configuration
var config = {};
config.IOT_BROKER_ENDPOINT      = "a16zyi8b5y4ebz.iot.us-west-2.amazonaws.com".toLowerCase();
config.IOT_BROKER_REGION        = "us-west-2";
config.IOT_THING_NAME           = "WalaBeerTank";
AWS.config.region = config.IOT_BROKER_REGION;
//Initializing client for IoT
var iotData = new AWS.IotData({endpoint: config.IOT_BROKER_ENDPOINT});

//Payload skeleton
var mqttPayload = {
    state: {
        desired : {
            led : "off",
            crane: "close",
            follow : "stop"
        }
    }
}

//Sending the payload to the AWS IoT and responding to the user with Alexa
function processing(mqttTopic, alexaResponse, alexa) {
	var paramsUpdate = {
            topic: mqttTopic,
            payload: JSON.stringify(mqttPayload),
            qos:0
        };
        iotData.publish(paramsUpdate, function(err, data) {
          if (err){
            //Handle the error here
            console.log("MQTT Error" + data);
          }
          else {
            console.log(data);
            alexa.emit(":ask", alexaResponse)
          }    
        });
}

const handlers = {

    'LaunchRequest': function() {
        this.emit(':ask', 'I\'m ready')
    },
    'LightSwitchIntent': function() {
        var command = this.event.request.intent.slots.state.value
        if (command == 'on') {
			mqttPayload.state.desired.led = command
			processing("wbt", 'Turning lighting on', this)
        }
        else if (command == 'off') {
			mqttPayload.state.desired.led = command
			processing("wbt", 'Turning lighting off', this)
        }
        else {
            this.emit(":ask", 'I don\'t understand your command')
        }
    },
    'CargoIntent': function() {
        var command = this.event.request.intent.slots.cargoState.value
        if (command == 'open') {
			mqttPayload.state.desired.crane = command
			processing("wbt", 'Elevating beer', this)
        }
        else if (command == 'close') {
			mqttPayload.state.desired.crane = command
			processing("wbt", 'Closing the cargo hold', this)
        }
        else {
            this.emit(":ask", 'I don\'t understand your command')
        }
    },
    'MovementIntent': function() {
        var command = this.event.request.intent.slots.movementCommand.value
        if (command == 'start' || command == 'follow me') {
			mqttPayload.state.desired.follow = 'start'
			processing("wbt", 'Beer Tank will follow you', this)
        }
        else if (command == 'stop') {
			mqttPayload.state.desired.follow = 'stop'
			processing("wbt", 'Beer Tank stopped', this)
        }
        else {
            this.emit(":ask", 'I don\'t understand your command ')
        }
    },
    "SessionEndedRequest": function() {
        this.emit()
    },
    "AMAZON.HelpIntent": function (intent, session, response) {
        this.emit(":ask", "You can ask me these: Turn on or off the lights. Follow you or stop. Open or close the cargo hold");
    },
    "Unhandled": function() {
        this.emit(":ask", 'I don\'t understand your command ')
    }
}

exports.handler = (event, context, callback) => {
    
    var alexa = Alexa.handler(event, context);
    alexa.registerHandlers(handlers);
    alexa.execute();
};
