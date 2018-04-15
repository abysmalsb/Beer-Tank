'use strict';

const Alexa = require('alexa-sdk');
const AWS = require('aws-sdk');

const HELP_MESSAGE = 'You can ask me these: Turn on or off the lights. Start or stop following you. Open or close the cargo hold and you can also say exit. What can I help you with?';
const HELP_REPROMPT = 'What can I help you with?';
const PROCEED = 'How would you like to proceed?';
const STOP_MESSAGE = 'Goodbye!';

//Environment Configuration
const config = {
    IOT_THING_NAME: "WalaBeerTank",
    IOT_BROKER_ENDPOINT: "a16zyi8b5y4ebz.iot.us-west-2.amazonaws.com",
    IOT_BROKER_REGION: "us-west-2"
};
AWS.config.region = config.IOT_BROKER_REGION;
//Initializing client for IoT
var iotData = new AWS.IotData({ endpoint: config.IOT_BROKER_ENDPOINT });

//Payload skeleton
var mqttPayload = {
    state: {
        desired: {
            led: "off",
            crane: "close",
            follow: "stop"
        }
    }
};

//Sending the payload to the AWS IoT and responding to the user with Alexa
function processing(mqttTopic, alexaResponse, alexa) {
    var paramsUpdate = {
        topic: mqttTopic,
        payload: JSON.stringify(mqttPayload),
        qos: 0
    };
    iotData.publish(paramsUpdate, function(err, data) {
        if (err) {
            //Handle the error here
            console.log("MQTT Error" + data);
        }
        else {
            console.log(data);
            alexa.response.speak(alexaResponse + " " + PROCEED).listen(alexaResponse + " " + PROCEED);
            alexa.emit(':responseReady');
        }
    });
}

const handlers = {

    'LaunchRequest': function() {
        const message = 'WalaBeer Tank is ready. ' + HELP_REPROMPT;
        this.response.speak(message).listen(message);
        this.emit(':responseReady');
    },
    'LightSwitchIntent': function() {
        var command = this.event.request.intent.slots.state.value;
        if (command == 'on') {
            mqttPayload.state.desired.led = command;
            processing("wbt", 'Turning lighting on.', this);
        }
        else if (command == 'off') {
            mqttPayload.state.desired.led = command;
            processing("wbt", 'Turning lighting off.', this);
        }
        else {
            this.response.speak('I don\'t understand your command.').listen('I don\'t understand your command.');
            this.emit(':responseReady');
        }
    },
    'CargoIntent': function() {
        var command = this.event.request.intent.slots.cargoState.value;
        if (command == 'open') {
            mqttPayload.state.desired.crane = command;
            processing("wbt", 'Elevating beer.', this);
        }
        else if (command == 'close') {
            mqttPayload.state.desired.crane = command;
            processing("wbt", 'Closing the cargo hold.', this);
        }
        else {
            this.response.speak('I don\'t understand your command.').listen('I don\'t understand your command.');
            this.emit(':responseReady');
        }
    },
    'MovementIntent': function() {
        var command = this.event.request.intent.slots.movementCommand.value;
        if (command == 'start' || command == 'follow me') {
            mqttPayload.state.desired.follow = 'start';
            processing("wbt", 'Beer Tank will follow you.', this);
        }
        else if (command == 'stop') {
            mqttPayload.state.desired.follow = 'stop';
            processing("wbt", 'Beer Tank stopped.', this);
        }
        else {
            this.response.speak('I don\'t understand your command.').listen('I don\'t understand your command.');
            this.emit(':responseReady');
        }
    },
    'AMAZON.HelpIntent': function() {
        const speechOutput = HELP_MESSAGE;
        const reprompt = HELP_REPROMPT;

        this.response.speak(speechOutput).listen(reprompt);
        this.emit(':responseReady');
    },
    'AMAZON.CancelIntent': function() {
        this.response.speak(STOP_MESSAGE);
        this.emit(':responseReady');
    },
    'AMAZON.StopIntent': function() {
        this.response.speak(STOP_MESSAGE);
        this.emit(':responseReady');
    },
    'Unhandled': function() {
        this.response.speak('I don\'t understand your command.').listen('I don\'t understand your command.');
        this.emit(':responseReady');
    },
};

exports.handler = (event, context, callback) => {
    const alexa = Alexa.handler(event, context, callback);
    alexa.registerHandlers(handlers);
    alexa.execute();
};
