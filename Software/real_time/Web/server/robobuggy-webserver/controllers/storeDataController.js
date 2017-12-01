var express = require('express');
var router = express.Router();

var encoderDataController = require('./encoderDataController');
var Encoder = require('../models/encoder');
var GPS = require('../models/gps');
var Diagnostics = require('../models/diagnostics');
var Brakes = require('../models/brakes')

exports.saveData = function(req, res) {

	console.log("Encode data ");
	console.log(req.body);

	var encoderData = new Encoder({

		ticks: req.body.ticks,
		timeStamp: Date.now()

	});

	console.log("encoderData: " + encoderData);

	encoderData.save(function(err) {
		if (err) {
			console.log("Error saving encoderData");
			res.send(404, "Error saving encoderData")
		}

		//res.render('index', {title:})

		console.log("Success saving encoderData");

		//res.send(200, req.body.ticks);
	});

	var gpsData = new GPS({
		latitude: req.body.latitude,
		longitude: req.body.longitude,
		timeStamp: Date.now()

	});

	console.log("gpsData: " + gpsData);

	gpsData.save(function(err) {
		if (err) {
			console.log("Error saving gpsData");
			res.send(400, "Error saving gpsData");
		}

		console.log("Success saving gpsData");

		//res.send(200, req.body.latitude);
		//res.send(req.body.long);
	});


	var diagnosticsData = new Diagnostics({

		batteryLevel: req.body.batteryLevel,
		autonState: req.body.autonState,
		diagnosticsError: req.body.diagnosticsError,
		timeStamp: Date.now()

	});

	console.log("diagnosticsData: " + diagnosticsData);

	diagnosticsData.save(function(err) {
		if (err) {
			console.log("Error saving diagnosticsData");
			res.send(404, "Error saving diagnosticsData");
		}

		//res.render('index', {title:})

		console.log("Success saving diagnosticsData");

		//res.send(200, req.body.batteryLevel);
	});

	var brakesData = new Brakes({

		brakeState: req.body.brakeState,
		brakeCmdTeleop: req.body.brakeCmdTeleop, 
		brakeCmdAuton: req.body.brakeCmdAuton,
		timeStamp: Date.now()
		
	});

	console.log("brakesData: " + brakesData);

	brakesData.save(function(err) {

		if(err) {
			console.log("Error saving brakesData");
			res.send(404, "Error saving brakesData");			
		}

		console.log("Success saving brakesData");

	});

	res.send(200, req.body);

};