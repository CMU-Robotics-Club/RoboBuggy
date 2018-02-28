var GPS = require('../models/gps');

exports.saveGPSData = function(req, res) {


	console.log(req.body);

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

		res.send(200, req.body.latitude);
	});
};

exports.getLatestGPSData = function(req, res) {

	GPS.find({}, null, {sort: {'_id': -1}}, function(err, docs) {
		console.log("gpsData");

		console.log(docs.length);

	 	if (docs.length == 0){
	 		res.status(200).json({"latitude": "NO DATA", "longitude": "NO DATA"});
	 	}
	 	else {
	 		res.status(200).json({"latitude": docs[0]["latitude"], "longitude": docs[0]["longitude"]});
	 	}
	});

};