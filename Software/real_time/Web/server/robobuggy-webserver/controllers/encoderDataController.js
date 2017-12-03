var Encoder = require('../models/encoder');

exports.saveEncoderData = function(req, res) {

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

		console.log("Success saving encoderData");

		res.send(200, req.body.ticks);
	});
};

exports.getLatestEncoderData = function(req, res) {

	Encoder.find({}, null, {sort: {'_id': -1}}, function(err, docs) {
		console.log("encoderData");

		console.log(docs.length);

	 	if (docs.length == 0){
	 		res.status(200).json("NO DATA");
	 	}
	 	else {
	 		res.status(200).json(docs[0]["ticks"]);
	 	}
	});

};