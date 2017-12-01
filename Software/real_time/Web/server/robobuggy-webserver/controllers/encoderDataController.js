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

		//res.render('index', {title:})

		console.log("Success saving encoderData");

		res.send(200, req.body.ticks);
	});
};

exports.getLatestEncoderData = function(req, res) {

	Encoder.find({}, null, {sort: {'_id': -1}}, function(err, docs) {
		console.log("encoderData");

		console.log(docs.length);
	 	//console.log(docs[docs.length-1]);

	 	if (docs.length == 0){
	 		res.status(200).json("NO DATA");
	 	}
	 	else {

	 	//res.send(200, docs[docs.length-1]);
	 		res.status(200).json(docs[0]["ticks"]);
	 	//return docs[docs.length-1]
	 	//res.status(200).json(docs[docs.length-1]["id"]);
	 	//res.status(200).send(docs[docs.length-1]["id"]);
	 	}
	});

};