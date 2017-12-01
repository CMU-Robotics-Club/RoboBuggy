var Brakes = require('../models/brakes');

exports.getLatestBrakesData = function(req, res) {

	Brakes.find({}, null, {sort: {'_id': -1}}, function(err, docs) {
		
		console.log("brakesData");
		console.log(docs.length);

	 	if (docs.length == 0) {
	 		res.status(200).json("NO DATA");
	 	}
	 	else {
	 		res.status(200).json(docs[0]);
	 	}
	});

};