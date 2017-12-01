var Diagnostics = require('../models/diagnostics');

exports.saveDiagnosticsData = function(req, res) {

	console.log(req.body);

	var diagnosticsData = new Diagnostics({

		batteryLevel: req.body.batteryLevel,
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

		res.send(200, req.body.batteryLevel);
	});
};

exports.getLatestDiagnosticsData = function(req, res) {

	Diagnostics.find({}, null, {sort: {'_id': -1}}, function(err, docs) {
		console.log("diagnosticsData");

		console.log(docs.length);
	 	//console.log(docs[docs.length-1]);

	 	if (docs.length == 0) {
	 		res.status(200).json("NO DATA");
	 	}
	 	else {

	 	//res.send(200, docs[docs.length-1]);
	 		res.status(200).json(docs[0]);
	 	//return docs[docs.length-1]
	 	//res.status(200).json(docs[docs.length-1]["id"]);
	 	//res.status(200).send(docs[docs.length-1]["id"]);
	 	}
	});

};