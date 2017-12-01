var express = require('express');
var router = express.Router();

var encoderDataController = require('../controllers/encoderDataController');

/* GET home page. */
router.get('/', function(req, res, next) {

	// var data = encoderDataController.getLatestEncoderData(function(){
	// 	console.log("Data is");
	// 	console.log(data);
	// });

	// console.log("reached index route");
	//res.render('index', { title: 'Express' });
	
	res.sendFile(__dirname + '/web/index.html');
});

module.exports = router;
