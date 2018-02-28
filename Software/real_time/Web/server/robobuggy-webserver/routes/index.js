var express = require('express');
var router = express.Router();

var encoderDataController = require('../controllers/encoderDataController');

/* GET home page. */
router.get('/', function(req, res, next) {
	
	res.sendFile(__dirname + '/web/index.html');
});

module.exports = router;
