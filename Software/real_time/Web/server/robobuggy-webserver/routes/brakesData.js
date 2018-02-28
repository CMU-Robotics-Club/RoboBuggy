var express = require('express');
var router = express.Router();

var brakesDataController = require('../controllers/brakesDataController');

router.get('/', brakesDataController.getLatestBrakesData);

module.exports = router;
