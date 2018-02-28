var express = require('express');
var router = express.Router();

var gpsDataController = require('../controllers/gpsDataController');

router.post('/', gpsDataController.saveGPSData);
router.get('/', gpsDataController.getLatestGPSData);

module.exports = router;
