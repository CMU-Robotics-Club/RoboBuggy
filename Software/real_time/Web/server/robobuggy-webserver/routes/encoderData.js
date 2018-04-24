var express = require('express');
var router = express.Router();

var encoderDataController = require('../controllers/encoderDataController');

router.post('/', encoderDataController.saveEncoderData);
router.get('/', encoderDataController.getLatestEncoderData);

module.exports = router;
