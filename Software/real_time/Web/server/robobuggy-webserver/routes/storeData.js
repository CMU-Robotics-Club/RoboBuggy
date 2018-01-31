var express = require('express');
var router = express.Router();

var storeDataController = require('../controllers/storeDataController');

router.post('/', storeDataController.saveData);

module.exports = router;
