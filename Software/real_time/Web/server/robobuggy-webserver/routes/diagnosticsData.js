var express = require('express');
var router = express.Router();

var diagnosticsDataController = require('../controllers/diagnosticsDataController');

router.post('/', diagnosticsDataController.saveDiagnosticsData);
router.get('/', diagnosticsDataController.getLatestDiagnosticsData);

module.exports = router;
