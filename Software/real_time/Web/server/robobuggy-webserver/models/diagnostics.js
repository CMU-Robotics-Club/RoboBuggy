var mongoose = require('mongoose');

var Schema = mongoose.Schema;

var diagnosticsSchema = Schema({
	batteryLevel: Number,
	autonState: Boolean,
	diagnosticsError: Number,
	timeStamp: Date
});

module.exports = mongoose.model('Diagnostics', diagnosticsSchema);
