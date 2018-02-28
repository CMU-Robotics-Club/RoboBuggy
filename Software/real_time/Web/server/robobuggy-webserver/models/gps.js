var mongoose = require('mongoose');

var Schema = mongoose.Schema;

var gpsSchema = Schema({
	latitude: Number,
	longitude: Number,
	timeStamp: Date
});

module.exports = mongoose.model('GPS', gpsSchema);
