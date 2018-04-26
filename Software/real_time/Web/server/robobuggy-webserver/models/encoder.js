var mongoose = require('mongoose');

var Schema = mongoose.Schema;

var encoderSchema = Schema({
	ticks: Number,
	timeStamp: Date
});

module.exports = mongoose.model('Encoder', encoderSchema);