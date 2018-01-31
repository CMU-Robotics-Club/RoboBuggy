var mongoose = require('mongoose');

var Schema = mongoose.Schema;

var brakesSchema = Schema({
	brakeState: Boolean,
	brakeCmdTeleop: Boolean, 
	brakeCmdAuton : Boolean,
	timeStamp: Date
});

module.exports = mongoose.model('Brakes', brakesSchema);
