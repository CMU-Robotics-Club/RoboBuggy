//LOADER 
$(window).load(function() {
	$("#pagecontent").show();
	$("#loader").remove();

});

console.log("running");

window.setInterval(function(){
    $.get("https://robobuggy-web-server.herokuapp.com/encoderData", function(data){
        var jsonString = JSON.stringify(data);
        var result = JSON.parse(jsonString);
        console.log("encoderresult " + result); 
        $('#ticks').text(result);
    });

    $.get("https://robobuggy-web-server.herokuapp.com/gpsData", function(gpsData){
        
        var jsonStringGPS = JSON.stringify(gpsData);
        var gpsresult = JSON.parse(jsonStringGPS);
        console.log("gpsresult " + gpsresult["latitude"] + " " + gpsresult["longitude"]); 
        $('#latitude').text(gpsresult["latitude"]);
        $('#longitude').text(gpsresult["longitude"]);
    });

    $.get("https://robobuggy-web-server.herokuapp.com/diagnosticsData", function(diagnosticsData){
        
        var jsonStringDiagnostics = JSON.stringify(diagnosticsData);
        var diagnosticsResult = JSON.parse(jsonStringDiagnostics);
        var batteryLevel = diagnosticsResult["batteryLevel"];
        var autonState = diagnosticsResult["autonState"];
        var diagnosticsError = diagnosticsResult["diagnosticsError"];
        console.log("diagnosticsResult " + batteryLevel + " " + autonState + " " + diagnosticsError); 
        $('#batteryLevel').text(batteryLevel + "mV");
        $('#autonState').text(autonState);
        $('#diagnosticsError').text(diagnosticsError);
    });

    $.get("https://robobuggy-web-server.herokuapp.com/brakesData", function(brakesData){
        
        var jsonStringBrakes = JSON.stringify(brakesData);
        var brakesResult = JSON.parse(jsonStringBrakes);

        var brakeState = brakesResult["brakeState"];
        var brakeCmdTeleop  = brakesResult["brakeCmdTeleop"]; 
        var brakeCmdAuton =  brakesResult["brakeCmdAuton"];
        console.log("diagnosticsResult " + brakeState + " " + brakeCmdTeleop + " " + brakeCmdAuton); 
        $('#brakeState').text(brakeState);
        $('#brakeCmdTeleop').text(brakeCmdTeleop);
        $('#brakeCmdAuton').text(brakeCmdAuton);
    });

}, 1000);
