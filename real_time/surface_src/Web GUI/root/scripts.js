
$(document).ready(function(){/* google maps -----------------------------------------------------*/
google.maps.event.addDomListener(window, 'load', initialize);

function initialize() {
  var x;
  
  $.get("http://ipinfo.io", function(response) {
    $("#ip").html(response.ip);
    x = $("#address").html(response.loc);
  }, "jsonp");
  
  
  /*var latLngArray = x.split(",");
  var lat = latLngArray[0]
  var lng = latLngArray[1]*/
  /* position that map centers on (right now amsterdam) */
  var latlng = new google.maps.LatLng(52.3731, 4.8922);

  var mapOptions = {
    center: latlng,
    scrollWheel: false,
    zoom: 13
  };
  
  var marker = new google.maps.Marker({
    position: latlng,
    url: '/',
    animation: google.maps.Animation.DROP
  });
  
  var map = new google.maps.Map(document.getElementById("map-canvas"), mapOptions);
  marker.setMap(map);

};
/* end google maps -----------------------------------------------------*/
});