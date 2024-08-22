//set up udp connection with camera
const dgram = require("dgram");
const siYiCameraClient = dgram.createSocket("udp4"); //CREATING A UDP SOCKET TO CONNECT TO THE CAMERA

const SiyiA8SDK = require("../SiyiCameraSDK"); //IMPORTING SIYI CALSS
const mySiyiCamera = new SiyiA8SDK();

const SIYI_CAMERA_UDP_IP_ADDERSS = "192.168.2.3"; // REPLACE WITH THE CAMERA's IP ADDRESS
const SIYI_CAMERA_UDP_PORT = 37260; // THIS IS THE DEFAULT PORT CHANGE AS PER NEED

//-------------------------------------------------------------------------------------------------------------------
var zoomPosition = 0*20;   //Position can only be between 0(full zoom out) and 2000(full zoom in), initialized to 0 for testing
var guiPercentZoom = 50;   //*****CHANGE ME - add event to correspond to GUI******
var desiredZoomPosition = guiPercentZoom * 20;
var amtPositionChange = 0;
  
//print to console starting a new zoom
console.log("Initial Zoom: " + (zoomPosition / 20) + "%");   //print starting zoom position to console
console.log("Desired Zoom: " + (desiredZoomPosition / 20) + "%");

//**Zoom IN case**-----------------------------------------------------------------------------------------------------
if(desiredZoomPosition > zoomPosition){

  //See current position and calculate how much further needed to zoom in
  if(zoomPosition == 0) {    //if already at max zoom out(0)
      amtPositionChange = desiredZoomPosition;
    } else if(zoomPosition == 2000){   //if already at max zoom in(2000)
      amtPositionChange = 0;
    } else{   //if in between max zoom out and zoom in
      amtPositionChange = desiredZoomPosition - zoomPosition;
    }

  console.log("Starting Zoom");
  siYiCameraClient.send(
    mySiyiCamera.start_zoom(),
    SIYI_CAMERA_UDP_PORT,
    SIYI_CAMERA_UDP_IP_ADDERSS,
    (err) => {
      if (err) {
        console.error("Error sending data:", err);
      } else {
        console.log("Good Command");
      }
    }
  );

  //TO HAVE TO STOP ZOOM OTHER WISE IT WILL STOP AT FULL ZOOM. YOU CAN REDUCE THE TIMEOUT
  setTimeout(() => {
    console.log("Stopping Zoom");
    siYiCameraClient.send(
      mySiyiCamera.stop_zoom(),
      SIYI_CAMERA_UDP_PORT,
      SIYI_CAMERA_UDP_IP_ADDERSS,
      (err) => {
        if (err) {
          console.error("Error sending data:", err);
        } else {
          console.log("Good Command");
          //print to console that zoom is complete
          console.log("Zoom complete");
          console.log("Final Zoom: " + finalPercentZoom + "%");
          console.log("");
        }
      }
    );
  }, amtPositionChange);

  zoomPosition += amtPositionChange;    //update position based on change in zoom
  var finalPercentZoom = zoomPosition / 20;

  siYiCameraClient.on("message", (response) => {
    console.log(`Received response from Camera: ${response.toString("hex")}`);
  });
}

//**Zoom OUT case**--------------------------------------------------------------------------------------------
else if(desiredZoomPosition < zoomPosition){

  //See current position and calculate how much further needed to zoom out
  if(zoomPosition == 0) {    //if already at max zoom out(0)
    amtPositionChange = 0;
  } else if(zoomPosition == 2000){   //if already at max zoom in(2000)
    amtPositionChange = 2000 - desiredZoomPosition;
  } else{   //if in between max zoom out and zoom in
    amtPositionChange = zoomPosition - desiredZoomPosition;
  }

  console.log("Starting Zooming Out");
  siYiCameraClient.send(
    mySiyiCamera.zoom_out(),
    SIYI_CAMERA_UDP_PORT,
    SIYI_CAMERA_UDP_IP_ADDERSS,
    (err) => {
      if (err) {
        console.error("Error sending data:", err);
      } else {
        console.log("Good Command");
      }
    }
  );

  //TO HAVE TO STOP ZOOM OTHER WISE IT WILL STOP AT FULL ZOOM. YOU CAN REDUCE THE TIMEOUT
  setTimeout(() => {
    console.log("Stopping Zoom-out");
    siYiCameraClient.send(
      mySiyiCamera.stop_zoom(),
      SIYI_CAMERA_UDP_PORT,
      SIYI_CAMERA_UDP_IP_ADDERSS,
      (err) => {
        if (err) {
          console.error("Error sending data:", err);
        } else {
          console.log("Good Command");
          //print to console that zoom is complete
          console.log("Zoom complete");
          console.log("Final Zoom: " + finalPercentZoom + "%");
          console.log("");
        }
      }
    );
  }, amtPositionChange);
  siYiCameraClient.on("message", (response) => {
    console.log(`Received response from Camera: ${response.toString("hex")}`);
  });

  zoomPosition -= amtPositionChange;  //update position based on change in zoom
  var finalPercentZoom = zoomPosition / 20;
}

//**No Zoom Case**--------------------------------------------------------------------------
else{
  console.log("No Zoom needed. \nFinal Zoom: " + guiPercentZoom + "%");
}
