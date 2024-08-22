# Getting Started with the TE-UAS-GUI


## Available Scripts

In the te_uas_desktop or server directory, you can run:

### `npm start`

This will run the app in development mode, [http://localhost:3000](http://localhost:3000) to view  in your browser.

### Manually starting the servers
For testing, start the servers: `npm start`
* Run auth-server: `node auth-server.js`
* Run server: `node database-server.js`

## Simulation Setup
`make px4_sitl px4.launch`

Run mavros: `ros2 launch te_uas simulation.launch` 
* `ros2 launch mavros px4.launch`
* `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
* `ros2 run te_uas logger`
* `ros2 run te_uas killswitch`

Run flight script: `ros2 launch drone_control.launch`
