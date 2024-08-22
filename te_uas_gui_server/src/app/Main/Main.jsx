/* Styling */
import './Main.css';
import './Components/Modules.css';

/* Default */
import PropTypes from 'prop-types';

// Components
import Camera from './Components/Camera';
import BatteryIndicator from './Components/Battery';
import DroneModule from './Components/DroneModule';
import HangarModule from './Components/HangarModule';
import QuickControlModule from './Components/QCModule';

function Main({
  ip,
  serverIp,
  rosConnected,
  droneConnected,
  gAltitude,
  altitude,
  desAltitude,
  velocity,
  desVelocity,
  armed,
  mode,
  battery,
  tetherUnspooled,
  unspoolVelocity,
  hangarState,
}) {
  return (
    <div className="Main">
      <div className="main-container">
        <div className="main-body">
          <div className="default-module">
            <Camera ip={ip} />
            <div />
            <BatteryIndicator charge={battery} tethered />
          </div>
          <div className="module1">
            <QuickControlModule
              ip={ip}
              serverIp={serverIp}
              rosConnected={rosConnected}
              armed={armed}
              mode={mode}
            />
          </div>
          <div className="module2">
            <DroneModule
              droneConnected={droneConnected}
              currentGAltitude={gAltitude}
              currentAltitude={altitude}
              targetAltitude={desAltitude}
              currentVelocity={velocity}
              targetVelocity={desVelocity}
              armed={armed}
              mode={mode}
              battery={battery}
            />
          </div>
          <div className="module3">
            <HangarModule
              rosConnected={rosConnected}
              tetherUnspooled={tetherUnspooled}
              unspoolVelocity={unspoolVelocity}
              hangarState={hangarState}
            />
          </div>
        </div>
      </div>
    </div>
  );
}
Main.defaultProps = {
  desVelocity: '3',
};
Main.propTypes = {
  ip: PropTypes.string.isRequired,
  serverIp: PropTypes.string.isRequired,
  rosConnected: PropTypes.bool.isRequired,
  droneConnected: PropTypes.bool.isRequired,
  altitude: PropTypes.string.isRequired,
  gAltitude: PropTypes.string.isRequired,
  desAltitude: PropTypes.string.isRequired,
  velocity: PropTypes.string.isRequired,
  desVelocity: PropTypes.string,
  armed: PropTypes.bool.isRequired,
  mode: PropTypes.string.isRequired,
  battery: PropTypes.string.isRequired,
  tetherUnspooled: PropTypes.string.isRequired,
  unspoolVelocity: PropTypes.string.isRequired,
  hangarState: PropTypes.bool.isRequired,
};

export default Main;
