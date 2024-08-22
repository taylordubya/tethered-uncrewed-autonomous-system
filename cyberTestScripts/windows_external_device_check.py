import psutil # type: ignore
import time

def check_external_devices():
    # Get list of connected USB devices
    usb_devices = psutil.disk_partitions(all=True)
    
    # Check if any external devices are connected
    external_devices_connected = any(device.opts == 'rw,removable' for device in usb_devices)
    
    return external_devices_connected

def alert_user():
    print("External device detected!")

def main():
    while True:
        if check_external_devices():
            alert_user()
        time.sleep(1)  # Check every second

if __name__ == "__main__":
    main()
