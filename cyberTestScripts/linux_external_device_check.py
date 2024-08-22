import subprocess
import time

def check_external_devices():
    # Run lsblk command to list block devices
    output = subprocess.check_output(["lsblk", "-o", "NAME,TYPE,RM"])
    lines = output.decode().split('\n')
    
    # Check if any external devices are connected
    for line in lines[1:]:
        if line.strip():  # Skip empty lines
            parts = line.split()
            device_name = parts[0]
            device_type = parts[1]
            removable = int(parts[2])
            if device_type == 'disk' and removable == 1:
                return True
    return False

def alert_user():
    print("External device detected!")

def main():
    while True:
        if check_external_devices():
            alert_user()
        time.sleep(1)  # Check every second

if __name__ == "__main__":
    main()
