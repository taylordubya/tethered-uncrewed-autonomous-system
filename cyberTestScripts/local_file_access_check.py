import os.path
import sys
import time

def check_last_access_time(file_path):
    # Check if the file exists
    if os.path.exists(file_path):
        # Get the last access time of the file
        last_access_time = os.path.getatime(file_path)
        
        # Convert the last access time to a human-readable format
        last_access_time_readable = time.ctime(last_access_time)
        
        print("Last accessed:", last_access_time_readable)
    else:
        print("File does not exist.")

if __name__ == "__main__":
    # Check if the correct number of arguments is provided
    if len(sys.argv) != 2:
        print("Usage: python script_name.py file_path")
    else:
        file_path = sys.argv[1]
        check_last_access_time(file_path)
