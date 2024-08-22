import os.path
import sys
import time
import hashlib

def calculate_file_hash(file_path):
    # Open the file in binary mode for reading
    with open(file_path, 'rb') as f:
        # Read the contents of the file
        file_content = f.read()
        # Calculate the SHA-256 hash of the file content
        file_hash = hashlib.sha256(file_content).hexdigest()
    return file_hash

def check_file_copy(file_path):
    # Check if the file exists
    if os.path.exists(file_path):
        # Get the hash of the file before accessing it
        initial_hash = calculate_file_hash(file_path)
        # Wait for a short duration (e.g., 1 second)
        time.sleep(15)
        # Get the hash of the file after accessing it
        final_hash = calculate_file_hash(file_path)
        
        # Compare the initial and final hashes
        if initial_hash != final_hash:
            print("File might have been copied.")
        else:
            print("File has not been copied.")
    else:
        print("File does not exist.")

if __name__ == "__main__":
    # Check if the correct number of arguments is provided
    if len(sys.argv) != 2:
        print("Usage: python script_name.py file_path")
    else:
        file_path = sys.argv[1]
        check_file_copy(file_path)
