from Crypto.Protocol.KDF import PBKDF2
from Crypto.Hash import SHA256
import base64

# for mfa:
import pyotp # type: ignore
import os

def generate_key(password):
    # Using PBKDF2 for key generation
    salt = b'h5kx93p2'  # if this changes, change it in generate_password_key.py as well
    key = PBKDF2(password, salt, dkLen=32, count=100000, hmac_hash_module=SHA256)
    encoded_key = base64.b64encode(key).decode('utf-8')
    return encoded_key

def check_password(password, stored_key):
    # Generating the key for the entered password
    entered_key = generate_key(password)
    return entered_key == stored_key

def read_stored_key_from_file(file_path):
    with open(file_path, 'r') as file:
        stored_key = file.read().strip()
    return stored_key

def verify_password(bad_input_counter):
    if bad_input_counter > 5:
        print(f"****** TOO MANY FAILED ATTEMPTS. SHUTTING DOWN ******")
        exit(1)
    
    stored_key = read_stored_key_from_file("correct_key.txt")
    entered_password = input("Enter your password: ")
    
    if check_password(entered_password, stored_key):
        print("Password verified. Access granted.")
        return True, 0
    else:
        print("Incorrect password. Access denied.")
        bad_input_counter += 1
        return False, bad_input_counter

def verify_otp(bad_input_counter):
    if bad_input_counter > 4:
        print(f"****** TOO MANY FAILED ATTEMPTS. SHUTTING DOWN ******")
        exit(1)
    
    # pyotp_key = "py3773otpthing7725twenty"
    pyotp_key = os.environ.get('PYOTP_SECRET_KEY')
    if pyotp_key is None:
        print("Error: PyOTP secret key is not set as an environment variable.")
        exit(1)

    # print(f"Found a key: {pyotp_key}")
    totp = pyotp.TOTP(pyotp_key)
    
    # *****
    # uri = pyotp.totp.TOTP(pyotp_key).provisioning_uri(name="Kevin",
    #                                                   issuer_name="multi_factor example")
    # qrcode.make(uri).save("totp.png")
    # *****

    # print(totp.now())
    input_code = input("Enter 2FA Code: ")
    verification_status = totp.verify(input_code)
    if verification_status:
        print("OTP verified. Access granted.")
        return True, 0
    else:
        print("Incorrect OTP. Access denied.")
        bad_input_counter += 1
        return False, bad_input_counter

    return totp.verify(input_code)

def main():
    bad_input_counter = 0
    
    is_valid_input, bad_input_counter = verify_password(bad_input_counter)
    while not is_valid_input:
        print(f"Try again.\n")
        is_valid_input, bad_input_counter = verify_password(bad_input_counter)
    
    is_valid_input, bad_input_counter = verify_otp(bad_input_counter)
    while not is_valid_input:
        print(f"Try again.\n")
        is_valid_input, bad_input_counter = verify_otp(bad_input_counter)
    print(f"****** SUCCESSFUL LOGIN ******")
    print(f"Exiting now...")

if __name__ == "__main__":
    main()
