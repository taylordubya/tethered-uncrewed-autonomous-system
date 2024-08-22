from Crypto.Protocol.KDF import PBKDF2
from Crypto.Hash import SHA256
import base64
import sys

def generate_key(password="password"):
    # Using PBKDF2 for key generation
    salt = b'h5kx93p2'  # if this changes, change it in multi_factor.py as well
    key = PBKDF2(password, salt, dkLen=32, count=100000, hmac_hash_module=SHA256)
    encoded_key = base64.b64encode(key).decode('utf-8')
    with open("correct_key.txt", "w") as file:
        file.write(encoded_key)
    return encoded_key

def main():
    if len(sys.argv) != 2:
        password = "password"
    else:
        password = sys.argv[1]
    generate_key(password)

if __name__ == "__main__":
    main()