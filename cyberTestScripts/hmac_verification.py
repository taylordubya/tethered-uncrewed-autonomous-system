import os
import json
import hmac
import hashlib
from Crypto.Cipher import AES # type: ignore
from Crypto.Random import get_random_bytes # type: ignore
from Crypto.Protocol.KDF import scrypt # type: ignore

def generate_hmac(key, message):
    """
    Generate HMAC tag for a given message using a key.
    
    Args:
        key (bytes): The key used for HMAC.
        message (bytes): The message to generate HMAC tag for.
    
    Returns:
        bytes: HMAC tag.
    """
    h = hmac.new(key, message, hashlib.sha256)
    return h.digest()

def get_json_data(json_filename: str):
    # Read JSON file
    with open(json_filename, 'r') as f:
        data = json.load(f)
    
    # Convert hex strings to bytes
    ciphertext = bytes.fromhex(data['ciphertext'])
    iv = bytes.fromhex(data['iv'])
    salt = bytes.fromhex(data['salt'])
    hmac_tag = bytes.fromhex(data['hmac_tag'])

    return ciphertext, iv, salt, hmac_tag

def derive_key(password: str, salt: str, key_len=32, N=2**14, r=8, p=1):
    key = scrypt(password, salt, key_len, N, r, p)
    return key

def pad(plaintext, block_size=16):
    # Pad the plaintext to a multiple of 16 bytes
    remainder = len(plaintext) % block_size
    if remainder != 0:
        padding_length = block_size - remainder
        plaintext += bytes([padding_length]) * padding_length

    return plaintext

def decrypt(key, iv, ciphertext):
    # Decrypt the ciphertext
    cipher = AES.new(key, AES.MODE_CBC, iv)
    decrypted_data = cipher.decrypt(ciphertext)
    
    # Remove padding
    padding_length = decrypted_data[-1]
    decrypted_data = decrypted_data[:-padding_length]

    return decrypted_data

def encrypt_file(filename, password):
    """
    Encrypts a file using AES 256 with a password and stores the encrypted data along with
    IV, salt, and HMAC tag in a JSON file.
    
    Args:
        filename (str): Name of the file to be encrypted.
        password (str): Password provided by the user.
    """
    # Read the file content
    with open(filename, 'rb') as f:
        plaintext = f.read()
    
    # Pad the text
    padded_text = pad(plaintext=plaintext)

    # Generate salt and derive key
    salt = get_random_bytes(16)
    key = derive_key(password=password, salt=salt)
    
    # Generate initialization vector (IV)
    iv = get_random_bytes(16)
    
    # Encrypt the padded plaintext
    cipher = AES.new(key, AES.MODE_CBC, iv)
    ciphertext = cipher.encrypt(padded_text)
    
    # Generate HMAC tag
    hmac_tag = generate_hmac(key, ciphertext)
    
    # Write encrypted data, IV, salt, and HMAC tag to JSON file
    with open(filename + '.json', 'w') as f:
        json.dump({
            'ciphertext': ciphertext.hex(),
            'iv': iv.hex(),
            'salt': salt.hex(),
            'hmac_tag': hmac_tag.hex()
        }, f)

    # Delete the original file
    os.remove(filename)

def decrypt_file(json_filename: str, password: str) -> bool:
    """
    Decrypts a JSON file containing encrypted data, IV, salt, and HMAC tag using AES 256
    with a password. Verifies the HMAC tag and then decrypts the data.
    
    Args:
        json_filename (str): Name of the JSON file containing encrypted data.
        password (str): Password provided by the user.
    """
    # Read JSON file for ciphertext, iv, salt, and hmac_tag
    ciphertext, iv, salt, hmac_tag = get_json_data(json_filename=json_filename)

    # Derive key from password and salt
    key = derive_key(password=password, salt=salt)
    
    # Verify HMAC tag
    computed_hmac_tag = generate_hmac(key, ciphertext)
    if not hmac.compare_digest(computed_hmac_tag, hmac_tag):
        print("HMAC tag verification failed. File may have been tampered with.")
        return False
    
    # Decrypt the data using key
    decrypted_data = decrypt(key=key, iv=iv, ciphertext=ciphertext)
    
    # Write decrypted data to a new file
    output_filename = os.path.splitext(json_filename)[0]
    with open(output_filename, 'wb') as f:
        f.write(decrypted_data)

    # Delete the original file
    os.remove(json_filename)

    # Successful decryption
    return True

def process_folder(folder_path, password, action):
    """
    Encrypts or decrypts all files in a folder based on the specified action.
    
    Args:
        folder_path (str): Path to the folder containing the files.
        password (str): Password provided by the user.
        action (str): Action to perform ('encrypt' or 'decrypt').
    """
    for filename in os.listdir(folder_path):
        file_path = os.path.join(folder_path, filename)
        if os.path.isfile(file_path):
            if action == 'encrypt':
                encrypt_file(file_path, password)
                print("Encryption completed for:", filename)
            elif action == 'decrypt':
                if file_path.endswith('.json'):
                    is_successful = decrypt_file(file_path, password)
                    if is_successful:
                        print("Decryption completed for:", filename)
                    else:
                        print("Failed decryption for:", filename)
                else:
                    print("Skipping decryption for non-JSON file:", filename)
        else:
            print("Skipping non-file entry:", filename)

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Encrypt or decrypt files in a folder.')
    parser.add_argument('action', choices=['encrypt', 'decrypt'], help='Action to perform (encrypt/decrypt)')
    parser.add_argument('folder', help='Folder containing files to process')
    parser.add_argument('password', help='Password for encryption/decryption')
    args = parser.parse_args()
    
    process_folder(args.folder, args.password, args.action)
