"""
Start Date: 2024-05-14
Author: Caleb Cope
Purpose: To test AES encryption and decryption functions

References: 
    AES 256 Tutorial: https://medium.com/bootdotdev/aes-256-cipher-python-cryptography-examples-b877b9d2e45e

Installs:
    pip install pycryptodome

5/17/24 Note:
    Added encryption/decryption for entire folders. Added encrypt_file, decrypt_file,
        encrypt_folder, and decrypt_folder functions
"""

import base64
import hashlib
import os
from Crypto.Cipher import AES # type: ignore
from Crypto.Random import get_random_bytes # type: ignore

import sys # to accept arguments (for file name)
import time # for recording encryption/decryption time

# to maintain cipher_text's integrity
import hmac
import json

# salt = b'0orJw7t3DsRfR78UkI+YFA'
# iv = b'5j2qCL2veC67qdjw'
hmac_key = b'SecretKeyForHMAC'

# This function pads the input string to be a multiple of 16 bytes
def pad(s):
    # AES uses 16 byte blocks
    block_size = 16
    remainder = len(s) % block_size
    padding_needed = block_size - remainder
    # print(f'String length: {len(s)}')
    # print(f'Padding needed: {padding_needed}')
    return s + bytes([padding_needed] * padding_needed)

# This function removes the padding from a string
def unpad(s):
    return s[:-s[-1]]

# This function encrypts a string using AES
def encrypt(plain_data, password):
    # "Salt: A new random salt is used for each run of our encryption. This makes
    # it impossible for an attacker to use precomputed hashes in an attempt to 
    # crack the cipher. (see rainbow table)"
    # Generate a random salt
    salt = os.urandom(AES.block_size)

    # "IV: Initialization vector. The initialization vector must be random and new 
    # for each time our encryption function is used. Think of it as a random salt 
    # for a cipher."
    # Generate a random IV
    iv = get_random_bytes(AES.block_size)

    # "Scrypt is used to generate a secure private key from the password. This
    # will make it harder for an attacker to brute-force our encryption. 
    # Scrypt parameters: N is cost factor. It must be a power of 2, and the higher
    # it is, the more secure the key, but the more resources it requres. R is the block size.
    # P is the parallelization factor. It is the number of parallel chains that are computed."
    # Use the Scrypt KDF to get a private key from the password
    private_key = hashlib.scrypt(password.encode(), salt=salt, n=2**14, r=8, p=1, dklen=32)

    # Pad the text for 16 byte blocks
    padded_text = pad(plain_data)

    # Create cipher config
    cipher_config = AES.new(private_key, AES.MODE_CBC, iv)

    encrypted_text = cipher_config.encrypt(padded_text)
    
    # Compute HMAC over the encrypted text
    hmac_tag = hmac.new(hmac_key, encrypted_text, hashlib.sha256).digest()
    
    return {
        'cipher_text': base64.b64encode(encrypted_text),
        'salt': base64.b64encode(salt),
        'iv': base64.b64encode(iv),
        'hmac_tag': base64.b64encode(hmac_tag)
    }

# This function decrypts a string using AES
# def decrypt(enc_dict, password):
def decrypt(enc_bytes, password, iv, salt):
    # Decode the dictionary entries from base64
    # salt = base64.b64decode(enc_dict['salt'])
    # enc = base64.b64decode(enc_dict['cipher_text'])
    # iv = base64.b64decode(enc_dict['iv'])

    # Generate the private key from the password and salt
    private_key = hashlib.scrypt(password.encode(), salt=salt, n=2**14, r=8, p=1, dklen=32)

    # Create the cipher config
    cipher = AES.new(private_key, AES.MODE_CBC, iv)

    # Decrypt the cipher text
    decrypted = cipher.decrypt(enc_bytes)

    # Unpad the tex to remove the padding
    original = unpad(decrypted)

    return original

def encrypt_with_hmac(plain_data, password):
    # Encrypt the plaintext with AES
    encrypted_data = encrypt(plain_data, password)

    # Convert binary data to base64-encoded strings
    cipher_text = base64.b64encode(encrypted_data['cipher_text']).decode('utf-8')
    salt = base64.b64encode(encrypted_data['salt']).decode('utf-8')
    iv = base64.b64encode(encrypted_data['iv']).decode('utf-8')
    hmac_tag = base64.b64encode(encrypted_data['hmac_tag']).decode('utf-8')

    return {
        'cipher_text': cipher_text,
        'salt': salt,
        'iv': iv,
        'hmac_tag': hmac_tag
    }

def decrypt_with_hmac(enc_data, password):
    # Deserialize encrypted data
    encrypted_data = json.loads(enc_data)

    # Decode base64-encoded strings back to binary data
    cipher_text = base64.b64decode(encrypted_data['cipher_text'])
    salt = base64.b64decode(encrypted_data['salt'])
    iv = base64.b64decode(encrypted_data['iv'])
    hmac_tag = base64.b64decode(encrypted_data['hmac_tag'])

    # Compute HMAC over the ciphertext
    print(f"Computing HMAC tag using \n\tcipher_text: {cipher_text}, \n\tsalt: {salt}, \n\tiv: {iv}, \n\thmac_tag: {hmac_tag}")
    computed_hmac_tag = hmac.new(hmac_key, cipher_text, hashlib.sha256).digest()

    # Verify HMAC tags
    print("Computed HMAC tag:", computed_hmac_tag)
    print("Stored HMAC tag:", hmac_tag)
    if hmac.compare_digest(hmac_tag, computed_hmac_tag):
        # HMAC tags match, proceed with decryption
        print("HMAC calculated successfully!\n")
        decrypted = decrypt(base64.b64decode(cipher_text), base64.b64decode(password), base64.b64decode(iv), base64.b64decode(salt))  # Pass IV to the decrypt function
        return unpad(decrypted)
    else:
        # HMAC tags do not match, reject the ciphertext
        raise ValueError("HMAC verification failed.")


def encrypt_file(file_path: str, password: str):
    # file_extension = file_path.split('.')[1]    # get the .extension of the file to maintain file type after encryption
    file_name = file_path.split("/")[-1]
    with open(file_path, 'rb') as file:
        file_bytes = file.read()        # get the raw file bytes
    encrypted_dic = encrypt_with_hmac(file_bytes, password)   # call the encrypt function
    serialized_data = json.dumps(encrypted_dic)
    serialized_data_bytes = serialized_data.encode('utf-8')
    with open(file_name + ".json", "wb") as file:
        file.write(serialized_data_bytes)
    # encrypted_file_name = f'encrypted_log.{file_extension}'     # define hardcoded file name for encryption
    
    # with open(file_path, 'wb') as file:
    #     file.write(base64.b64decode(encrypted_dic['cipher_text']))  # get the cipher_text from the dictionary

    return file_path

def decrypt_file(file_path: str, password: str):
    # file_extension = file_path.split('.')[1]    # get the .extension of the file to maintain file type after decryption
    with open(file_path, 'rb') as file:
        file_bytes = file.read()        # get the raw file bytes
    decrypted = decrypt_with_hmac(file_bytes, password)   # call the decrypt function
    # decrypted_file_name = f'decrypted_log.{file_extension}'     # define hardcoded file name for decryption
    
    with open(file_path, 'wb') as file:
        file.write(decrypted)  # get the plain_text from decryption

    return file_path

def encrypt_folder(folder_path: str, password: str) -> bool:
    for file_name in os.listdir(folder_path):
        file_path = os.path.join(folder_path, file_name)
        if os.path.isfile(file_path):
            print(f"Encrypting {file_name}...")
            encrypt_file(file_path=file_path, password=password)
    return True

def decrypt_folder(folder_path: str, password: str) -> bool:
    for file_name in os.listdir(folder_path):
        file_path = os.path.join(folder_path, file_name)
        if os.path.isfile(file_path) and file_name.endswith(".json"):
            print(f"Decrypting {file_name}...")
            try:
                with open(file_path, 'r') as file:
                    encrypted_data = file.read()
                    print("Encrypted data from JSON file:", encrypted_data)
                decrypted_data = decrypt_file(file_path=file_path, password=password)
                print("Decrypted data:", decrypted_data)
                decrypted_file_path = file_path.replace('.json', '_decrypted.txt')
                with open(decrypted_file_path, 'wb') as decrypted_file:
                    decrypted_file.write(decrypted_data)
                print(f"Decryption successful. Decrypted file saved as {decrypted_file_path}")
            except Exception as e:
                print(f"Failed to decrypt {file_name}: {e}")
    return True

def get_folder_size(folder_path):
    total_size = 0
    for dirpath, dirnames, filenames in os.walk(folder_path):
        for filename in filenames:
            file_path = os.path.join(dirpath, filename)
            total_size += os.path.getsize(file_path)
    return total_size

# Test the encryption and decryption functions
def main():
    folder_path = sys.argv[1]

    # Test the encryption and decryption functions
    # password = 'password'
    choice = input("Would you like to encrypt or decrypt this folder (e/d): ")
    password = input("Password: ")
    #plain_text = 'This is a test string'

    # Encode the plain_text string into bytes
    # plain_text_bytes = plain_text.encode()

    folder_size = get_folder_size(folder_path=folder_path)
    folder_size_kb = folder_size / 1024
    folder_size_mb = folder_size_kb / 1024

    start_time = time.time()

    if choice == "e":
        # Encrypt the folder
        # encrypted_file_name = encrypt_file(log_file_name, log_file_bytes, password)
        # print(f'Encrypted file: {encrypted_file_name}')
        isSuccessful = encrypt_folder(folder_path, password)
        if isSuccessful:
            end_time = time.time()
            elapsed_time = end_time - start_time
            print(f"Folder encryption complete! Time elapsed: {elapsed_time} s")
            print(f"MB per second: {folder_size_mb / elapsed_time}")

    elif choice == "d":
        # Decrypt the folder
        # decrypted_file_name = decrypt_file(log_file_name, log_file_bytes, password)
        # print(f'Decrypted file: {decrypted_file_name}')
        # isSuccessful = decrypt_folder(folder_path, password=password)
        # if isSuccessful:
            # end_time = time.time()
            # elapsed_time = end_time - start_time
            # print(f"Folder decryption complete! Time elapsed: {elapsed_time} s")
            # print(f"MB per second: {folder_size_mb / elapsed_time}")
        # Read the encrypted JSON file
        # with open(folder_path, 'r') as file:
        #     encrypted_data = file.read()
        is_successful = decrypt_folder(folder_path=folder_path, password=password)
        if is_successful:
            end_time = time.time()
            elapsed_time = end_time - start_time
            print(f"Folder decryption complete! Time elapsed: {elapsed_time} s")

    # Print the results
    # print(f'Encrypted text: {encrypted}')
    # print(f'Decrypted text: {decrypted.decode()}')

if __name__ == '__main__':
    main()

