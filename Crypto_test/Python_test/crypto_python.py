# import sys
# import hashlib
# if sys.version_info < (3, 6):
#     import sha3
 
# str = "A"
 
# # create a sha3 hash object
# hash_sha3_512 = hashlib.new("sha3_512", str.encode())
# print("\nSHA3-512 Hash: ", hash_sha3_512.hexdigest())

# from Crypto.Cipher import AES

# # Encryption
# key = b'Sixteen byte key'
# iv = b'Sixteen byte IV'
# cipher = AES.new(key, AES.MODE_CBC, iv)
# data = b'This is the data to be encrypted'
# ciphertext = cipher.encrypt(data)

# # Decryption
# cipher = AES.new(key, AES.MODE_CBC, iv)
# plaintext = cipher.decrypt(ciphertext)

from cryptography.fernet import Fernet

# Generate a random encryption key
key = Fernet.generate_key()

# Initialize the Fernet symmetric encryption object
fernet = Fernet(key)
print(type(key))
# Data to be encrypted
message = "Hello, this is a secret message!"

# Encrypt the data
encrypted_message = fernet.encrypt(message.encode())

# Decrypt the data
decrypted_message = fernet.decrypt(encrypted_message)

print("Original Message:", message)
print("Encrypted Message:", encrypted_message)
print("Decrypted Message:", decrypted_message.decode())
