import requests
import json

# Replace with your server address if running remotely
URL = "http://127.0.0.1:5000/receive"

data = {
    "x": 100,
    "y": 200,
    "angle": 45
}

# Send POST request
response = requests.post(URL, json=data)

# Print response
print("Response:", response.json())
