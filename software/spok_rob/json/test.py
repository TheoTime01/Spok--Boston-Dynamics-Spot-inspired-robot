import json
import os


print(os.path)

with open('/json/forward.json', 'r') as file:
    data = json.load(file)


print(data["sequence"])