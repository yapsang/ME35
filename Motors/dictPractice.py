# Reading values from a dictionary
student = {
    "name": "Alex",
    "score": 92,
    "passed": True
}

print(student["name"])
print(student["score"])
print(student["passed"])

print("*** Reading a nested dictionary ***")

robot = {
    "name": "Rover",
    "position": {
        "x": 12,
        "y": 7
    }
}

print(robot["name"])
print(robot["position"]["x"])
print(robot["position"]["y"])

print("*** Reading a list inside a dictionary ***")
robot = {
    "name": "Rover",
    "sensors": ["distance", "temperature", "light"]
}

print(robot["sensors"][0])  # distance
print(robot["sensors"][2])  # light