# from picamera2 import Picamera2
# import shutil #new
# import os # new
# import subprocess

# picam2 = Picamera2()
# picam2.start_and_capture_file("image.jpg")


# #new stuff
# yolov5_folder = "./yolov5/"
# destination_file = os.path.join(yolov5_folder, "image.jpg")

# if os.path.exists(destination_file):
#     os.remove(destination_file)

# shutil.move ("image.jpg",yolov5_folder)

# command = ["python", "detect.py", "--source", "image.jpg", "--weights", "balls5n.pt"]
# result = subprocess.run(command, cwd=yolov5_folder)

# if result.returncode == 0:
#     print("Detection script ran successfully.")
#     print("Output:")
#     print(result.stdout)
# else:
#     print("Error running detection script.")
#     print("Error message:")
#     print(result.stderr)


from picamera2 import Picamera2
import shutil #new
import os # new
import subprocess

picam2 = Picamera2()
picam2.start_and_capture_file("image.jpg", show_preview=False)


#new stuff
yolov5_folder = "./yolov5/"
destination_file = os.path.join(yolov5_folder, "image.jpg")

print(destination_file)
print(os.getcwd())

if os.path.exists(destination_file):
    print("removed file")
    os.remove(destination_file)


try:
    
    shutil.move ("image.jpg",yolov5_folder)

except:
    print("Failed moving file")
    
try:
    
    command = ["python", "detect.py", "--source", "image.jpg", "--weights", "balls5n.pt"]
    result = subprocess.run(command, cwd=yolov5_folder)

except:
    print("problem running the command")

if result.returncode == 0:
    print("Detection script ran successfully.")
    print("Output:")
    print(result.stdout)
else:
    print("Error running detection script.")
    print("Error message:")
    print(result.stderr)
