import os
import shutil
import sys

# Check if a directory number is provided
if len(sys.argv) != 2:
    print("Error: No directory number provided.")
    sys.exit(1)

# Get the directory number from the command line arguments
dir_number = sys.argv[1]

# Validate the directory number
if dir_number not in ['201', '202', '203', '204', '205']:
    print("Error: Invalid directory number. Please provide a number between 201 and 205.")
    sys.exit(1)

# Directory containing the audio files
source_dir = f"{dir_number}/"
# Mount point of the SD card (modify as needed)
destination_dir = "E:/"

# Get the list of files
files = os.listdir(source_dir)

# Sort the files by name
files.sort()

# Copy the files in the sorted order
for file in files:
    # Path of the source file
    src_file = os.path.join(source_dir, file)
    # Path of the destination file
    dst_file = os.path.join(destination_dir, file)
    # Copy the file
    shutil.copy(src_file, dst_file)

print("File copy completed.")
