import os
import shutil

# Directory containing the audio files
source_dir = "sdcard/"
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