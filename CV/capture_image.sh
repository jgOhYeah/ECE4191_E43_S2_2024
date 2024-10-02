#!/bin/bash

# Directory to save images
SAVE_DIR="training_images"

# Create the directory if it doesn't exist
mkdir -p "$SAVE_DIR"

# Function to get the next available filename
get_next_filename() {
    local index=1
    while true; do
        filename=$(printf "image_%04d.jpg" $index)
        if [ ! -f "$SAVE_DIR/$filename" ]; then
            echo "$filename"
            return
        fi
        ((index++))
    done
}

# Check if ImageMagick is installed
if ! command -v convert &> /dev/null
then
    echo "ImageMagick is not installed. Please install it using:"
    echo "sudo apt-get update && sudo apt-get install imagemagick"
    exit 1
fi

echo "Starting image capture. Press Ctrl+C to stop."

# Main capture loop
while true; do
    # Get the next filename
    filename=$(get_next_filename)
    
    # Capture the image using libcamera-still at 1920x1080
    if ! libcamera-still --rotation 180 --width 1920 --height 1080 -o "$SAVE_DIR/temp_$filename" --nopreview; then
        echo "Error: Failed to capture image. Retrying in 5 seconds..."
        sleep 5
        continue
    fi
    
    # Check if the temporary file was created
    if [ ! -f "$SAVE_DIR/temp_$filename" ]; then
        echo "Error: Temporary file not created. Retrying in 5 seconds..."
        sleep 5
        continue
    fi
    
    # Resize the image to fit within 640x480 while maintaining aspect ratio,
    # then pad to exactly 640x480 if necessary
    if ! convert "$SAVE_DIR/temp_$filename" -resize 640x480 -background black -gravity center -extent 640x480 "$SAVE_DIR/$filename"; then
        echo "Error: Failed to resize image. Skipping this image."
        rm -f "$SAVE_DIR/temp_$filename"
        sleep 5
        continue
    fi
    
    # Remove the temporary high-res image
    rm -f "$SAVE_DIR/temp_$filename"
    
    echo "Successfully captured and resized $filename"
    
    # Wait for 5 seconds
    sleep 0.2
done