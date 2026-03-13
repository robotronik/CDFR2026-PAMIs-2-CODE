#!/bin/bash

# Default ESP-IDF installation path. Update this if you installed it elsewhere.
ESP_IDF_EXPORT_PATH="/opt/esp-idf/export.sh"
PORT="/dev/ttyUSB0"

# Function to check for idf.py and source the environment if needed
setup_idf() {
    if ! command -v idf.py &> /dev/null; then
        echo "idf.py not found in current environment."
        if [ -f "$ESP_IDF_EXPORT_PATH" ]; then
            echo "Sourcing ESP-IDF environment..."
            source "$ESP_IDF_EXPORT_PATH"
        else
            echo "Error: Could not find export.sh at $ESP_IDF_EXPORT_PATH"
            echo "Please verify your ESP-IDF installation path."
            exit 1
        fi
    fi
}

# Function to build the project
build_project() {
    echo "Starting build process..."
    idf.py build
}

# Function to clean the build directory
clean_project() {
    echo "Cleaning the build directory..."
    idf.py fullclean 
}

# Function to deploy (flash) the firmware
deploy_project() {
    echo "Deploying firmware to $PORT..."
    idf.py -p "$PORT" flash monitor
}

# Function to reset (erase) the ESP firmware
reset_firmware() { 
    echo "Erasing entire flash memory on $PORT..."
    idf.py -p "$PORT" erase-flash
}

# Function to display usage instructions
print_usage() {
    echo "Usage: $0 {build|clean|deploy|reset}"
    echo "  build   - Compile the ESP32 project"
    echo "  clean   - Remove the build directory contents"
    echo "  deploy  - Flash the firmware to $PORT"
    echo "  reset   - Erase the entire flash memory on $PORT"
    eco  "  setup_lsp - Get compile_commands.json to configure your LSP"
    exit 1
}


# Function to setup LSP (symlink compile_commands.json)
setup_lsp() {
    idf.py reconfigure

    if [ -f "build/compile_commands.json" ]; then
        echo "Symlinking build/compile_commands.json to project root..."
        ln -sf build/compile_commands.json .
        echo "LSP setup complete."
    else
        echo "Error: Failed to generate compile_commands.json."
        exit 1
    fi
}

setup_idf

case "$1" in
    build)
        build_project
        ;;
    clean)
        clean_project
        ;;
    deploy)
        deploy_project
        ;;
    reset)
        reset_firmware
        ;;
    setup_lsp)
        setup_lsp
        ;;
    *) 
        print_usage
        ;;
esac
