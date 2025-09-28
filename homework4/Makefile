# Define the compiler and flags
CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++11

# Define the build directory
BUILD_DIR = build

# Define the names of the source files
SOURCES = main.cpp graph.cpp

# Define the object files with the build directory path
# (e.g., build/main.o build/graph.o)
OBJECTS = $(patsubst %.cpp, $(BUILD_DIR)/%.o, $(SOURCES))

# NEW: Define the executable name
EXECUTABLE = dijkstra_program

# --- Targets ---

# Default target
all: $(BUILD_DIR) $(EXECUTABLE)

# Rule to create the build directory if it doesn't exist
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# Rule to link the object files into the executable
$(EXECUTABLE): $(OBJECTS)
	$(CXX) $^ -o $@

# Generic rule to compile a .cpp file into a .o file within the build directory
$(BUILD_DIR)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean target to remove generated files and the directory
clean:
	rm -rf $(BUILD_DIR) $(EXECUTABLE)