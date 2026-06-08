# Compiler and Flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall

# Source files (Update with full paths)
SRC = /home/gorka/dsor/catkin_ws_mPlanning/src/thesis_Gorka/motion_planning_cpp/src/motion_planning_cpp_algorithms/Test.cpp \
      /home/gorka/dsor/catkin_ws_mPlanning/src/thesis_Gorka/motion_planning_cpp/src/motion_planning_cpp_algorithms/BezierOptimization.cpp

# Object files
OBJ = $(SRC:.cpp=.o)

# Include directories
INCLUDE_DIRS = -I/home/gorka/dsor/catkin_ws_mPlanning/src/thesis_Gorka/motion_planning_cpp/include \
               -I/home/gorka/casadi/casadi  \
               -I/home/gorka/casadi/build/casadi/core  # Updated path  # Ensure this path is correct

# Linker flags (If necessary, link CasADi libraries or others here)
LDFLAGS = -L/home/gorka/casadi/casadi/lib -lcasadi

# Output executable
TARGET = motion_planning

# Build rules
all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(OBJ) -o $(TARGET) $(LDFLAGS)

# Compile source files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)
