#include <Spresense.h>
#include <UART.h>
#include <cmath>
#include <vector>
#include <fstream>

// Map Parameters
const int MAP_SIZE = 500;               // 500x500 grid
const float MAP_RESOLUTION = 0.05;      // 5 cm per cell
const int MAP_CENTER = MAP_SIZE / 2;
const int OCCUPIED = 100;
const int FREE = 0;
const int UNKNOWN = -1;

// LiDAR Parameters
const int LIDAR_BAUD_RATE = 128000;
const int LIDAR_RANGE_MAX = 10.0;       // Maximum range in meters

// Global Variables
int occupancyGrid[MAP_SIZE][MAP_SIZE];  // Occupancy grid map

// Helper Function: Initialize the Map
void initializeMap() {
    for (int i = 0; i < MAP_SIZE; ++i) {
        for (int j = 0; j < MAP_SIZE; ++j) {
            occupancyGrid[i][j] = UNKNOWN;
        }
    }
}

// Helper Function: Convert World to Grid Coordinates
std::pair<int, int> worldToGrid(float x, float y) {
    int gx = static_cast<int>(x / MAP_RESOLUTION) + MAP_CENTER;
    int gy = static_cast<int>(y / MAP_RESOLUTION) + MAP_CENTER;
    return {gx, gy};
}

// Helper Function: Update Map
void updateMap(float robotX, float robotY, float robotTheta, const std::vector<std::pair<float, float>>& lidarPoints) {
    for (const auto& point : lidarPoints) {
        // Transform LiDAR point to world coordinates
        float wx = point.first * cos(robotTheta) - point.second * sin(robotTheta) + robotX;
        float wy = point.first * sin(robotTheta) + point.second * cos(robotTheta) + robotY;

        // Convert world coordinates to grid coordinates
        auto [gx, gy] = worldToGrid(wx, wy);

        if (gx >= 0 && gx < MAP_SIZE && gy >= 0 && gy < MAP_SIZE) {
            // Mark as occupied
            occupancyGrid[gx][gy] = OCCUPIED;
        }
    }
}

// Helper Function: Save Map to File
void saveMapToFile(const char* filename) {
    std::ofstream file(filename);
    for (int i = 0; i < MAP_SIZE; ++i) {
        for (int j = 0; j < MAP_SIZE; ++j) {
            file << (occupancyGrid[i][j] == OCCUPIED ? "#" : 
                     occupancyGrid[i][j] == FREE ? "." : " ");
        }
        file << "\n";
    }
    file.close();
}

// Read LiDAR Data from UART
std::vector<std::pair<float, float>> readLidarData(Uart& lidarUart) {
    std::vector<std::pair<float, float>> points;
    while (lidarUart.available()) {
        float distance = lidarUart.readFloat();
        float angle = lidarUart.readFloat();
        if (distance > 0 && distance < LIDAR_RANGE_MAX) {
            float x = distance * cos(angle);
            float y = distance * sin(angle);
            points.emplace_back(x, y);
        }
    }
    return points;
}

// Main Program
int main() {
    initializeMap();

    // Initialize UART for LiDAR
    Uart lidarUart(UART2);
    lidarUart.begin(LIDAR_BAUD_RATE);

    // Robot Pose (static for this example)
    float robotX = 0.0, robotY = 0.0, robotTheta = 0.0;

    while (true) {
        // Read LiDAR Data
        auto lidarPoints = readLidarData(lidarUart);

        // Update Map
        updateMap(robotX, robotY, robotTheta, lidarPoints);

        // Save Map for Visualization
        saveMapToFile("/mnt/sd0/map.txt");

        // Add a delay to prevent overloading
        delay(500);
    }
    return 0;
}
