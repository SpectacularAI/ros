#pragma once

#include <cstdint>
#include <vector>

// Spectacular AI
#include <spectacularAI/vio.hpp>

/**
 * A simple data structure for stroging and generating occupancy grid
 *
 */
struct OccupancyGrid {
    // Row major 2D map. 0,0 is bottom left cell. -1=unknown, 0=free, 100=occupied
    std::vector<int8_t> cells;
    int width;
    int height;
    float cellSize;
    int originX;
    int originY;
    int threshold;
    int maxWidth = 1024;
    int maxHeight = 1024;
    float minTraceDepth;
    float maxTraceDepth;

    bool empty() {
        return cells.empty();
    }

    void reset(const spectacularAI::Vector3f &center, float minDepth, float maxDepth, float cellSizeMeters, int occupiedThreshold) {
        cellSize = cellSizeMeters;
        width = maxDepth / cellSize * 2.0 + 1;
        height = width;
        cells.resize(width * height);
        memset(&cells[0], -1, cells.size() * sizeof(cells[0]));
        originX = std::round((center.x - cellSize * width * .5) / cellSize);
        originY = std::round((center.y - cellSize * height * .5) / cellSize);
        threshold = std::min(127, occupiedThreshold);
        minTraceDepth = minDepth;
        maxTraceDepth = maxDepth;
    }

    int8_t& getCell(int x, int y) {
        return cells[x + y * width];
    }

    // Safely add point to this map
    void addPoint(const spectacularAI::Vector3f &point) {
        int x, y;
        if (!getIdxForWorldPosition(point, x, y)) return; // Outside grid
        int8_t &value = getCell(x, y);
        if (value == 127) return; // Avoid overflow
        if (value == -1) value = 1; // Skip zero, it marks unoccupied
        else value++;
    }

    bool getIdxForWorldPosition(const spectacularAI::Vector3f &point, int &x, int &y) {
        x = int(point.x / cellSize) - originX;
        y = int(point.y / cellSize) - originY;
        if (x < 0 || y < 0 || x >= width || y >= height) return false; // Outside grid
        return true;
    }

    void traceFreeSpace(const spectacularAI::Vector3f &start, const spectacularAI::mapping::Frame &frame, float cameraTestHeight) {
        // Midpoint circle algorithm
        int x0, y0;
        if (!getIdxForWorldPosition(start, x0, y0)) return;
        int radius = int(maxTraceDepth / cellSize) - 1;
        if (radius <= 0) return;
        int f = 1 - radius;
        int ddf_x = 1;
        int ddf_y = -2 * radius;
        int x = 0;
        int y = radius;
        trace(start, frame, cameraTestHeight, x0, y0 + radius);
        trace(start, frame, cameraTestHeight, x0, y0 - radius);
        trace(start, frame, cameraTestHeight, x0 + radius, y0);
        trace(start, frame, cameraTestHeight, x0 - radius, y0);
        while (x < y) {
            if (f >= 0) {
                y -= 1;
                ddf_y += 2;
                f += ddf_y;
            }
            x += 1;
            ddf_x += 2;
            f += ddf_x;
            trace(start, frame, cameraTestHeight, x0 + x, y0 + y);
            trace(start, frame, cameraTestHeight, x0 - x, y0 + y);
            trace(start, frame, cameraTestHeight, x0 + x, y0 - y);
            trace(start, frame, cameraTestHeight, x0 - x, y0 - y);
            trace(start, frame, cameraTestHeight, x0 + y, y0 + x);
            trace(start, frame, cameraTestHeight, x0 - y, y0 + x);
            trace(start, frame, cameraTestHeight, x0 + y, y0 - x);
            trace(start, frame, cameraTestHeight, x0 - y, y0 - x);
        }
    }

    void trace(const spectacularAI::Vector3f &start, const spectacularAI::mapping::Frame &frame, float cameraTestHeight, int targetX, int targetY) {
        spectacularAI::Vector3f target = {
            (originX + targetX + 0.5f) * cellSize,
            (originY + targetY + 0.5f) * cellSize,
            cameraTestHeight
        };

        // Check if target is outside camera view. Remove % of the view cone to avoid errors in depth map and such at the edges
        spectacularAI::PixelCoordinates tempPixel;
        constexpr float SAFETY_MARGIN = 0.05;
        float upperX = frame.image->getWidth() * (1.0f - SAFETY_MARGIN);
        float lowerX = frame.image->getWidth() * SAFETY_MARGIN;
        float upperY = frame.image->getWidth() * (1.0f - SAFETY_MARGIN);
        float lowerY = frame.image->getWidth() * SAFETY_MARGIN;
        if (!frame.cameraPose.worldToPixel({target.x, target.y, target.z}, tempPixel)) return;
        if (tempPixel.x < lowerX || tempPixel.y < lowerY || tempPixel.x > upperX || tempPixel.y > upperY) return;
        trace(start, target, targetX, targetY);
    }

    float fract(float f) {
        return std::fmod(f, 1.0f);
    }

    // Trace ray and mark unoccupied cells
    void trace(const spectacularAI::Vector3f &start, const spectacularAI::Vector3f &target, int targetX, int targetY) {
        // Fast Voxel Traversal
        spectacularAI::Vector3f dir = {
            target.x - start.x,
            target.y - start.y,
            0.0f
        };
        float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        dir.x /= len;
        dir.y /= len;

        spectacularAI::Vector3f startLocal = {
            start.x - originX * cellSize + dir.x * minTraceDepth,
            start.y - originY * cellSize + dir.y * minTraceDepth,
            0.0f
        };

        int currentX = int(startLocal.x / cellSize);
        int currentY = int(startLocal.y / cellSize);

        int stepX = dir.x > 0 ? 1 : -1;
        int stepY = dir.y > 0 ? 1 : -1;

        float tMaxX, tMaxY;

        if (dir.x == 0) tMaxX = std::numeric_limits<float>::infinity();
        else if (dir.x > 0) tMaxX = (1.0f - fract(startLocal.x / cellSize)) / dir.x;
        else tMaxX = fract(startLocal.x / cellSize) / -dir.x;

        if (dir.y == 0) tMaxY = std::numeric_limits<float>::infinity();
        else if (dir.y > 0) tMaxY = (1.0f - fract(startLocal.y / cellSize)) / dir.y;
        else tMaxY = fract(startLocal.y / cellSize) / -dir.y;

        float tDeltaX = std::abs(1.0 / dir.x);
        float tDeltaY = std::abs(1.0 / dir.y);

        while (currentX != targetX || currentY != targetY) {
            // Shouldn't be necessary, but account for floating point inaccuracies?
            if (currentX < 0 || currentY < 0 || currentX >= width || currentY >= height) return; // Outside grid
            int8_t &cell = getCell(currentX, currentY);
            if (cell >= threshold) return; // Occupied cell, stop tracing
            else cell = 0; // Unoccupied cell, set to 0
            if (tMaxX < tMaxY) {
                tMaxX += tDeltaX;
                currentX += stepX;
            } else {
                tMaxY += tDeltaY;
                currentY += stepY;
            }
        }
    }

    static void bounds(int aX, int aWidth, int bX, int bWidth, int &min, int &max) {
        min = std::min(aX, bX);
        max = std::max(aX + aWidth, bX + bWidth);
    }

    static void overlap(int aX, int aWidth, int bX, int bWidth, int &min, int &max) {
        min = std::max(aX, bX);
        max = std::min(aX + aWidth, bX + bWidth);
    }

    // Copy overlapping area from src to dst
    static void copyCells(const std::vector<int8_t> &src, int srcOriginX, int srcOriginY, int srcWidth, int srcHeight,
        std::vector<int8_t> &dst, int dstOriginX, int dstOriginY, int dstWidth, int dstHeight) {
        int leftX, rightX, bottomY, topY;
        overlap(srcOriginX, srcWidth, dstOriginX, dstWidth, leftX, rightX);
        overlap(srcOriginY, srcHeight, dstOriginY, dstHeight, bottomY, topY);
        if (leftX >= rightX || bottomY >= topY) return; // No overlap
        for (int x = leftX; x < rightX; x++) {
            for (int y = bottomY; y < topY; y++) {
                int cell = src[(x - srcOriginX) + (y - srcOriginY) * srcWidth];
                if (cell != -1) { // Don't copy "unknown" values, we don't want to override existing information for those
                    dst[(x - dstOriginX) + (y - dstOriginY) * dstWidth] = cell == 0 ? 0 : 100;
                }
            }
        }
    }

    // Merge other map into to this, possibly resizing and moving this map
    void merge(const OccupancyGrid &other) {
        int leftX, rightX, bottomY, topY;
        bounds(originX, width, other.originX, other.width, leftX, rightX);
        int newWidth = rightX - leftX;
        if (newWidth > maxWidth) {
            if (leftX != other.originX) {
                leftX = rightX - maxWidth;
            }
            newWidth = maxWidth;
        }
        bounds(originY, height, other.originY, other.height, bottomY, topY);
        int newHeight = topY - bottomY;
        if (newHeight > maxHeight) {
            if (bottomY != other.originY) {
                bottomY = topY - maxHeight;
            }
            newHeight = maxHeight;
        }

        // Other map fits entirely into this one
        if (originX == leftX && originY == bottomY && width == newWidth && height == newHeight) {
            copyCells(
                other.cells, other.originX, other.originY, other.width, other.height,
                cells, originX, originY, width, height
            );
        } else { // Otherwise create a new map
            std::vector<int8_t> newCells;
            newCells.resize(newWidth * newHeight, -1);
            copyCells(
                cells, originX, originY, width, height,
                newCells, leftX, bottomY, newWidth, newHeight
            );
            copyCells(
                other.cells, other.originX, other.originY, other.width, other.height,
                newCells, leftX, bottomY, newWidth, newHeight
            );

            cells = newCells;
            originX = leftX;
            originY = bottomY;
            width = newWidth;
            height = newHeight;
        }
    }
};
