#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <chrono>
#include <set>
#include <map>
#include <optional>


struct Particle {
    sf::CircleShape outer;
    sf::CircleShape inner;
    sf::Vector2f velocity;
    sf::RectangleShape coverageArea;

    std::set<std::pair<int, int>> coveredGridCells;
    std::vector<std::pair<float, int>> velocityMovementPairs; // speed, movements between collisions
    int movementsSinceLastCollision = 0;
    std::pair<int, int> lastGridPosition = {-1, -1}; // Track last grid position

    Particle(float radius, sf::Vector2f pos, sf::Vector2f vel)
        : velocity(vel) {
        outer.setRadius(radius);
        outer.setFillColor(sf::Color::White);
        outer.setOrigin(sf::Vector2f(radius, radius));
        outer.setPosition(pos);

        float innerRadius = radius * 0.3f;
        inner.setRadius(innerRadius);
        inner.setFillColor(sf::Color::Cyan);
        inner.setOrigin(sf::Vector2f(innerRadius, innerRadius));
        inner.setPosition(pos);

        // Setup coverage area visualization (will be sized based on grid)
        coverageArea.setFillColor(sf::Color(255, 255, 0, 60)); // Semi-transparent yellow
        coverageArea.setOutlineColor(sf::Color(255, 255, 0, 120));
        coverageArea.setOutlineThickness(1.0f);
    }

    void updateCoverageArea(float cellWidth, float cellHeight) {
        // Coverage area spans 3x3 grid cells
        float coverageWidth = 3.0f * cellWidth;
        float coverageHeight = 3.0f * cellHeight;
        
        coverageArea.setSize(sf::Vector2f(coverageWidth, coverageHeight));
        coverageArea.setOrigin(sf::Vector2f(coverageWidth / 2.0f, coverageHeight / 2.0f));
        coverageArea.setPosition(outer.getPosition());
    }

    void move(float dt) {
        sf::Vector2f delta = velocity * dt;
        outer.move(delta);
        inner.move(delta);
        coverageArea.move(delta);
    }

    sf::Vector2f getPosition() const {
        return outer.getPosition();
    }

    float getRadius() const {
        return outer.getRadius();
    }

    void setPosition(sf::Vector2f pos) {
        outer.setPosition(pos);
        inner.setPosition(pos);
        coverageArea.setPosition(pos);
    }

    void draw(sf::RenderWindow& window) const {
        window.draw(coverageArea);
        window.draw(outer);
        window.draw(inner);
    }

    void updateCoverage(int gridRows, int gridCols, float cellWidth, float cellHeight, 
                       std::vector<std::vector<bool>>& globalCoverage) {
        sf::Vector2f pos = outer.getPosition();
        
        // Convert position to grid coordinates
        int centerCol = static_cast<int>(pos.x / cellWidth);
        int centerRow = static_cast<int>(pos.y / cellHeight);
        
        // Check if particle moved to a different grid cell
        std::pair<int, int> currentGridPos = {centerRow, centerCol};
        if (lastGridPosition.first != -1 && currentGridPos != lastGridPosition) {
            movementsSinceLastCollision++;
        }
        lastGridPosition = currentGridPos;
        
        // Cover 3x3 grid cells centered on particle position
        for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
                int row = centerRow + dr;
                int col = centerCol + dc;
                
                // Check bounds
                if (row >= 0 && row < gridRows && col >= 0 && col < gridCols) {
                    coveredGridCells.insert({row, col});
                    globalCoverage[row][col] = true;
                }
            }
        }
    }

    void registerCollision() {
        float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
        velocityMovementPairs.emplace_back(speed, movementsSinceLastCollision);
        movementsSinceLastCollision = 0;
    }
};

class CoverageTracker {
private:
    std::map<float, int> milestones; // percentage -> total movements
    bool milestone50 = false;
    bool milestone60 = false;
    bool milestone70 = false;
    bool milestone80 = false;
    bool milestone90 = false;
    bool milestone95 = false;
    bool milestone99 = false;

public:
    void checkMilestone(float percent, int totalMovements) {
        if (percent >= 50.0f && !milestone50) {
            milestones[50.0f] = totalMovements;
            milestone50 = true;
            std::cout << "50% coverage reached after " << totalMovements << " total movements\n";
        }
        if (percent >= 60.0f && !milestone60) {
            milestones[60.0f] = totalMovements;
            milestone60 = true;
            std::cout << "60% coverage reached after " << totalMovements << " total movements\n";
        }
        if (percent >= 70.0f && !milestone70) {
            milestones[70.0f] = totalMovements;
            milestone70 = true;
            std::cout << "70% coverage reached after " << totalMovements << " total movements\n";
        }  
        if(percent >= 80.0f && !milestone80) {
            milestones[80.0f] = totalMovements;
            milestone80 = true;
            std::cout << "80% coverage reached after " << totalMovements << " total movements\n";
        }
        if (percent >= 90.0f && !milestone90) {
            milestones[90.0f] = totalMovements;
            milestone90 = true;
            std::cout << "90% coverage reached after " << totalMovements << " total movements\n";
        }
        if (percent >= 95.0f && !milestone95) {
            milestones[95.0f] = totalMovements;
            milestone95 = true;
            std::cout << "95% coverage reached after " << totalMovements << " total movements\n";
        }
        if (percent >= 99.0f && !milestone99) {
            milestones[99.0f] = totalMovements;
            milestone99 = true;
            std::cout << "99% coverage reached after " << totalMovements << " total movements\n";
        }
    }

    int getMovementsFor50Percent() const {
        auto it = milestones.find(50.0f);
        return (it != milestones.end()) ? it->second : -1;
    }
    int getMovementsFor60Percent() const {
        auto it = milestones.find(60.0f);
        return (it != milestones.end()) ? it->second : -1;
    }
    int getMovementsFor70Percent() const {
        auto it = milestones.find(70.0f);
        return (it != milestones.end()) ? it->second : -1;
    }
    int getMovementsFor80Percent() const {
        auto it = milestones.find(80.0f);
        return (it != milestones.end()) ? it->second : -1;
    }
    int getMovementsFor90Percent() const {
        auto it = milestones.find(90.0f);
        return (it != milestones.end()) ? it->second : -1;
    }

    int getMovementsFor95Percent() const {
        auto it = milestones.find(95.0f);
        return (it != milestones.end()) ? it->second : -1;
    }

    int getMovementsFor99Percent() const {
        auto it = milestones.find(99.0f);
        return (it != milestones.end()) ? it->second : -1;
    }

    void printSummary() const {
        std::cout << "\n=== Coverage Summary ===\n";
        for (const auto& [percent, movements] : milestones) {
            std::cout << percent << "% coverage: " << movements << " total movements\n";
        }
    }
};

void drawGrid(sf::RenderWindow& window, int rows, int cols, float cellWidth, float cellHeight,
              const std::vector<std::vector<bool>>& coverage) {
    // Draw grid lines
    sf::RectangleShape line;
    line.setFillColor(sf::Color(100, 100, 100, 100)); // Semi-transparent gray
    
    // Vertical lines
    for (int col = 0; col <= cols; ++col) {
        line.setSize(sf::Vector2f(1, rows * cellHeight));
        line.setPosition(sf::Vector2f(col * cellWidth, 0));
        window.draw(line);
    }
    
    // Horizontal lines
    for (int row = 0; row <= rows; ++row) {
        line.setSize(sf::Vector2f(cols * cellWidth, 1));
        line.setPosition(sf::Vector2f(0, row * cellHeight));
        window.draw(line);
    }
    
    // Draw covered cells
    sf::RectangleShape cell;
    cell.setFillColor(sf::Color(50, 150, 50, 80)); // Semi-transparent green
    cell.setSize(sf::Vector2f(cellWidth - 1, cellHeight - 1)); // Slightly smaller to show grid lines
    
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            if (coverage[row][col]) {
                cell.setPosition(sf::Vector2f(col * cellWidth + 0.5f, row * cellHeight + 0.5f));
                window.draw(cell);
            }
        }
    }
}

void handleWallCollision(Particle& p, int width, int height) {
    auto pos = p.getPosition();
    float r = p.getRadius();
    bool collided = false;

    if (pos.x - r < 0) {
        pos.x = r;
        p.velocity.x *= -1;
        collided = true;
    } else if (pos.x + r > width) {
        pos.x = width - r;
        p.velocity.x *= -1;
        collided = true;
    }

    if (pos.y - r < 0) {
        pos.y = r;
        p.velocity.y *= -1;
        collided = true;
    } else if (pos.y + r > height) {
        pos.y = height - r;
        p.velocity.y *= -1;
        collided = true;
    }

    if (collided)
        p.registerCollision();

    p.setPosition(pos);
}

void handleParticleCollision(Particle& p1, Particle& p2, float minDistance) {
    sf::Vector2f diff = p1.getPosition() - p2.getPosition();
    float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);

    if (dist < minDistance && dist > 0) {
        sf::Vector2f normal = diff / dist;
        sf::Vector2f relativeVelocity = p1.velocity - p2.velocity;
        float dot = relativeVelocity.x * normal.x + relativeVelocity.y * normal.y;

        if (dot < 0) {
            float impulse = 2 * dot / 2;
            p1.velocity -= impulse * normal;
            p2.velocity += impulse * normal;

            p1.registerCollision();
            p2.registerCollision();

            // Separate particles to avoid overlap
            float overlap = minDistance - dist;
            sf::Vector2f separation = normal * (overlap / 2.0f);
            p1.setPosition(p1.getPosition() + separation);
            p2.setPosition(p2.getPosition() - separation);
        }
    }
}

// Function to calculate current coverage percentage
float calculateCoveragePercentage(const std::vector<std::vector<bool>>& coverage, int rows, int cols) {
    int coveredCells = 0;
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            if (coverage[row][col]) {
                coveredCells++;
            }
        }
    }
    int totalCells = rows * cols;
    return (static_cast<float>(coveredCells) / totalCells) * 100.0f;
}

int main() {
    int width = 800, height = 600, numParticles = 6;
    int gridRows = 12, gridCols = 12;
    float radius = 16.0f; // Smaller radius to fit better with grid
    float maxSpeed = 160.0f;
    float minDistanceInCells = 1.0f; // Default to 1 grid cell

    std::cout << "Enter number of particles: ";
    std::cin >> numParticles;

    std::cout << "Enter screen width and height (e.g., 800 600): ";
    std::cin >> width >> height;

    std::cout << "Enter grid dimensions (rows cols, default 12 12): ";
    std::cin >> gridRows >> gridCols;

    std::cout << "Enter minimum distance between particles in grid cells (default 1.0): ";
    std::cin >> minDistanceInCells;

    // Calculate cell dimensions
    float cellWidth = static_cast<float>(width) / gridCols;
    float cellHeight = static_cast<float>(height) / gridRows;
    
    // Convert minimum distance from grid cells to pixels
    float avgCellSize = (cellWidth + cellHeight) / 2.0f;
    float minDistance = minDistanceInCells * avgCellSize;

    std::cout << "\nGrid Info:\n";
    std::cout << "Grid size: " << gridRows << "x" << gridCols << " = " << (gridRows * gridCols) << " cells\n";
    std::cout << "Cell size: " << cellWidth << "x" << cellHeight << " pixels\n";
    std::cout << "Minimum distance: " << minDistanceInCells << " grid cells (" << minDistance << " pixels)\n";
    std::cout << "Each mobile node covers 3x3 = 9 cells\n\n";

    sf::RenderWindow window(sf::VideoMode(sf::Vector2u(width, height)), "Grid-Based Coverage Simulation");
    window.setFramerateLimit(60);

    // Initialize global coverage grid
    std::vector<std::vector<bool>> globalCoverage(gridRows, std::vector<bool>(gridCols, false));

    sf::Font font;
    bool fontLoaded = font.openFromFile("Arial.ttf");
    if (!fontLoaded) {
        std::cerr << "Warning: Failed to load font. Text will not display.\n";
    }

    std::srand(std::time(nullptr));
    std::vector<Particle> particles;
    CoverageTracker tracker;

    // Create particles with positions that align better with grid
    for (int i = 0; i < numParticles; ++i) {
        float x = radius + std::rand() % (width - (int)(2 * radius));
        float y = radius + std::rand() % (height - (int)(2 * radius));
        float vx = ((200) / 100.0f - 1) * maxSpeed;
        float vy = ((200) / 100.0f - 1) * maxSpeed;

        particles.emplace_back(radius, sf::Vector2f(x, y), sf::Vector2f(vx, vy));
        particles.back().updateCoverageArea(cellWidth, cellHeight);
    }

    sf::Clock clock;
    const float checkInterval = 1.0f;
    float elapsedSinceLastCheck = 0.0f;
    auto start_time = std::chrono::high_resolution_clock::now();

    std::cout << "Simulation started. Press ESC or close window to exit.\n";
    std::cout << "Coverage milestones will be printed as they are reached.\n\n";

    while (window.isOpen()) {
        // Handle events with new SFML 3.x API
        while (std::optional<sf::Event> event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>() || 
                (event->is<sf::Event::KeyPressed>() && 
                 event->getIf<sf::Event::KeyPressed>()->code == sf::Keyboard::Key::Escape)) {
                window.close();
            }
        }

        float dt = clock.restart().asSeconds();
        elapsedSinceLastCheck += dt;

        // Update particles
        for (auto& p : particles) {
            p.move(dt);
            p.updateCoverageArea(cellWidth, cellHeight);
            p.updateCoverage(gridRows, gridCols, cellWidth, cellHeight, globalCoverage);
        }

        // Handle collisions
        for (size_t i = 0; i < particles.size(); ++i) {
            handleWallCollision(particles[i], width, height);
            for (size_t j = i + 1; j < particles.size(); ++j) {
                handleParticleCollision(particles[i], particles[j], minDistance);
            }
        }

        // Draw everything
        window.clear(sf::Color::Black);
        
        // Draw grid and coverage
        drawGrid(window, gridRows, gridCols, cellWidth, cellHeight, globalCoverage);

        auto now = std::chrono::high_resolution_clock::now();
        float totalTime = std::chrono::duration<float>(now - start_time).count();

        // Calculate total movements across all particles
        int totalMovements = 0;
        for (const auto& p : particles) {
            totalMovements += p.movementsSinceLastCollision;
            for (const auto& [speed, movements] : p.velocityMovementPairs) {
                totalMovements += movements;
            }
        }

        // Draw particles and info
        for (size_t i = 0; i < particles.size(); ++i) {
            const auto& p = particles[i];
            p.draw(window);

            if (fontLoaded) {
                sf::Text label(font, "drone_" + std::to_string(i), 10);
                label.setFillColor(sf::Color::Red);
                label.setPosition(sf::Vector2f(p.getPosition().x + 10, p.getPosition().y - 10));
                window.draw(label);

                sf::Text area(font, "Cells: " + std::to_string((int)p.coveredGridCells.size()), 8);
                area.setFillColor(sf::Color::White);
                area.setPosition(sf::Vector2f(p.getPosition().x + 10, p.getPosition().y + 5));
                window.draw(area);

                // Calculate score based on movements instead of time
                float weightedSum = 0.0f;
                int totalParticleMovements = 0;
                for (const auto& [v, m] : p.velocityMovementPairs) {
                    weightedSum += v * m;
                    totalParticleMovements += m;
                }
                totalParticleMovements += p.movementsSinceLastCollision;

                float score = (totalParticleMovements > 0) ? (weightedSum / totalParticleMovements) : 0.0f;

                sf::Text vel(font, "Score: " + std::to_string((int)score), 8);
                vel.setFillColor(sf::Color::Yellow);
                vel.setPosition(sf::Vector2f(p.getPosition().x + 10, p.getPosition().y + 15));
                window.draw(vel);
            }
        }

        // Display overall coverage info
        if (fontLoaded) {
            float currentCoverage = calculateCoveragePercentage(globalCoverage, gridRows, gridCols);
            sf::Text coverageText(font, "Coverage: " + std::to_string((int)currentCoverage) + "%", 16);
            coverageText.setFillColor(sf::Color::White);
            coverageText.setPosition(sf::Vector2f(10, 10));
            window.draw(coverageText);

            sf::Text movementText(font, "Movements: " + std::to_string(totalMovements), 16);
            movementText.setFillColor(sf::Color::White);
            movementText.setPosition(sf::Vector2f(10, 30));
            window.draw(movementText);

            sf::Text timeText(font, "Time: " + std::to_string((int)totalTime) + "s", 16);
            timeText.setFillColor(sf::Color::White);
            timeText.setPosition(sf::Vector2f(10, 50));
            window.draw(timeText);
        }

        window.display();

        // Check milestones
        if (elapsedSinceLastCheck >= checkInterval) {
            elapsedSinceLastCheck = 0.0f;
            float percent = calculateCoveragePercentage(globalCoverage, gridRows, gridCols);
            tracker.checkMilestone(percent, totalMovements);
            
            std::cout << "Movements: " << totalMovements << ", Grid Coverage: " << percent << "%\n";

            if (percent >= 99.0f) {
                std::cout << "\n99% grid coverage achieved! Simulation complete.\n";
                tracker.printSummary();
                
                sf::sleep(sf::seconds(3));
                window.close();
            }
        }
    }

    // Print final summary
    tracker.printSummary();
    
    std::cout << "\n=== Function Results ===\n";
    int movements50 = tracker.getMovementsFor50Percent();
    int movements60 = tracker.getMovementsFor60Percent();
    int movements70 = tracker.getMovementsFor70Percent();
    int movements80 = tracker.getMovementsFor80Percent();
    int movements90 = tracker.getMovementsFor90Percent();
    int movements95 = tracker.getMovementsFor95Percent();
    int movements99 = tracker.getMovementsFor99Percent();
    
    if (movements50 > 0) std::cout << "getMovementsFor50Percent(): " << movements50 << " total movements\n";
    if (movements60 > 0) std::cout << "getMovementsFor60Percent(): " << movements60 << " total movements\n";
    if (movements70 > 0) std::cout << "getMovementsFor70Percent(): " << movements70 << " total movements\n";
    if (movements80 > 0) std::cout << "getMovementsFor80Percent(): " << movements80 << " total movements\n";
    if (movements90 > 0) std::cout << "getMovementsFor90Percent(): " << movements90 << " total movements\n";
    if (movements95 > 0) std::cout << "getMovementsFor95Percent(): " << movements95 << " total movements\n";
    if (movements99 > 0) std::cout << "getMovementsFor99Percent(): " << movements99 << " total movements\n";

    return 0;
}