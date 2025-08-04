#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <chrono>
#include <set>

struct Particle {
    sf::CircleShape outer;
    sf::CircleShape inner;
    sf::Vector2f velocity;

    std::set<std::pair<int, int>> coveredPixels;
    std::vector<std::pair<float, float>> velocityTimePairs;
    float timeSinceLastCollision = 0.0f;

    Particle(float radius, sf::Vector2f pos, sf::Vector2f vel)
        : velocity(vel) {
        outer.setRadius(radius);
        outer.setFillColor(sf::Color::White);
        outer.setOrigin(radius, radius);
        outer.setPosition(pos);

        float innerRadius = radius * 0.3f;
        inner.setRadius(innerRadius);
        inner.setFillColor(sf::Color::Cyan);
        inner.setOrigin(innerRadius, innerRadius);
        inner.setPosition(pos);
    }

    void move(float dt) {
        timeSinceLastCollision += dt;
        sf::Vector2f delta = velocity * dt;
        outer.move(delta);
        inner.move(delta);
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
    }

    void draw(sf::RenderWindow& window) const {
        window.draw(outer);
        window.draw(inner);
    }

    void drawTrail(sf::RenderTexture& trailTexture, sf::Color color) {
        sf::CircleShape trail;
        trail.setRadius(outer.getRadius());
        trail.setOrigin(outer.getRadius(), outer.getRadius());
        trail.setPosition(outer.getPosition());
        trail.setFillColor(color);
        trailTexture.draw(trail, sf::BlendAlpha);

        int px = static_cast<int>(outer.getPosition().x);
        int py = static_cast<int>(outer.getPosition().y);
        coveredPixels.insert({px, py});
    }

    void registerCollision() {
        float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
        velocityTimePairs.emplace_back(speed, timeSinceLastCollision);
        timeSinceLastCollision = 0.0f;
    }
};

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

void handleParticleCollision(Particle& p1, Particle& p2) {
    sf::Vector2f diff = p1.getPosition() - p2.getPosition();
    float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
    float minDist = p1.getRadius() + p2.getRadius();

    if (dist < minDist && dist > 0) {
        sf::Vector2f normal = diff / dist;
        sf::Vector2f relativeVelocity = p1.velocity - p2.velocity;
        float dot = relativeVelocity.x * normal.x + relativeVelocity.y * normal.y;

        if (dot < 0) {
            float impulse = 2 * dot / 2;
            p1.velocity -= impulse * normal;
            p2.velocity += impulse * normal;

            p1.registerCollision();
            p2.registerCollision();
        }
    }
}

int main() {
    int width = 800, height = 600, numParticles = 6;
    float radius = 32.0f;
    float maxSpeed = 160.0f;

    std::cout << "Enter number of particles: ";
    std::cin >> numParticles;

    std::cout << "Enter screen width and height (e.g., 800 600): ";
    std::cin >> width >> height;

    sf::RenderWindow window(sf::VideoMode(width, height), "Brownian Motion with Coverage");
    window.setFramerateLimit(60);

    sf::RenderTexture trailTexture;
    trailTexture.create(width, height);
    trailTexture.clear(sf::Color::Black);
    trailTexture.display();

    sf::Texture texture = trailTexture.getTexture();
    sf::Sprite trailSprite(texture);

    sf::Font font;
    if (!font.loadFromFile("Arial.ttf")) {
        std::cerr << "Failed to load font.\n";
        return -1;
    }

    std::srand(std::time(nullptr));
    std::vector<Particle> particles;

    for (int i = 0; i < numParticles; ++i) {
        float x = radius + std::rand() % (width - (int)(2 * radius));
        float y = radius + std::rand() % (height - (int)(2 * radius));
        float vx = ((std::rand() % 200) / 100.0f - 1) * maxSpeed;
        float vy = ((std::rand() % 200) / 100.0f - 1) * maxSpeed;

        particles.emplace_back(radius, sf::Vector2f(x, y), sf::Vector2f(vx, vy));
    }

    sf::Clock clock;
    const float checkInterval = 1.0f;
    float elapsedSinceLastCheck = 0.0f;
    auto start_time = std::chrono::high_resolution_clock::now();

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event))
            if (event.type == sf::Event::Closed)
                window.close();

        float dt = clock.restart().asSeconds();
        elapsedSinceLastCheck += dt;

        for (auto& p : particles) {
            p.drawTrail(trailTexture, sf::Color(50, 50, 50));
            p.move(dt);
        }

        for (size_t i = 0; i < particles.size(); ++i) {
            handleWallCollision(particles[i], width, height);
            for (size_t j = i + 1; j < particles.size(); ++j) {
                handleParticleCollision(particles[i], particles[j]);
            }
        }

        trailTexture.display();
        trailSprite.setTexture(trailTexture.getTexture());

        window.clear();
        window.draw(trailSprite);

        auto now = std::chrono::high_resolution_clock::now();
        float totalTime = std::chrono::duration<float>(now - start_time).count();

        for (size_t i = 0; i < particles.size(); ++i) {
            const auto& p = particles[i];
            p.draw(window);

            sf::Text label("drone_" + std::to_string(i), font, 12);
            label.setFillColor(sf::Color::Red);
            label.setPosition(p.getPosition().x + 10, p.getPosition().y - 10);
            window.draw(label);

            sf::Text area("Area: " + std::to_string((int)p.coveredPixels.size()), font, 10);
            area.setFillColor(sf::Color::Green);
            area.setPosition(p.getPosition().x + 10, p.getPosition().y + 5);
            window.draw(area);

            float weightedSum = 0.0f, timeSum = 0.0f;
            for (const auto& [v, t] : p.velocityTimePairs) {
                weightedSum += v * t;
                timeSum += t;
            }

            float score = (totalTime > 0.0f) ? (weightedSum / totalTime) : 0.0f;

            sf::Text vel("Score: " + std::to_string(score), font, 10);
            vel.setFillColor(sf::Color::Yellow);
            vel.setPosition(p.getPosition().x + 10, p.getPosition().y + 20);
            window.draw(vel);
        }

        window.display();

        if (elapsedSinceLastCheck >= checkInterval) {
            elapsedSinceLastCheck = 0.0f;
            sf::Image img = trailTexture.getTexture().copyToImage();
            std::size_t covered = 0;
            for (unsigned y = 0; y < img.getSize().y; ++y) {
                for (unsigned x = 0; x < img.getSize().x; ++x) {
                    sf::Color pixel = img.getPixel(x, y);
                    if (pixel != sf::Color::Black)
                        ++covered;
                }
            }
            float total = width * height;
            float percent = (covered / total) * 100.0f;
            std::cout << "Covered: " << percent << "%\n";

            if (percent >= 99.0f) {
                std::cout << "99% area covered in " << totalTime << " seconds. Exiting...\n";
                window.close();
            }
        }
    }

    return 0;
}
