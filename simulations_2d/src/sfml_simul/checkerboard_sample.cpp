#include <SFML/Graphics.hpp>

int main()
{
    int n;
    n =8;
    sf::RenderWindow window(sf::VideoMode(100*n, 100*n), "chessboard");
    std::vector<std::vector<sf::RectangleShape>> whiteRect(n, std::vector<sf::RectangleShape>(n));
    std::vector<std::vector<sf::RectangleShape>> blackRect(n, std::vector<sf::RectangleShape>(n));

    std::vector<std::vector<std::pair<float, float>>> coordinates(n, std::vector<std::pair<float, float>>(n));

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            coordinates[i][j].first = i * 100;
            coordinates[i][j].second = j * 100;
        }
    }

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            whiteRect[i][j].setSize(sf::Vector2f(100.f, 100.f));
            whiteRect[i][j].setFillColor(sf::Color::White);
            whiteRect[i][j].setPosition(coordinates[i][j].first, coordinates[i][j].second);
        }
    }

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            blackRect[i][j].setSize(sf::Vector2f(100.f, 100.f));
            blackRect[i][j].setFillColor(sf::Color::Black);
            blackRect[i][j].setPosition(coordinates[i][j].first, coordinates[i][j].second);
        }
    }

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {

            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::Green);

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if ((i + j) % 2 == 0)
                {
                    window.draw(whiteRect[i][j]);
                }
                else
                {
                    window.draw(blackRect[i][j]);
                }
            }
        }

        window.display();
    }

    return 0;
}
