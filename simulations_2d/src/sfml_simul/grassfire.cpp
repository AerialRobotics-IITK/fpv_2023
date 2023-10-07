#include <SFML/Graphics.hpp>
#include <queue>
#include <iostream>
#include <chrono>
#include <thread>
#include <limits>

const int gridSize = 14;
const int cellSize = 50;

enum CellState {
    Empty,
    Obstacle,
    Path,
    Start,
    End, 
    Pathfound
};

std::vector<std::vector<CellState>> grid(gridSize, std::vector<CellState>(gridSize, Empty));

int start[] = {3, 0}; int end[] = {gridSize-3, gridSize-3}; 
int startRow = start[0], startCol = start[1];
int endRow = end[0], endCol = end[1];

int dx[] = {-1, 1, 0, 0};
int dy[] = {0, 0, -1, 1};

void drawGrid(sf::RenderWindow& window) {
    for (int row = 0; row < gridSize; ++row) {
        for (int col = 0; col < gridSize; ++col) {
            sf::RectangleShape cell(sf::Vector2f(cellSize, cellSize));
            cell.setPosition(col * cellSize, row * cellSize);
            cell.setOutlineThickness(1);
            cell.setOutlineColor(sf::Color::Black); 
            if (grid[row][col] == Empty)
                cell.setFillColor(sf::Color::White);
            else if (grid[row][col] == Obstacle)
                cell.setFillColor(sf::Color::Black);
            else if (grid[row][col] == Start)
                cell.setFillColor(sf::Color::Green);
            else if (grid[row][col] == End)
                cell.setFillColor(sf::Color::Red);
            else if (grid[row][col] == Path)
                cell.setFillColor(sf::Color::Blue);
            else if (grid[row][col] == Pathfound) {
                cell.setFillColor(sf::Color::Magenta);
            }

            window.draw(cell);
        }
    }
}

void grassfire(sf::RenderWindow& window) { 
    int gridarray[gridSize][gridSize] = {};
    for (int i=0; i<gridSize; i++) {
        for (int j=0; j<gridSize; j++) {
            gridarray[i][j] = std::numeric_limits<int>::max();
        }
    }

    std::queue<std::pair<int, int>> q;
    gridarray[endRow][endCol] = 0;
    q.push({endRow, endCol});

    bool pathFound = false;

    while (!q.empty() && !pathFound) {
        int row = q.front().first;
        int col = q.front().second;
        q.pop();

        for (int d = 0; d < 4; ++d) {
            int newRow = row + dx[d];
            int newCol = col + dy[d];

            if (newRow >= 0 && newRow < gridSize && newCol >= 0 && newCol < gridSize &&
                gridarray[newRow][newCol] == std::numeric_limits<int>::max() && grid[newRow][newCol] != Obstacle) {
                grid[newRow][newCol] = Path;
                gridarray[newRow][newCol] = gridarray[row][col] + 1;

                sf::sleep(sf::milliseconds(10));
                window.clear(); drawGrid(window); window.display();

                q.push({newRow, newCol});

                if (newRow == startRow && newCol == startCol) { pathFound = true; break; }
            }
        }
    }

    if (pathFound) {
        int row = startRow; int col = startCol;
        while (gridarray[row][col]>0) {
            grid[row][col] = Pathfound;
            sf::sleep(sf::milliseconds(10));
            window.clear();
            drawGrid(window);
            window.display();

            int val = gridarray[row][col] - 1;
            int nr; int nc;
            if (val == gridarray[row][col+1]) {
                nr = row; nc = col+1;
            } else if (val == gridarray[row+1][col]) {
                nr = row+1; nc = col;
            } else if (val == gridarray[row-1][col]) {
                nr = row-1; nc = col;
            } else if (val == gridarray[row][col-1]) {
                nr = row; nc = col-1;
            }
            row = nr; col = nc;
        }
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode(gridSize*cellSize, gridSize*cellSize), "Grassfire Path Planning");

    grid[2][2] = grid[3][2] = grid[4][2] = grid[5][2] = grid[6][2] = grid[7][3] = grid[7][5] = Obstacle;

    grid[startRow][startCol] = Start;
    grid[endRow][endCol] = End;

    drawGrid(window);
    window.display();

    sf::sleep(sf::seconds(1));

    grassfire(window);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();

        drawGrid(window);

        window.display();
    }
    return 0;
}

