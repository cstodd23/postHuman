#include "raylib.h"
#include "raymath.h"
#include <iostream>
#include "verlet.hpp"

const int SCREEN_WIDTH = 1920;
const int SCREEN_HEIGHT = 1080;


int main() {
    Game game = Game(SCREEN_WIDTH, SCREEN_HEIGHT);
    game.MainLoop();

    std::cout << "Goodbye World\n";
    return 0;
}