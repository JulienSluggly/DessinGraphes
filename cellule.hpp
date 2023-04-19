#ifndef CELLULE_HPP
#define CELLULE_HPP

#include <mutex>

class Cellule {
public:
	int _id; // id unique
    int _numeroX; // numero de la colone dans la grille
    int _numeroY; // numero de la ligne dans la grille

    int _x1, _y1; // Coin haut gauche
    int _x2, _y2; // Coin bas droite

    double _x1reel, _y1reel; // Coin haut gauche
    double _x2reel, _y2reel; // Coin bas droite

    std::vector<int> vecEmplacementId;
    std::vector<int> vecAreteId;

    std::vector<int> containAreteId;

    std::mutex* mutexEmplacement;
    std::mutex* mutexArete;

    Cellule(int id, int nx, int ny, int x1, int y1, int x2, int y2, int nbArete) {
        _id = id; _numeroX = nx; _numeroY = ny;
        _x1 = x1; _x2 = x2; _y1 = y1; _y2 = y2;
        mutexEmplacement = new mutex();
        mutexArete = new mutex();
        containAreteId.resize(nbArete,-1);
    }

    Cellule(int id, int nx, int ny, double x1, double y1, double x2, double y2, int nbArete) {
        _id = id; _numeroX = nx; _numeroY = ny;
        _x1reel = x1; _x2reel = x2; _y1reel = y1; _y2reel = y2;
        mutexEmplacement = new mutex();
        mutexArete = new mutex();
        containAreteId.resize(nbArete,-1);
    }

    int getTopLeftX() { return _x1; }
    int getTopLeftY() { return _y1; }
    int getTopRightX() { return _x2; }
    int getTopRightY() { return _y1; }
    int getBottomLeftX() { return _x1; }
    int getBottomLeftY() { return _y2; }
    int getBottomRightX() { return _x2; }
    int getBottomRightY() { return _y2; }

    double getTopLeftXReel() { return _x1reel; }
    double getTopLeftYReel() { return _y1reel; }
    double getTopRightXReel() { return _x2reel; }
    double getTopRightYReel() { return _y1reel; }
    double getBottomLeftXReel() { return _x1reel; }
    double getBottomLeftYReel() { return _y2reel; }
    double getBottomRightXReel() { return _x2reel; }
    double getBottomRightYReel() { return _y2reel; }

};

#endif