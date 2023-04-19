#ifndef DISPOPENGL_HPP
#define DISPOPENGL_HPP

#include "personnel.hpp"

#if defined(OPENGL_INSTALLED)

#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <stdio.h>
#include <random>
#include "graphe.hpp"
#include "ogdfFunctions.hpp"

bool useReel = false;
bool singleKeyPress = false;
int keyPressFunctionNum = -1;

double windowWidth = 1750.0, windowHeight=900.0;
int gridWidth, gridHeight, initialGridWidth, initialGridHeight;

bool repeatInfinitely = false;
bool display_genetic = false;
bool isGeneticSetUp = false;
bool show_triangulation = false;
bool show_cells = false;
double currentZoom = 0.0;
bool moving = false;
int selectedNode = 0;
bool show_selected_emplacement = false;
bool show_selected_node = false;
bool show_selected_cell = false;
int selectedCellX = 0, selectedCellY=0;
int maxCellX = 0, maxCellY = 0;
int selectedEmplacement = 0;
int maxNodeIndex = 0;
int maxEmplacementIndex = 0;
bool showIllegal = false;
bool showEmplacement = true;
bool showEdges = true;
bool showNodes = true;
bool stepSetup = false;
bool isCursorDraging = false, dragModeOn = false;
double dragOriginX, dragOriginY;
double stepT;
int stepNbCrois;
std::vector<int> graphCopy;
// Parents pour affichage genetique
Graphe parent1("parent1"), parent2("parent2"), enfant("enfant");
int nbNoeudATraiter, currentGrapheNumber=0;
double initialClicX, initialClicY;
double sensiDrag;
double clicX=0.0, clicY=0.0;
double cursorPosX, cursorPosY;
double initialMargeXDebut = 1, initialMargeXFin = 1, initialMargeYDebut = 1, initialMargeYFin = 1;
double margeXDebut, margeXFin, margeYDebut, margeYFin;
double orthoStartX, orthoStartY, orthoEndX, orthoEndY;
double originalOrthoStartX, originalOrthoStartY, originalOrthoEndX, originalOrthoEndY;
std::vector<std::vector<double>> historiqueOpenGL;

void updateOrtho() {
	orthoStartX = -margeXDebut - (gridWidth * currentZoom);
	orthoEndX = gridWidth + margeXFin +(gridWidth * currentZoom);
	orthoStartY = -margeYDebut - (gridHeight * currentZoom);
	orthoEndY = gridHeight + margeYFin + (gridHeight * currentZoom);
}

void error_callback(int error, const char* description) {
	fputs(description, stderr);
}

void window_size_callback(GLFWwindow* window, int width, int height) {
	windowWidth = width; windowHeight = height;
}

static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
	cursorPosX = xpos;
	cursorPosY = ypos;
}


static void mouse_scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
	if ((int)yoffset == -1) { // Dezoom
		currentZoom += 0.01;
	}
	else if ((int)yoffset == 1) { // Zoom
		currentZoom -= 0.01;
	}
	updateOrtho();
}

static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
	glfwGetCursorPos(window, &initialClicX, &initialClicY);
	clicY = windowHeight-initialClicY;
	clicX = (orthoEndX-orthoStartX)*(initialClicX/windowWidth)+(orthoStartX-originalOrthoStartX)-1;
	clicY = (orthoEndY-orthoStartY)*(clicY/windowHeight)+(orthoStartY-originalOrthoStartY)-1;
    if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		if (dragModeOn) {
			isCursorDraging = true;
			dragOriginX = initialClicX; dragOriginY = initialClicY;
		}
		else {
			if (show_selected_emplacement) {
				keyPressFunctionNum = 27; singleKeyPress = true;
			}
			else if (show_cells) {
				// TO DO IF NEEDED
			}
			else if (show_selected_node) {
				keyPressFunctionNum = 26; singleKeyPress = true;
			}
		}
    }
	else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
		isCursorDraging = false;
	}
	else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
		if (show_selected_node) {
			keyPressFunctionNum = 29; singleKeyPress = true;
		}
	}
	else if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS) {
		dragModeOn = !dragModeOn;
	}
}

// Callback pour OpenGL
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	if (action == GLFW_PRESS)
		switch (key) {
			// Fermer l'application
		case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GLFW_TRUE); break;
		case GLFW_KEY_R: // Reset Zoom
			currentZoom = 0.0;
			gridWidth = initialGridWidth;
			gridHeight = initialGridHeight;
			margeXDebut = initialMargeXDebut, margeXFin = initialMargeXFin, margeYDebut = initialMargeYDebut, margeYFin = initialMargeYFin;
			orthoStartX = originalOrthoStartX; orthoEndX = originalOrthoEndX; orthoStartY = originalOrthoStartY; orthoEndY = originalOrthoEndY;
			sensiDrag = 10.0;
			break;
			// GAUCHE,DROITE,HAUT,BAS deplace le point selectionne de 1 case dans la direction de la fleche
		case GLFW_KEY_LEFT:
			if (show_selected_emplacement) {
				if (selectedEmplacement > 0) {
					selectedEmplacement--;
				}
			}
			else if (show_cells) {
				if (selectedCellX > 0) {
					selectedCellX--;
					show_selected_cell = true;
				}
			}
			else {
				if (moving) {
					keyPressFunctionNum = 9; singleKeyPress = true;
				}
				else {
					if (selectedNode > 0) {
						selectedNode--;
						show_selected_node = true;
					}
				}
			}
			break;
		case GLFW_KEY_RIGHT:
			if (show_selected_emplacement) {
				if (selectedEmplacement < maxEmplacementIndex) {
					selectedEmplacement++;
				}
			}
			else if (show_cells) {
				if (selectedCellX < maxCellX) {
					selectedCellX++;
					show_selected_cell = true;
				}
			}
			else {
				if (moving) {
					keyPressFunctionNum = 10; singleKeyPress = true;
				}
				else {
					if (selectedNode < maxNodeIndex) {
						selectedNode++;
						show_selected_node = true;
					}
				}
			}
			break;
		case GLFW_KEY_DOWN:
			if (show_cells) {
				if (selectedCellY > 0) {
					selectedCellY--;
					show_selected_cell = true;
				}
			}
			break;
		case GLFW_KEY_UP:
			if (show_cells) {
				if (selectedCellY < maxCellY) {
					selectedCellY++;
					show_selected_cell = true;
				}
			}
			break;
		case GLFW_KEY_1:
			keyPressFunctionNum = 5; singleKeyPress = true;
			break;
		case GLFW_KEY_2:
			keyPressFunctionNum = 6; singleKeyPress = true;
			break;
		case GLFW_KEY_3: // Recuit Simule
			keyPressFunctionNum = 3; singleKeyPress = true;
			break;
		case GLFW_KEY_4:
			keyPressFunctionNum = 7; singleKeyPress = true;
			break;
		case GLFW_KEY_5: // Step Stress Majorization
			keyPressFunctionNum = 20; singleKeyPress = true;
			break;
		case GLFW_KEY_8:
			showEmplacement = !showEmplacement;
			break;
		case GLFW_KEY_9:
			showEdges = !showEdges;
			break;
		case GLFW_KEY_0:
			showNodes = !showNodes;
			break;
		case GLFW_KEY_KP_1: // Copy Graph
			keyPressFunctionNum = 1; singleKeyPress = true;
			break;
		case GLFW_KEY_KP_2: // Load Graph
			keyPressFunctionNum = 2; singleKeyPress = true;
			break;
		case GLFW_KEY_KP_7: // Rotate Graph +5°
			keyPressFunctionNum = 24; singleKeyPress = true;
			break;
		case GLFW_KEY_KP_4: // Rotate Graphe -5°
			keyPressFunctionNum = 25; singleKeyPress = true;
			break;
		case GLFW_KEY_KP_9: // Save Graph
			keyPressFunctionNum = 0; singleKeyPress = true;
			break;
		case GLFW_KEY_F1:
			keyPressFunctionNum = 13; singleKeyPress = true;
			break;
		case GLFW_KEY_F2: // Recuit Low Temp
			keyPressFunctionNum = 28; singleKeyPress = true;
			break;
		case GLFW_KEY_F3:
			keyPressFunctionNum = 12; singleKeyPress = true;
			break;
		case GLFW_KEY_F4:
			keyPressFunctionNum = 14; singleKeyPress = true;
			break;
		case GLFW_KEY_F5:
			keyPressFunctionNum = 4; singleKeyPress = true;
			stepSetup = true;
			break;
		case GLFW_KEY_F6:
			display_genetic = !display_genetic;
			if (display_genetic) {
				keyPressFunctionNum = 15; singleKeyPress = true;
			}
			else {
				gridHeight = initialGridHeight;
				gridWidth = initialGridWidth;
			}
			break;
		case GLFW_KEY_F7:
			keyPressFunctionNum = 16; singleKeyPress = true;
			break;
		case GLFW_KEY_F8:
			keyPressFunctionNum = 19; singleKeyPress = true;
			break;
		case GLFW_KEY_F9: // Print current seed
			keyPressFunctionNum = 21; singleKeyPress = true;
			break;
		case GLFW_KEY_F10: // Generate x2 Emplacements
			keyPressFunctionNum = 30; singleKeyPress = true;
			break;
		case GLFW_KEY_F11: // ogdfGMMM
			keyPressFunctionNum = 32; singleKeyPress = true;
			break;
		case GLFW_KEY_F12: // Repeat Infinitely
			repeatInfinitely = !repeatInfinitely;
			if (repeatInfinitely) { printf("Mode Repeat: ON\n"); }
			else { printf("Mode Repeat: OFF\n"); }
			break;
		case GLFW_KEY_KP_ADD:
			sensiDrag = sensiDrag / 2;
			std::cout << sensiDrag << std::endl;
			break;
		case GLFW_KEY_KP_SUBTRACT:
			sensiDrag *= 2;
			std::cout << sensiDrag << std::endl;
			break;
		case GLFW_KEY_N:
			keyPressFunctionNum = 8; singleKeyPress = true;
			break;
		case GLFW_KEY_D:
			moving = !moving;
			break;
		case GLFW_KEY_E:
			if (!useReel) { show_selected_emplacement = !show_selected_emplacement; }
			break;
		case GLFW_KEY_G:
			show_cells = !show_cells;
			break;
		case GLFW_KEY_S:
			keyPressFunctionNum = 11; singleKeyPress = true;
			break;
		case GLFW_KEY_I:
			showIllegal = !showIllegal;
			keyPressFunctionNum = 18; singleKeyPress = true;
			break;
		case GLFW_KEY_T:
			show_triangulation = !show_triangulation;
			keyPressFunctionNum = 17; singleKeyPress = true;
			break;
		case GLFW_KEY_PAGE_UP: // Ajoute 5 au m_edgecost du SM
			keyPressFunctionNum = 22; singleKeyPress = true;
			break;
		case GLFW_KEY_PAGE_DOWN: // Enleve 5 au m_edgecost du SM
			keyPressFunctionNum = 23; singleKeyPress = true;
			break;
		case GLFW_KEY_BACKSPACE: // CTRL + Z
			keyPressFunctionNum = 31; singleKeyPress = true;
			break;
		case GLFW_KEY_KP_DECIMAL:
			dragModeOn = !dragModeOn;
			break;
		}
}

void openGLShowEmplacement(Graphe& G) {
	if (showEmplacement) {
		if (display_genetic) {
			glPointSize(7);
			glBegin(GL_POINTS);
			glColor3f(0.0f, 1.0f, 0.0f);
			for (int i = 0; i < parent1._emplacements.size(); i++) {
				glVertex2d(parent1._emplacements[i].getX(), parent1._emplacements[i].getY() + parent1.gridHeight*2);
				glVertex2d(parent2._emplacements[i].getX() + parent1.gridWidth*2, parent2._emplacements[i].getY()+ parent1.gridHeight*2);
				glVertex2d(enfant._emplacements[i].getX() + parent1.gridWidth, enfant._emplacements[i].getY());
			}
			glEnd();
		}
		else {
			glPointSize(7);
			glBegin(GL_POINTS);
			glColor3f(0.0f, 1.0f, 0.0f);
			for (int i = 0; i < G._emplacements.size(); i++) {
				glVertex2d(G._emplacements[i].getX(), G._emplacements[i].getY());
			}
			if (show_selected_emplacement) {
				glColor3f(0.5f, 0.0f, 0.0f);
				glVertex2d(G._emplacements[selectedEmplacement].getX(), G._emplacements[selectedEmplacement].getY());
			}
			glEnd();
		}
	}
}

void openGLShowNodes(Graphe& G) {
	if (showNodes) {
		if (display_genetic) {
			glPointSize(7);
			glBegin(GL_POINTS);
			glColor3f(1.0f, 0.0f, 0.0f);
			// Affiche les nodes placés du parent 1, parent 2 et enfant.
			for (int i = 0; i < parent1._noeuds.size(); i++) {
				if (parent1._noeuds[i].getEmplacement() != nullptr) {
					glVertex2d(parent1._noeuds[i].getX(), parent1._noeuds[i].getY()+ parent1.gridHeight*2);
				}
				if (parent2._noeuds[i].getEmplacement() != nullptr) {
					glVertex2d(parent2._noeuds[i].getX() + parent1.gridWidth*2, parent2._noeuds[i].getY()+ parent1.gridHeight*2);
				}
				if (enfant._noeuds[i].getEmplacement() != nullptr) {
					glVertex2d(enfant._noeuds[i].getX() + parent1.gridWidth, enfant._noeuds[i].getY());
				}
			}
			glEnd();
		}
		else {
			glPointSize(7);
			glBegin(GL_POINTS);
			glColor3f(1.0f, 0.0f, 0.0f);
			for (int i = 0; i < G._noeuds.size(); i++) {
				if (G._noeuds[i].estPlace()) {
					glVertex2d(G._noeuds[i].getX(), G._noeuds[i].getY());
				}
			}
			glEnd();
		}
	}
}

void openGLShowNodesReel(Graphe& G) {
	if (showNodes) {
		if (!display_genetic) {
			glPointSize(7);
			glBegin(GL_POINTS);
			glColor3f(1.0f, 0.0f, 0.0f);
			for (int i = 0; i < G._noeuds.size(); i++) {
				glVertex2d(G._noeuds[i]._xreel, G._noeuds[i]._yreel);
			}
			glEnd();
		}
	}
}

void openGLShowEdges(Graphe& G) {
	if (showEdges) {
		glLineWidth(3.0f);
		if (display_genetic) {
			glColor3f(1.0f, 1.0f, 1.0f);
			for (int i = 0; i < parent1._aretes.size(); i++) {
				if (parent1._aretes[i].getNoeud1()->estPlace() && parent1._aretes[i].getNoeud2()->estPlace()) {
					glBegin(GL_LINE_STRIP);
					glVertex2d(parent1._aretes[i].getNoeud1()->getX(), parent1._aretes[i].getNoeud1()->getY() + parent1.gridHeight*2);
					glVertex2d(parent1._aretes[i].getNoeud2()->getX(), parent1._aretes[i].getNoeud2()->getY() + parent1.gridHeight*2);
					glEnd();
				}
				if (parent2._aretes[i].getNoeud1()->estPlace() && parent2._aretes[i].getNoeud2()->estPlace()) {
					glBegin(GL_LINE_STRIP);
					glVertex2d(parent2._aretes[i].getNoeud1()->getX() + parent1.gridWidth*2, parent2._aretes[i].getNoeud1()->getY() + parent1.gridHeight*2);
					glVertex2d(parent2._aretes[i].getNoeud2()->getX() + parent1.gridWidth*2, parent2._aretes[i].getNoeud2()->getY() + parent1.gridHeight*2);
					glEnd();
				}
				if (enfant._aretes[i].getNoeud1()->estPlace() && enfant._aretes[i].getNoeud2()->estPlace()) {
					glBegin(GL_LINE_STRIP);
					glVertex2d(enfant._aretes[i].getNoeud1()->getX() + parent1.gridWidth, enfant._aretes[i].getNoeud1()->getY());
					glVertex2d(enfant._aretes[i].getNoeud2()->getX() + parent1.gridWidth, enfant._aretes[i].getNoeud2()->getY());
					glEnd();
				}
			}
		}
		else {
			glColor3f(1.0f, 1.0f, 1.0f);
			for (int i = 0; i < G._aretes.size(); i++) {
				if (G._aretes[i].estPlace()) {
					glBegin(GL_LINE_STRIP);
					glVertex2d(G._aretes[i].getNoeud1()->getX(), G._aretes[i].getNoeud1()->getY());
					glVertex2d(G._aretes[i].getNoeud2()->getX(), G._aretes[i].getNoeud2()->getY());
					glEnd();
				}
			}
			if (showIllegal) {
				for (auto &a : G.areteInter) {
					glColor3f(0.75f, 0.0f, 0.0f);
					glBegin(GL_LINE_STRIP);
					glVertex2d(a->getNoeud1()->getX(), a->getNoeud1()->getY());
					glVertex2d(a->getNoeud2()->getX(), a->getNoeud2()->getY());
					glEnd();
				}
				for (auto &a : G.areteIll) {
					glColor3f(1.0f, 0.0f, 1.0f);
					glBegin(GL_LINE_STRIP);
					glVertex2d(a->getNoeud1()->getX(), a->getNoeud1()->getY());
					glVertex2d(a->getNoeud2()->getX(), a->getNoeud2()->getY());
					glEnd();
				}
				for (auto &a : G.areteIllSelf) {
					glColor3f(0.3f, 0.2f, 0.5f);
					glBegin(GL_LINE_STRIP);
					glVertex2d(a->getNoeud1()->getX(), a->getNoeud1()->getY());
					glVertex2d(a->getNoeud2()->getX(), a->getNoeud2()->getY());
					glEnd();
				}
			}
		}
	}
}

void openGLShowEdgesReel(Graphe& G) {
	if (showEdges) {
		glLineWidth(3.0f);
		if (!display_genetic) {
			glColor3f(1.0f, 1.0f, 1.0f);
			for (int i = 0; i < G._aretes.size(); i++) {
				glBegin(GL_LINE_STRIP);
				glVertex2d(G._aretes[i].getNoeud1()->_xreel, G._aretes[i].getNoeud1()->_yreel);
				glVertex2d(G._aretes[i].getNoeud2()->_xreel, G._aretes[i].getNoeud2()->_yreel);
				glEnd();
			}
			if (showIllegal) {
				for (auto &a : G.areteInter) {
					glColor3f(0.75f, 0.0f, 0.0f);
					glBegin(GL_LINE_STRIP);
					glVertex2d(a->getNoeud1()->_xreel, a->getNoeud1()->_yreel);
					glVertex2d(a->getNoeud2()->_xreel, a->getNoeud2()->_yreel);
					glEnd();
				}
				for (auto &a : G.areteIll) {
					glColor3f(1.0f, 0.0f, 1.0f);
					glBegin(GL_LINE_STRIP);
					glVertex2d(a->getNoeud1()->_xreel, a->getNoeud1()->_yreel);
					glVertex2d(a->getNoeud2()->_xreel, a->getNoeud2()->_yreel);
					glEnd();
				}
				for (auto &a : G.areteIllSelf) {
					glColor3f(0.3f, 0.2f, 0.5f);
					glBegin(GL_LINE_STRIP);
					glVertex2d(a->getNoeud1()->_xreel, a->getNoeud1()->_yreel);
					glVertex2d(a->getNoeud2()->_xreel, a->getNoeud2()->_yreel);
					glEnd();
				}
			}
		}
	}
}

void openGLShowTriangulation(Graphe& G) {
	if (show_triangulation) {
		glLineWidth(2.0f);
		glColor3f(0.2f, 0.3f, 0.4f);
		for (int i=0;i<G._c.nbDemiCote();i+=2) {
			int x1 = G._c.demiCote(i)->sommet()->getX();
			int y1 = G._c.demiCote(i)->sommet()->getY();
			int x2 = G._c.demiCote(i)->oppose()->sommet()->getX();
			int y2 = G._c.demiCote(i)->oppose()->sommet()->getY();
			glBegin(GL_LINE_STRIP);
			glVertex2d(x1, y1);
			glVertex2d(x2, y2);
			glEnd();
		}
	}
}

void openGLShowCells(Graphe& G) {
	if (show_cells) {
		glLineWidth(2.0f);
		glColor3f(0.8f, 0.7f, 0.6f);
		for (int i=0;i<G.grille.size();i++) {
			for (int j=0;j<G.grille[i].size();j++) {
				int x1 = G.grille[i][j].getBottomRightX();
				int y1 = G.grille[i][j].getBottomRightY();
				int x2 = G.grille[i][j].getBottomLeftX();
				int y2 = G.grille[i][j].getTopRightY();
				glBegin(GL_LINE_STRIP);
				glVertex2d(x1, y1);
				glVertex2d(x2, y1);
				glEnd();
				glBegin(GL_LINE_STRIP);
				glVertex2d(x1, y1);
				glVertex2d(x1, y2);
				glEnd();
				if (j == 0) {
					glBegin(GL_LINE_STRIP);
					glVertex2d(x2, y1);
					glVertex2d(x2, y2);
					glEnd();
				}
				if (i == G.grille.size()-1) {
					glBegin(GL_LINE_STRIP);
					glVertex2d(x2, y2);
					glVertex2d(x1, y2);
					glEnd();
				}
			}
		}
		if (show_selected_cell) {
			int x1 = G.grille[selectedCellY][selectedCellX].getTopLeftX();
			int y1 = G.grille[selectedCellY][selectedCellX].getTopLeftY();
			int x2 = G.grille[selectedCellY][selectedCellX].getBottomRightX();
			int y2 = G.grille[selectedCellY][selectedCellX].getBottomRightY();
			glBegin(GL_QUADS);
			glVertex2f(x1, y1);
			glVertex2f(x2, y1);
			glVertex2f(x2, y2);
			glVertex2f(x1, y2);
			glEnd();
		}
	}
}

void openGLShowCellsReel(Graphe& G) {
	if (show_cells) {
		glLineWidth(2.0f);
		glColor3f(0.8f, 0.7f, 0.6f);
		for (int i=0;i<G.grille.size();i++) {
			for (int j=0;j<G.grille[i].size();j++) {
				double x1 = G.grille[i][j].getBottomRightXReel();
				double y1 = G.grille[i][j].getBottomRightYReel();
				double x2 = G.grille[i][j].getBottomLeftXReel();
				double y2 = G.grille[i][j].getTopRightYReel();
				glBegin(GL_LINE_STRIP);
				glVertex2d(x1, y1);
				glVertex2d(x2, y1);
				glEnd();
				glBegin(GL_LINE_STRIP);
				glVertex2d(x1, y1);
				glVertex2d(x1, y2);
				glEnd();
				if (j == 0) {
					glBegin(GL_LINE_STRIP);
					glVertex2d(x2, y1);
					glVertex2d(x2, y2);
					glEnd();
				}
				if (i == G.grille.size()-1) {
					glBegin(GL_LINE_STRIP);
					glVertex2d(x2, y2);
					glVertex2d(x1, y2);
					glEnd();
				}
			}
		}
		if (show_selected_cell) {
			double x1 = G.grille[selectedCellY][selectedCellX].getTopLeftXReel();
			double y1 = G.grille[selectedCellY][selectedCellX].getTopLeftYReel();
			double x2 = G.grille[selectedCellY][selectedCellX].getBottomRightXReel();
			double y2 = G.grille[selectedCellY][selectedCellX].getBottomRightYReel();
			glBegin(GL_QUADS);
			glVertex2f(x1, y1);
			glVertex2f(x2, y1);
			glVertex2f(x2, y2);
			glVertex2f(x1, y2);
			glEnd();
		}
	}
}

void openGLShowSelected(Graphe& G) {
	glPointSize(7);
	glBegin(GL_POINTS);
	if (show_selected_node) {
		if (G._noeuds[selectedNode].estPlace()) {
			glColor3f(0.0f, 0.0f, 1.0f);
			if (moving) {
				glColor3f(0.0f, 0.5f, 0.0f);
			}
			glVertex2d(G._noeuds[selectedNode].getX(), G._noeuds[selectedNode].getY());
		}
	}
	if (show_selected_emplacement) {
		glColor3f(0.5f, 0.0f, 0.0f);
		glVertex2d(G._emplacements[selectedEmplacement].getX(), G._emplacements[selectedEmplacement].getY());
	}
	glEnd();
}

void openGLShowSelectedReel(Graphe& G) {
	glPointSize(7);
	glBegin(GL_POINTS);
	if (show_selected_node) {
		glColor3f(0.0f, 0.0f, 1.0f);
		if (moving) {
			glColor3f(0.0f, 0.5f, 0.0f);
		}
		glVertex2d(G._noeuds[selectedNode]._xreel, G._noeuds[selectedNode]._yreel);
	}
	glEnd();
}

void openGLShowGrid() {
	// affichage de la grille avec une marge de 1
	glLineWidth(2.0f);
	glColor3f(0.0f, 1.0f, 0.0f);
	glBegin(GL_LINE_STRIP);
	glVertex2d(-1, -1);
	glVertex2d(gridWidth + 1, -1);
	glVertex2d(gridWidth + 1, gridHeight + 1);
	glVertex2d(-1, gridHeight + 1);
	glVertex2d(-1, -1);
	glEnd();
}

void openGLPrintRaccourcis(bool printRaccourcis) {
	if (printRaccourcis) {
		std::cout << "-------------------------------------" << std::endl;
		std::cout << "RACCOURCIS CLAVIER POUR INTERRACTION OPENGL" << std::endl;
		std::cout << "Touche Escape/Echap: Ferme la fenetre OpenGl." << std::endl;
		std::cout << "Touche Gauche/Left: " << std::endl;
		std::cout << "     -Si en mode selection de noeud(par defaut): Selectionne le noeud precedent." << std::endl;
		std::cout << "     -Si en mode selection d'emplacement: Selectionne l'emplacement precedent." << std::endl;
		std::cout << "     -Si en mode deplacement: Deplace le noeud selectionne a un emplacement precedent le sien." << std::endl;
		std::cout << "Touche Droite/Right: Similaire a Gauche/Left mais pour le suivant." << std::endl;
		std::cout << "Touche 1/&: Reinitialise les noeuds du graphe et effectue un placement aleatoire." << std::endl;
		std::cout << "Touche 2/e: Reinitialise les noeuds du graphe et effectue un placement glouton." << std::endl;
		std::cout << "Touche 3/\": Appelle la fonction Recuit Simule sur le graphe." << std::endl;
		std::cout << "Touche 4/' Affiche le score du graphe, du noeud selectionne, et le score swap si l'emplacement selectionne n'est pas disponible." << std::endl;
		std::cout << "Touche 5/( Calcule la moyenne de score de 1000 placements aleatoire et de 1000 placements gloutons." << std::endl;
		std::cout << "Touche NumPad+/KeyPadAdd: Zoom." << std::endl;
		std::cout << "Touche NumPad-/KeyPadMinus: Dezoom." << std::endl;
		std::cout << "Touche N: Affiche le nombre de croisement dans le graphe." << std::endl;
		std::cout << "Touche D: Active/Desactive le mode deplacement." << std::endl;
		std::cout << "Touche E: Active/Desactive le mode selection d'emplacement." << std::endl;
		std::cout << "Touche S: Si le mode selection d'emplacement est actif, effectue un swap entre le noeud selectionne et l'emplacement selectionne." << std::endl;
		std::cout << "Touche I: Active/Desactive l'affichage de croisements(rouge) et croisements illegaux(violet)" << std::endl;
		std::cout << "-------------------------------------" << std::endl;
	}
}

void openGLKeyPressFunction(Graphe& G) {
	if (singleKeyPress) {
		//std::cout << "KeyPress: " << keyPressFunctionNum << std::endl;
		bool recalcIllegal = false;
		switch(keyPressFunctionNum) {
		case 0: {// Save Graphe to JSON
			G.writeToJsonGraph("currentGraphe.json");
			G.writeToJsonSlots("currentGrapheSlots.json");
			break;
		}
		case 1: {// Copie le graphe
			graphCopy.clear();
			graphCopy = G.saveCopy();
			break;
		}
		case 2: {// Load copy
			if (graphCopy.size() == G._noeuds.size()) { G.loadCopy(graphCopy); }
			else { std::cout << "Copy failed.\n"; }
			if (showIllegal) recalcIllegal = true;
			historiqueOpenGL.clear();
			break;
		}
		case 3: {// Recuit simule (KEY: 3")
			if (useReel) {
				std::cout << "Nb Croisement debut recuit: " << G.getNbCroisementReelConst() << std::endl;
				auto start = std::chrono::system_clock::now();
				double timeBest;
				bool useGrille = G.grillePtr.size() > 0;
				G.recuitSimuleReel(timeBest,start,{},0.99999,100.0,0.0001,1,0,2,useGrille,false);
				auto end = std::chrono::system_clock::now();
				std::chrono::duration<double> secondsTotal = end - start;
				std::cout << "Temps calcul: " << secondsTotal.count() << " secondes." << std::endl;
				std::cout << "Temps Meilleur: " << timeBest << " secondes.\n";
				std::cout << "Nb Croisement fin recuit: " << G.getNbCroisementReelConst() << std::endl;
				if (G.grillePtr.size() > 0) { G.reinitGrilleReel(); }
			}
			else {
				std::cout << "Nb Croisement debut recuit: " << G.getNbCroisement() << std::endl;
				auto start = std::chrono::system_clock::now();
				double timeBest;
				bool useGrille = G.grillePtr.size() > 0;
				if (useGrille) { G.reinitGrille(); }
				G.recuitSimule(timeBest,start,{},0.99999,100.0,0.0001,1,0,2,useGrille);
				auto end = std::chrono::system_clock::now();
				std::chrono::duration<double> secondsTotal = end - start;
				std::cout << "Temps calcul: " << secondsTotal.count() << " secondes." << std::endl;
				std::cout << "Temps Meilleur: " << timeBest << " secondes.\n";
				std::cout << "Nb Croisement fin recuit: " << G.getNbCroisement() << std::endl;
				if (useGrille) { G.reinitGrille(); }
			}
			historiqueOpenGL.clear();
			break;
		}
		case 4: {// Step Recuit Simule
			if (stepSetup) {
				stepT = 100.0;
				stepNbCrois = G.getNbCroisement();
				stepSetup = false;
			}
			G.stepRecuitSimule(stepT, stepNbCrois);
			if (stepT <= 0.0001 || stepNbCrois == 0) {
				std::cout << stepT << " " << stepNbCrois << std::endl;
			}
			historiqueOpenGL.clear();
			break;
		}
		case 5: {// Placement aleatoire (KEY: 1&)
			if (display_genetic) {
				parent1.clearNodeEmplacement();
				parent2.clearNodeEmplacement();
				enfant.clearNodeEmplacement();
				parent1.placementAleatoire();
				parent2.placementAleatoire();
				parent1.initGraphAndNodeScoresAndCrossings();
				parent2.initGraphAndNodeScoresAndCrossings();
				nbNoeudATraiter = parent1._noeuds.size() - parent1.nbNoeudEnCommun(parent2);
			}
			else {
				if (!useReel) {
					G.clearNodeEmplacement();
					G.placementAleatoire();
					if (G.grillePtr.size() > 0) { G.reinitGrille(); }
				}
				else {
					G.placementAleatoireReel();
					if (G.grillePtr.size() > 0 ) { G.reinitGrilleReel(); }
				}
			}
			if (showIllegal) recalcIllegal = true;
			historiqueOpenGL.clear();
			break;
		}
		case 6: {// Placement Glouton Revisite
			if (display_genetic) {
				parent1.clearNodeEmplacement();
				parent2.clearNodeEmplacement();
				enfant.clearNodeEmplacement();
				parent1.gloutonRevisite();
				parent2.gloutonRevisite();
				parent1.initGraphAndNodeScoresAndCrossings();
				parent2.initGraphAndNodeScoresAndCrossings();
				nbNoeudATraiter = parent1._noeuds.size() - parent1.nbNoeudEnCommun(parent2);
			}
			else {
				G.clearNodeEmplacement();
				G.gloutonRevisite();
			}
			if (showIllegal) recalcIllegal = true;
			historiqueOpenGL.clear();
			break;
		}
		case 7: {// Affiche score (KEY: 4')
			if (!display_genetic) {
				if (!useReel) {
					G.getNbCroisementDiff();
					std::cout << "Total Inter: " << G.nombreInter + G.nombreInterIll + G.nombreInterIllSelf << " normales: " << G.nombreInter << " illegales: " << G.nombreInterIll << " self: " << G.nombreInterIllSelf << std::endl;
					//std::cout << "Selected Node: " << selectedNode << " emplacement: " << G._noeuds[selectedNode].getEmplacement()->_id << " Selected Emplacement: " << selectedEmplacement << std::endl;
					std::cout << "Nb Intersection: " << G.getNbCroisement() << std::endl;
					//std::cout << "Selected node score: " << G.getScoreCroisementNode(selectedNode) << std::endl;
					if (show_selected_emplacement) {
						if (!G._emplacements[selectedEmplacement].estDisponible()) {
							int swapId = G._emplacements[selectedEmplacement]._noeud->getId();
							std::cout << "Selected emplacement score: " << G.getScoreCroisementNode(swapId) << std::endl;
							int score = G.getScoreCroisementNode(selectedNode);
							score += G.getScoreCroisementNode(swapId, selectedNode);
							std::cout << "Score before swap: " << score << std::endl;
							G._noeuds[selectedNode].swap(G._noeuds[swapId].getEmplacement());
							score = G.getScoreCroisementNode(selectedNode);
							score += G.getScoreCroisementNode(swapId, selectedNode);
							std::cout << "Score after swap: " << score << std::endl;
							G._noeuds[selectedNode].swap(G._noeuds[swapId].getEmplacement());
						}
					}
				}
				else {
					G.getNbCroisementDiffReel();
					std::cout << "Total Inter: " << G.nombreInter + G.nombreInterIll + G.nombreInterIllSelf << " normales: " << G.nombreInter << " illegales: " << G.nombreInterIll << " self: " << G.nombreInterIllSelf << std::endl;
				}
			}
			else {
				std::cout << "Score Parent1: " << parent1.nombreCroisement << "&" << parent1.getNbCroisementConst() << "&" << parent1.getNbCroisementArray() << std::endl;
				std::cout << "Score Parent2: " << parent2.nombreCroisement << "&" << parent2.getNbCroisementConst() << "&" << parent2.getNbCroisementArray() <<std::endl;
				//std::cout << "Score Enfant: " << enfant.nombreCroisement<< "&" << enfant.getNbCroisementConst() << std::endl;
			}
			break;
		}
		case 8: {// Affiche le nombre d'intersection du graphe
			if (display_genetic) {
				if (parent1.estPlace()) std::cout << "Parent1 Nb Intersections: " << parent1.getNbCroisementConst() << " saved: " << parent1.nombreCroisement << " array: " << parent1.getNbCroisementArray() << std::endl;
				if (parent2.estPlace()) std::cout << "Parent2 Nb Intersections: " << parent2.getNbCroisementConst() << " saved: " << parent2.nombreCroisement << " array: " << parent2.getNbCroisementArray() << std::endl;
				if (enfant.estPlace()) std::cout << "Parent3 Nb Intersections: " << enfant.getNbCroisementConst() << " saved: " << enfant.nombreCroisement << " array: " << enfant.getNbCroisementArray() << std::endl;
			}
			else {
				std::cout << "Nb Intersections: " << G.getNbCroisementConst() << " saved: " << G.nombreCroisement << " array: " << G.getNbCroisementArray() << std::endl;
			}
			break;
		}
		case 9: {// Move current node left
			if (!display_genetic) {
				G.clearSetAreteInter();
				int numPos = G._noeuds[selectedNode].getEmplacement()->getId();
				int i = numPos-1;
				while (!G._emplacements[i].estDisponible() && i>= 0) {
					i--;
				}
				G._noeuds[selectedNode].setEmplacement(&G._emplacements[i]);
				if (showIllegal) recalcIllegal = true;
			}
			historiqueOpenGL.clear();
			break;
		}
		case 10: {// Move current node right
			if (!display_genetic) {
				G.clearSetAreteInter();
				int numPos = G._noeuds[selectedNode].getEmplacement()->getId();
				int i = numPos+1;
				while (!G._emplacements[i].estDisponible() && i<G._emplacements.size()) {
					i++;
				}
				G._noeuds[selectedNode].setEmplacement(&G._emplacements[i]);
				if (showIllegal) recalcIllegal = true;
			}
			historiqueOpenGL.clear();
			break;
		}
		case 11: {// Make Swap (KEY: S)
			G.clearSetAreteInter();
			if (G._noeuds[selectedNode].getEmplacement()->getId() != selectedEmplacement) {
				int oldEmplacement = G._noeuds[selectedNode].getEmplacement()->getId();
				if (G._emplacements[selectedEmplacement].estDisponible()) {
					G._noeuds[selectedNode].setEmplacement(&G._emplacements[selectedEmplacement]);
				}
				else {
					G._noeuds[selectedNode].swap(&G._emplacements[selectedEmplacement]);
				}
				selectedEmplacement = oldEmplacement;
			}
			if (showIllegal) recalcIllegal = true;
			historiqueOpenGL.clear();
			break;
		}
		case 12: {// Debug All Info (KEY: F3)
			if (display_genetic) {
				parent1.afficherInfo(); parent1.afficherEmplacement(); parent1.afficherLiens(); parent1.afficherNoeuds();
				parent2.afficherInfo(); parent2.afficherEmplacement(); parent2.afficherLiens(); parent2.afficherNoeuds();
			}
			else {
				if (useReel) {
					G.afficherNoeudsReel();
				}
				else {
					G.afficherInfo();
					//G.afficherNoeuds();
					G.afficherLiens();
					//G.afficherEmplacement();
				}
			}
			break;
		}
		case 13: {// Debug Specific Info
			if (display_genetic) {
				//parent1.afficherInfo(); parent1.afficherEmplacement(); parent1.afficherLiens(); parent1.afficherNoeuds();
				//parent2.afficherInfo(); parent2.afficherEmplacement(); parent2.afficherLiens(); parent2.afficherNoeuds();
				parent1.afficherNoeuds();
				parent2.afficherNoeuds();
				//enfant.afficherInfo(); enfant.afficherEmplacement(); enfant.afficherLiens(); enfant.afficherNoeuds();
			}
			else {
				G.afficherInfo();
				//G.afficherNoeuds();
				G.afficherLiens();
				//G.afficherEmplacement();
			}
			break;
		}
		case 14: {
			break;
		}
		case 15: {// Affiche fenetre genetique
			if (!isGeneticSetUp) {
				std::string nomFichierParent1 = chemin + "combined/" + "cgraph-1" + ".json";
				std::string nomFichierParent2 = chemin + "combined/" + "cgraph-2" + ".json";
				parent1.readFromJsonGraphAndSlot(nomFichierParent1);
				parent2.readFromJsonGraphAndSlot(nomFichierParent2);
				enfant.readFromJsonGraphAndSlot(nomFichierParent1);
				enfant.clearNodeEmplacement();
				nbNoeudATraiter = parent1._noeuds.size() - parent1.nbNoeudEnCommun(parent2);
				isGeneticSetUp = true;
				parent1.initGraphAndNodeScoresAndCrossings();
				parent2.initGraphAndNodeScoresAndCrossings();
				std::cout << "Genetic set up:\n" << "File Parent1: " << nomFichierParent1 << "\nFile Parent2: " << nomFichierParent2 << "\nNNT: " << nbNoeudATraiter << std::endl;
				std::cout << "Score parent1: " << parent1.nombreCroisement << " Score parent2: " << parent2.nombreCroisement << std::endl;
				parent1.debugScoreNoeud();
				parent2.debugScoreNoeud();
				//parent1.afficherInfo(); parent1.afficherEmplacement(); parent1.afficherLiens(); parent1.afficherNoeuds();
				//parent2.afficherInfo(); parent2.afficherEmplacement(); parent2.afficherLiens(); parent2.afficherNoeuds();
			}
			gridHeight = parent1.gridHeight*3;
			gridWidth = parent1.gridWidth*3;
			break;
		}
		case 16: {// Step Genetique
			if (display_genetic) {
				if (nbNoeudATraiter > 0) {
					enfant.stepCroisementVoisinageFrom(parent1,parent2,false,nbNoeudATraiter,currentGrapheNumber);
					//enfant.stepCroisementVoisinageScore(parent1,parent2,false,nbNoeudATraiter,currentGrapheNumber);
					std::cout << "NNT: " << nbNoeudATraiter << std::endl;
				}
			}
			historiqueOpenGL.clear();
			break;
		}
		case 17: {// Triangulation
			if (!G.isCarteSetUp) {
				G.triangulationDelaunay();
			}
			break;
		}
		case 18: {// Recalc Illegal (KEY: I)
			recalcIllegal = true;
			break;
		}
		case 19: {// Affiche le contenu de la cellule (KEY:F8)
			G.afficherLiens();
			if (show_selected_emplacement) {
				std::cout << "Selected Emplacement: " << selectedEmplacement;
				if (selectedEmplacement >= 0) {
					if (G._emplacements[selectedEmplacement]._noeud != nullptr) {
						std::cout << " Non Disponible. Node Id: " << G._emplacements[selectedEmplacement]._noeud->_id << std::endl;
					}
					else {
						std::cout << " Disponible.\n";
					}
				}
			}
			if (show_selected_node) {
				std::cout << "Selected Node: " << selectedNode;
				if (selectedNode >= 0) {
					if (G._noeuds[selectedNode]._emplacement != nullptr) {
						std::cout << " Emplacement Id: " << G._noeuds[selectedNode]._emplacement->_id << std::endl;
					}
					else if (G.useCoordReel) {
						std::cout << " Coord: X: " << G._noeuds[selectedNode]._xreel << " Y: " << G._noeuds[selectedNode]._yreel << std::endl;
					}
				}
			}
			if (show_cells && show_selected_cell) {
				std::cout << "Selected Cellule: " << selectedCellY << " " << selectedCellX << " id: " << selectedCellY * G.grille[0].size() + selectedCellX << std::endl;
				for (const int& id : G.grille[selectedCellY][selectedCellX].vecEmplacementId) {
					std::cout << "	Emplacement: " << id << " Noeud: " << G._emplacements[id]._noeud->_id << std::endl;
				}
				for (const int& id : G.grille[selectedCellY][selectedCellX].vecAreteId) {
					std::cout << "	Arete: " << id << " N1: " << G._aretes[id].getNoeud1()->_id << " N2: " << G._aretes[id].getNoeud2()->_id << std::endl;
				}
			}
			break;
		}
		case 20: {// Step Stress Majorization (KEY: 5)
			if (!useReel) {
				G.stepStressMajorization();
				std::cout << "Iteration: " << G._sm.totalIterationDone << " Stress: " << G._sm.calcStress() << " EdgeCost: " << G._sm.m_edgeCosts << " MoyAretes: " << G.moyenneLongueurAretes()<< std::endl;
			}
			else {
				G.stressMajorizationReel();
			}
			if (showIllegal) recalcIllegal = true;
			historiqueOpenGL.clear();
			break;
		}
		case 21: {// Print current seed
			std::cout << "Current Seed: "; 
			if (isSeedResetting(0)) { std::cout << "R-"; }
			else { std::cout << "NR-"; }
			std::cout << getSeed(0) << "\n";
			break;
		}
		case 22: {//Ajoute 5 au m_edgecost du SM (KEY:PageUp)
			if (G._sm.G != nullptr) {
				G._sm.addToEdgeCost(2);
				std::cout << "EdgeCost: " << G._sm.m_edgeCosts << std::endl;
			}
			break;
		}
		case 23: {//Enleve 5 au m_edgecost du SM (KEY:PageDown)
			if (G._sm.G != nullptr) {
				G._sm.addToEdgeCost(-2);
				std::cout << "EdgeCost: " << G._sm.m_edgeCosts << std::endl;
			}
			break;
		}
		case 24: {//Rotate le graphe +5° (KEYPAD:7)
			G.rotateGraph(5);
			if (showIllegal) recalcIllegal = true;
			historiqueOpenGL.clear();
			break;
		}
		case 25: {//Rotate le graphe -5° (KEYPAD:4)
			G.rotateGraph(-5);
			if (showIllegal) recalcIllegal = true;
			historiqueOpenGL.clear();
			break;
		}
		case 26: {//Selection node (clic gauche avec modeNode)
			if (useReel) { 
				selectedNode = G.getClosestNodeFromPointReel(clicX,clicY); 
				std::cout << "Selected Node Id: " << G._noeuds[selectedNode]._id << " X: " << G._noeuds[selectedNode]._xreel << " Y: " << G._noeuds[selectedNode]._yreel << " Degre: " << G._noeuds[selectedNode].voisins.size() << " " << G._noeuds[selectedNode].voisinString() << std::endl;
			}
			else { 
				selectedNode = G.getClosestNodeFromPoint(clicX,clicY);
				std::cout << "Selected Node Id: " << G._noeuds[selectedNode]._id << " X: " << G._noeuds[selectedNode].getEmplacement()->_x << " Y: " << G._noeuds[selectedNode].getEmplacement()->_y << " Degre: " << G._noeuds[selectedNode].voisins.size() << " " << G._noeuds[selectedNode].voisinString() << std::endl;
			}
			break;
		}
		case 27: {//Selection emplacement (clic gauche avec modeEmplacement)
			if (showEmplacement) {
				selectedEmplacement = G.getClosestEmplacementFromPoint(clicX,clicY)->_id;
			}
			else {
				selectedEmplacement = G._noeuds[G.getClosestNodeFromPoint(clicX,clicY)].getEmplacement()->_id;
			}
			break;
		}
		case 28: {// Recuit simule a basse temperature (KEY: F2)
			if (useReel) {
				std::cout << "Nb Croisement debut recuit: " << G.getNbCroisementReelConst() << std::endl;
				auto start = std::chrono::system_clock::now();
				double timeBest;
				bool useGrille = G.grillePtr.size() > 0;
				G.recuitSimuleReel(timeBest,start,{},0.99999,0.01,0.0001,1,0,2,useGrille,false);
				auto end = std::chrono::system_clock::now();
				std::chrono::duration<double> secondsTotal = end - start;
				std::cout << "Temps calcul: " << secondsTotal.count() << " secondes." << std::endl;
				std::cout << "Temps Meilleur: " << timeBest << " secondes.\n";
				std::cout << "Nb Croisement fin recuit: " << G.getNbCroisementReelConst() << std::endl;
				if (G.grillePtr.size() > 0) { G.reinitGrilleReel(); }
			}
			else {
				std::cout << "Nb Croisement debut recuit: " << G.getNbCroisement() << std::endl;
				auto start = std::chrono::system_clock::now();
				double timeBest;
				bool useGrille = G.grillePtr.size() > 0;
				G.recuitSimule(timeBest,start,{},0.99999,0.01,0.0001,1,0,2,useGrille,false);
				auto end = std::chrono::system_clock::now();
				std::chrono::duration<double> secondsTotal = end - start;
				std::cout << "Temps calcul: " << secondsTotal.count() << " secondes." << std::endl;
				std::cout << "Temps Meilleur: " << timeBest << " secondes.\n";
				std::cout << "Nb Croisement fin recuit: " << G.getNbCroisement() << std::endl;
				if (G.grillePtr.size() > 0) { G.reinitGrille(); }
			}
			if (showIllegal) recalcIllegal = true;
			break;
		}
		case 29: {//Force Selected Emplacement, (UseReel) Change selected node coord (Right Click)
			if (!useReel) {
				selectedEmplacement = G.getClosestEmplacementFromPoint(clicX,clicY)->_id;
			}
			else {
				historiqueOpenGL.push_back({0.0,(double)selectedNode,G._noeuds[selectedNode]._xreel,G._noeuds[selectedNode]._yreel});
				G._noeuds[selectedNode]._xreel = clicX;
				G._noeuds[selectedNode]._yreel = clicY;
				if (showIllegal) recalcIllegal = true;
			}
			break;
		}
		case 30: {//Generate x2 more emplacement (KEY: F10)
			G.generateMoreEmplacement(2);
			break;
		}
		case 31: {//CTRL + Z (KEY: Backspace)
			int lastIndex = historiqueOpenGL.size()-1;
			if (lastIndex >= 0) {
				if ((int)historiqueOpenGL[lastIndex][0] == 0) {
					int nodeId = (int)historiqueOpenGL[lastIndex][1];
					G._noeuds[nodeId]._xreel = historiqueOpenGL[lastIndex][2];
					G._noeuds[nodeId]._yreel = historiqueOpenGL[lastIndex][3];
					historiqueOpenGL.pop_back();
				}
			}
			break;
		}
		case 32: {//ogdfFMMM (KEY: F11)
#if defined(OGDF_INSTALLED)
			if (!useReel) { 
				ogdfFastMultipoleMultilevelEmbedder(G);
				if (G.grillePtr.size() > 0) { G.reinitGrille(); }
			}
			else {
				ogdfFastMultipoleMultilevelEmbedderReel(G); G.translateGrapheToOriginReel(-1);
				if (G.grillePtr.size() > 0 ) { G.reinitGrilleReel(); }
			}
			historiqueOpenGL.clear();
#endif
			break;
		}
		default:{
			std::cout << "No function found.\n";
		}
		}
		if (recalcIllegal) {
			G.clearSetAreteInter();
			if (useReel) { G.recalculateIllegalIntersectionsReel(); }
			else { G.recalculateIllegalIntersections(); }
		}
		if (!repeatInfinitely) {
			singleKeyPress = false;
			keyPressFunctionNum = -1;
		}
	}
	if (isCursorDraging) {
		double diffX = (cursorPosX - dragOriginX)/(sensiDrag);
		double diffY = (cursorPosY - dragOriginY)/(sensiDrag);
		margeXDebut += diffX;
		margeXFin -= diffX;
		margeYDebut -= diffY;
		margeYFin += diffY;
		dragOriginX = cursorPosX;
		dragOriginY = cursorPosY;
		updateOrtho();
	}
}

void openGLShowEverything(Graphe& G) {
	openGLShowGrid();
	if (!useReel) {
		openGLShowCells(G);
		openGLShowEdges(G);
		openGLShowTriangulation(G);
		openGLShowEmplacement(G);
		openGLShowNodes(G);
		openGLShowSelected(G);
	}
	else {
		openGLShowCellsReel(G);
		openGLShowEdgesReel(G);
		openGLShowNodesReel(G);
		openGLShowSelectedReel(G);
	}
}

void openGLDisplay() {
	glOrtho(orthoStartX,orthoEndX,orthoStartY,orthoEndY,1.f,-1.f);
}

void openGLInitGlobalVariables(Graphe& G,bool useReelCoord) {
	margeXDebut = initialMargeXDebut, margeXFin = initialMargeXFin, margeYDebut = initialMargeYDebut, margeYFin = initialMargeYFin;
	if (G._emplacements.size() >= (G._noeuds.size() * G._noeuds.size())/2) { showEmplacement = false; }
	useReel = useReelCoord;
	maxNodeIndex = G._noeuds.size() - 1;
	maxEmplacementIndex = G._emplacements.size() - 1;
	maxCellY = G.grille.size()-1;
	if (maxCellY > 0) {
		maxCellX = G.grille[0].size()-1;
	}
	updateOrtho();
	originalOrthoStartX = orthoStartX; originalOrthoStartY = orthoStartY; originalOrthoEndX = orthoEndX; originalOrthoEndY = orthoEndY;
	sensiDrag = 10.0/(gridWidth/280.0);
}

void dispOpenGL(Graphe& G, int w, int h, bool useReelCoord=false) {
	gridWidth = w; gridHeight = h; initialGridWidth = gridWidth; initialGridHeight = gridHeight;

	openGLInitGlobalVariables(G,useReelCoord);
	openGLPrintRaccourcis(false);

	if (!glfwInit()) { exit(EXIT_FAILURE); }
	GLFWwindow* window = glfwCreateWindow(windowWidth, windowHeight, "Fenetre OpenGL", NULL, NULL);
	if (!window) { glfwTerminate(); exit(EXIT_FAILURE); }
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	glfwSetErrorCallback(error_callback);
	glfwSetKeyCallback(window, key_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetScrollCallback(window,mouse_scroll_callback);
	glfwSetCursorPosCallback(window, cursor_position_callback);
	glfwSetWindowSizeCallback(window, window_size_callback);
	glfwMakeContextCurrent(window);
	int width, height;
	while (!glfwWindowShouldClose(window)) {
		glfwGetFramebufferSize(window, &width, &height);
		glViewport(0, 0, width, height);
		glClear(GL_COLOR_BUFFER_BIT);
		glLoadIdentity();

		openGLDisplay();

		openGLKeyPressFunction(G);

		openGLShowEverything(G);

		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	glfwDestroyWindow(window);
	glfwTerminate();
}
#else
	class Graphe;
	void dispOpenGL(Graphe& G, int w, int h, int mx, int my) {
		std::cout << "OPENGL NOT INSTALLED.\n";
		return;
	}
#endif

#endif