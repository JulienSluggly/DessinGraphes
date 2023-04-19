#ifndef GRAPHE_HPP
#define GRAPHE_HPP

#include <vector>
#include <set>
#include <string>
#include <utility>
#include <chrono>
#include "utilitaire.hpp"
#include "aretes.hpp"
#include "noeud.hpp"
#include "carte.h"
#include "cellule.hpp"
#include "personnel.hpp"
#include "stressMaj.hpp"
#include "pivotMDS.hpp"
#include <omp.h>
#include <mutex>

class Graphe {
public:
	std::vector<Emplacement> _emplacements;
	std::vector<Aretes> _aretes;
	std::vector<Noeud> _noeuds;

	std::mutex* mutexEmplacements;
	std::mutex* mutexAretes;
	std::mutex* mutexNoeud;

	std::set<Aretes*> areteIll;
	std::set<Aretes*> areteIllSelf;
	std::set<Aretes*> areteInter;

	Carte _c;
	StressMajorization _sm;
	PivotMDS _pmds;

	std::vector<std::vector<Cellule>> grille;
	std::vector<Cellule*> grillePtr;

	// Chaque case contient le pointeur sur le noeud qui n'est pas en commun de l'arete i. commonNodeEdges[i][j]
	// Si les aretes n'ont pas de noeud en commun, contient nullptr
	std::vector<std::vector<Noeud*>> commonNodeEdges;

	std::vector<std::vector<std::pair<int,int>>> activationGrid;

	int PENALITE_MAX = 1000;
	int PENALITE_MAX_SELF = 1001;
	int gridHeight = 10;
	int gridWidth = 10;

	long nombreCroisement = -1; // Attention cette variable n'est pas forcément à jour! Voir 'isNombreCroisementUpdated'
	bool isNombreCroisementUpdated = false; // Indique si la valeur dans 'nombreCroisement' est à jour.
	bool isNodeScoreUpdated = false; // Indique les les scores stockés dans les noeuds sont à jour.
	bool isIntersectionVectorUpdated = false; // Indique si les vecteurs d'intersections dans les edge sont à jour.
	bool isCarteSetUp = false; // Indique si la triangulation de delaunay sur le graphe a été faite.
	bool useCoordReel = false; // Indique si les noeuds ne sont pas attribués a des emplacements mais ont des coord réelles.

	int maxVoisin = -1; // Nombre maximum de voisin d'un noeud dans le graphe. Pas forcément à jour.
	double avgVoisin = -1; // Nombre moyen de voisin d'un noeud dans le graphe. Pas forcément à jour.

	bool RECUIT_LIMIT_3600 = true; // Indique si on limite le temps d'un rerecuit a 3600 secondes.

	bool grille_with_move = false; // Indique si on a initialisé la grille en déplacant le graphe.

	long nombreInter = -1; // Variable pas à jour
	long nombreInterIll = -1; // Variable pas à jour
	long nombreInterIllSelf = -1; // Variable pas à jour;

	int minXNode=-1,maxXNode=-1,minYNode=-1,maxYNode=-1; // Coord max des noeuds, pas forcément à jour.

	double boiteXSizeDepart=-1.0, boiteYSizeDepart=-1.0, diffXBoiteIter=-1.0, diffYBoiteIter=-1.0;

	std::string nomGraphe = "Graphe";

	std::vector<double> recuitDistanceAll;
	std::vector<std::pair<int,double>> recuitDistanceUpgrade;
	std::vector<std::pair<int,double>> recuitDistanceUpgradeGlobal;

	// Thread data:
	bool thread_IsRecuitFinished = true;
	int thread_tempNodeId = -1;
	long thread_tempNewScore = -1;

	double tempsPasseTmp = 0;

	Graphe();

	Graphe(std::string nom);

	void initGraphData();

	void afficherLiens(std::string nom = "");

	void afficherLiensEmp(std::string nom = "");

	void afficherNoeuds(std::string nom = "");

	void afficherNoeudsReel(std::string nom = "");

	void afficherEmplacement(std::string nom = "");

	void afficherInfo(std::string nom = "");

	// Affiche s'il y a un noeud n'ayant pas de voisin.
	int afficherNoeudSeul(bool display=true, std::string nom = "");

	// Indique s'il y a plusieurs aretes ayant la meme source et destination
	int afficherAreteDouble(bool display=true, std::string nom = "");

	// Indique s'il ya plusieurs noeud au meme emplacement
	int afficherNoeudDouble(bool display=true, std::string nom = "");

	// Indique s'il y a plusieurs emplacements aux meme coordonnées
	int afficherEmplacementDouble(bool display=true, std::string nom = "");

	// Indique si pour un noeud associé a un emplacement si l'inverse est aussi vrai. Puis pareil pour les emplacements.
	int debugDesyncNoeudEmplacement(bool display=true, std::string nom = "");

	// Calcule si le score d'un noeud est faux ou non.
	int debugScoreNoeud(bool display=true, std::string nom = "");

	// Calcule si il y a une incohérence dans les vecteurs d'intersections des cotés.
	int debugInterArrays(bool display=true, std::string nom = "");

	// Calcule si le score du graphe est le meme selon les différentes méthode que celui stocké.
	int debugScoreGraphe(bool display=true, bool useArray = false, std::string nom = "");

	// Affiche si un noeud n'est associé a aucun emplacement.
	int debugNoeudNonPlace(bool display=true, std::string nom = "");

	// Affiche le score calculé avec l'ancienne fonction d'intersection.
	int debugOldCroisement(bool display=true, std::string nom = "");

	// Indique si pour un emplacement associé a une cellule si la cellule contient cet emplacement. Puis pareil pour les cellules.
	int debugDesyncEmplacementCell(bool display=true, std::string nom = "");

	// Indique si pour un coté associé a une cellule si la cellule contient ce coté. Puis pareil pour les cellules.
	int debugDesyncAreteCell(bool display=true, std::string nom = "");

	// Indique pour les noeuds,aretes et emplacements si leur ordre dans le vecteur correspond bien a son ID.
	int debugOrdreIdVector(bool display=true, std::string nom="");

	// Indique si un noeud a une arete vers lui meme
	int debugSelfLoop(bool display=true, std::string nom="");

	// Appelle toutes les fonctions de debug et renvoie un entier en fonction de si un bug a été détecté.
	int debugEverything(bool displayOther=false, bool displaySelf=false);

	// Enleve tout les noeuds de leur emplacement.
	void clearNodeEmplacement();

	// Place les noeuds aleatoirement sur les emplacements disponibles.
	// Ne tient pas a jour le score des noeuds ou du graphe.
	void placementAleatoire();

	// Place les noeuds aleatoirement en coordonnée reel dans la grille
	// Ne tient pas a jour le score des noeuds ou du graphe.
	void placementAleatoireReel();

	// Place les noeuds sur l'emplacement de meme ID
	// Ne tient pas a jour le score des noeuds ou du graphe.
	void placementFixe();

	// Place un noeud aleatoirement sur un emplacement disponible.
	// Ne tient pas a jour le score du noeud ou du graphe.
	void placementNoeudAleatoire(int idNoeud);

	// Indique si le graphe contient une intersection illegale.
	bool hasIllegalCrossing();

	// Calcule le score d'intersection du graphe et le met a jour.
	long getNbCroisement();

	// Calcule le score d'intersection du graphe en coordonnée flottantes et le met a jour.
	long getNbCroisementReel();

	// Calcule le score d'intersection pour un graphe qui n'est pas forcement entierement place
	// Ne met pas a jour le nombre de croisement.
	long getNbCroisementGlouton();

	// Calcule le score d'intersection pour un noeud dans un graphe qui n'est pas forcement entierement place
	// Ne met pas a jour le nombre de croisement
	long getNbCroisementGloutonScore(int nodeId);

	// Selectionne deux noeud et retourne celui avec le score le plus faible. (le moin d'intersection)
	int selectionNoeudTournoiBinaire(bool isScoreUpdated=false);

	// Selectionne n noeuds et retourne celui avec le score le plus faible. (le moin d'intersection)
	int selectionNoeudTournoiMultiple(int n, bool isScoreUpdated=false);

	// Selectionne deux emplacements different entre eux et de celui du noeud, puis renvoie le plus proche du noeud.
	// Le noeud doit etre place.
	int selectionEmplacementTournoiBinaire(int nodeId);

	// Selectionne n emplacements different entre eux et de celui du noeud, puis renvoie le plus proche du noeud.
	// Le noeud doit etre place.
	int selectionEmplacementTournoiMultiple(int n, int nodeId);

	int selectionEmplacementTriangulation(int nodeId, int profondeur=1);

	// Effectue la selection du noeud en fonction de modeNoeud, 0=Aleatoire,1=TournoiBinaire,2=TournoiMultiple
	int selectionNoeud(int modeNoeud, int t, bool isScoreUpdated=false);

	// Effectue la selection de l'emplacement en fonction de modeEmplacement, 0=Aleatoire,1=TournoiBinaire,2=TournoiMultiple
	int selectionEmplacement(int modeEmplacement, int nodeId, int t, std::vector<std::vector<double>>& customParam, int iter);

	// Effectue la selection de l'emplacement en fonction de modeEmplacement, 0=Aleatoire,1=TournoiBinaire,2=TournoiMultiple
	std::pair<double,double> selectionEmplacementReel(int modeEmplacement, int nodeId, int t, std::vector<std::vector<double>>& customParam, int iter);

	// Calcule le delay a appliquer lors du recuit si besoin.
	void calculDelaiRefroidissement(int& delay, std::vector<std::vector<double>>& customParam, int iter);

	// Calcul les parametres a utiliser pour la selection des emplacements
	void setupSelectionEmplacement(int modeEmplacement, double t, double cool, double seuil, std::vector<std::vector<double>>& customParam);

	// Renvoie la taille maximale en largeur et en hauteur du graphe.
	std::pair<double,double> sizeOfGraphe();

	// Calcule l'improve apres avoir simulé le déplacement du noeud nodeId vers le slot slotId.
	int calculImprove(int nodeId, int slotId, bool& swapped, int& idSwappedNode,bool useGrille, bool useScore);

	// Calcule l'improve apres avoir simulé le déplacement du noeud nodeId vers le slot slotId.
	int calculImproveReel(int nodeId, std::pair<double,double>& randCoord,bool useGrille, bool useScore);

	// Calcule l'improve apres avoir simulé le déplacement du noeud nodeId vers le slot slotId.
	int calculImproveReelThread(int nodeId,std::pair<double,double>& randCoord, bool useGrille,bool useScore);

	// Calcule l'improve apres avoir simulé le déplacement du noeud nodeId vers le slot slotId.
	int calculImproveReelThreadPool(int nodeId,std::pair<double,double>& randCoord, bool useGrille,bool useScore);

	// Modifie les parametres du rerecuit en fonction des customParam
	void applyRerecuitCustomParam(double& t,double& cool,double& coolt,double& seuil,std::vector<std::vector<double>>& customParam);

	// Modifie les parametres du recuit en fonction des customParam
	void applyRecuitCustomParam(double& coeffImprove,std::vector<std::vector<double>>& customParam);

	// Effectue une sauvegarde du graphe en fonction de la méthode utilisée.
	void saveBestResultRecuit(std::vector<int>& bestResultVector, Graphe& bestResultGraphe, bool useScore, bool useGrille);

	// Effectue une sauvegarde des coordonnées flottantes des noeuds en graphe
	void saveBestResultRecuitReel(std::vector<std::pair<double,double>>& bestResultVector);

	// Charge la sauvegarde du graphe en fonction de la méthode utilisée.
	void loadBestResultRecuit(std::vector<int>& bestResultVector, Graphe& bestResultGraphe, long bestNbCroisement, bool useScore, bool useGrille);

	// Charge la sauvegarde du graphe en coordonnées flottantes
	void loadBestResultRecuitReel(std::vector<std::pair<double,double>>& bestResultVector, long bestNbCroisement);

	// Met a jour les variables importantes du graphe pour indiquer ce qui est stocké
	void updateGraphDataRecuit(bool useScore, bool useGrille);

	// Calcule le score du noeud en fonction de la méthode choisie.
	long calculScoreNodeMethode(int nodeId,int idSwappedNode, bool swapped, bool useGrille, bool useScore, bool useReel=false);

	// Calcule le score du noeud en fonction de la méthode choisie. Version multithreadée
	long calculScoreNodeMethodeThread(int nodeId, int idSwappedNode, bool swapped, bool useGrille, bool useScore, bool useReel, bool isFirstThread);

	// Lance l'algorithme de recuit simulé sur le graphe pour minimiser le nombre d'intersection
	// Met à jour la variable nombreCroisement du graphe.
	// delay est le nombre de tour auquel on reste à la même température, -1 pour le rendre dynamique en fonction de la taille du graphe.
	// modeNoeud et modeEMplacement sont le mode de sélection de noeud et d'emplacement, 0=Aléatoire, 1=TournoiBinaire, 2=TournoiMultiple
	void recuitSimule(double &timeBest, std::chrono::time_point<std::chrono::system_clock> start, std::vector<std::vector<double>> customParam = {{}}, double cool = 0.99999, double t = 100.0, double seuil = 0.0001, int delay = 1, int modeNoeud = 0, int modeEmplacement = 2,bool useGrille=true,bool useScore=false, bool noLimit=false);

	// Lance l'algorithme de recuit simulé sur le graphe pour minimiser le nombre d'intersection
	// Met à jour la variable nombreCroisement du graphe si elle etait a jour avant.
	void stepRecuitSimule(double& t, int& nbCrois, double cool = 0.99999, int modeNoeud = 0, int modeEmplacement = 0,std::vector<std::vector<double>> customParam = {{}});

	// Applique le recuit simulé plusieurs fois
	// Met a jour le nombre de croisement du graphe.
	void rerecuitSimule(double &timeBest, int &nombreRecuit, std::chrono::time_point<std::chrono::system_clock> start, std::vector<std::vector<double>> customParam = {{}}, int iter = -1, double cool = 0.99999, double coolt = 0.99, double t = 100.0, double seuil = 0.0001, int delay = 1, int modeNoeud = 0, int modeEmplacement = 2,bool useGrille=true,bool useScore=false, bool noLimit=false);

	// Lance l'algorithme de recuit simulé sur le graphe pour minimiser le nombre d'intersection en coordonnée flottantes
	// Met à jour la variable nombreCroisement du graphe.
	// delay est le nombre de tour auquel on reste à la même température, -1 pour le rendre dynamique en fonction de la taille du graphe.
	// modeNoeud et modeEMplacement sont le mode de sélection de noeud et d'emplacement, 0=Aléatoire, 1=TournoiBinaire, 2=TournoiMultiple
	void recuitSimuleReel(double &timeBest, std::chrono::time_point<std::chrono::system_clock> start, std::vector<std::vector<double>> customParam = {{}}, double cool = 0.99999, double t = 100.0, double seuil = 0.0001, int delay = 1, int modeNoeud = 0, int modeEmplacement = 2,bool useGrille=true,bool useScore=false, bool noLimit=false);

	// Lance l'algorithme de recuit simulé sur le graphe pour minimiser le nombre d'intersection en coordonnée flottantes
	// Met à jour la variable nombreCroisement du graphe.
	// delay est le nombre de tour auquel on reste à la même température, -1 pour le rendre dynamique en fonction de la taille du graphe.
	// modeNoeud et modeEMplacement sont le mode de sélection de noeud et d'emplacement, 0=Aléatoire, 1=TournoiBinaire, 2=TournoiMultiple
	// Version Multithreadé
	void recuitSimuleReelThread(double &timeBest, std::chrono::time_point<std::chrono::system_clock> start, std::vector<std::vector<double>> customParam = {{}}, double cool = 0.99999, double t = 100.0, double seuil = 0.0001, int delay = 1, int modeNoeud = 0, int modeEmplacement = 2,bool useGrille=true,bool useScore=false, bool noLimit=false);

	void recuitSimuleReelThreadPool(double &timeBest, std::chrono::time_point<std::chrono::system_clock> start, std::vector<std::vector<double>> customParam = {{}}, double cool = 0.99999, double t = 100.0, double seuil = 0.0001, int delay = 1, int modeNoeud = 0, int modeEmplacement = 2,bool useGrille=true,bool useScore=false, bool noLimit=false);

	void recuitSimuleReelThreadSelection(double &timeBest, std::chrono::time_point<std::chrono::system_clock> start, std::vector<std::vector<double>> customParam = {{}}, double cool = 0.99999, double t = 100.0, double seuil = 0.0001, int delay = 1, int modeNoeud = 0, int modeEmplacement = 2,bool useGrille=true,bool useScore=false, bool noLimit=false);

	// Applique le recuit simulé en coordonnée flottantes plusieurs fois
	// Met a jour le nombre de croisement du graphe.
	void rerecuitSimuleReel(double &timeBest, int &nombreRecuit, std::chrono::time_point<std::chrono::system_clock> start, std::vector<std::vector<double>> customParam = {{}}, int iter = -1, double cool = 0.99999, double coolt = 0.99, double t = 100.0, double seuil = 0.0001, int delay = 1, int modeNoeud = 0, int modeEmplacement = 2,bool useGrille=true,bool useScore=false, bool noLimit=false);


	// Applique l'algorithme meilleur deplacement sur le graphe.
	// On parcoure tout les noeuds et on teste chaque deplacement possible et on effectue le meilleur s'il ameliore le score. (O(n²*e))
	// Met a jour le nombre de croisement du graphe.
	void bestDeplacement();

	// Selectionne les emplacements disponibles a egale distance d'un point et en renvoie un aleatoirement.
	Emplacement* getEmplacementPlusProche(std::pair<int,int>& origin);

	// Selectionne les emplacements different de l'emplacement en parametre a egale distance de celui ci et en renvoie un aleatoirement.
	Emplacement* getEmplacementPlusProche(Emplacement* origin);

	// Retourne le centre de gravité des noeuds placés en flottant
	std::pair<double,double> getCentreGraviteDoubleNoeuds();

	// Retourne le centre de gravite des emplacements.
	std::pair<int,int> getCentreGravite();

	// Retourne le centre de gravite des noeuds place
	std::pair<int,int> getCentreGraviteNoeudPlaces();

	// Retourne le centre de gravite des voisins place d'un noeud.
	std::pair<int,int> getCentreGraviteVoisin(Noeud* noeud);

	// Renvoie l'id du meilleur emplacement disponible
	// Le noeud passé en argument ne doit pas être placé
	int getMeilleurEmplacement(Noeud& meilleurNoeud);

	// Renvoie l'id du meilleur emplacement disponible
	// Le noeud passé en argument ne doit pas être placé
	int getMeilleurEmplacementScore(Noeud& meilleurNoeud);

	// Renvoie le meilleur emplacement disponible
	int getMeilleurEmplacementGravite(Noeud* meilleurNoeud, std::pair<int,int>& gravite);

	// Ancien algorithme glouton non optimisé mais fonctionnel.
	void glouton();

	// Algorithme glouton
	void gloutonRevisite();

	void gloutonRevisiteGrid();

	void gloutonRevisiteGravite();

	void gloutonRevisiteVoisin();

	void completeBasicGlouton();

	void completeBasicGloutonScore(std::vector<int>& vecNode, int tailleMax);

	void completeBasicGloutonScoreGrille(std::vector<int>& vecNode, int tailleMax);

	void completePlacementAleatoire();

	void completePlacementAleatoireScore(std::vector<int>& vecNode, int tailleMax);

	double distanceReel(std::pair<double,double>& randCoord,std::pair<double,double>& nodeCoord);

	double distanceReelSqrt(std::pair<double,double>& randCoord,std::pair<double,double>& nodeCoord);

	// Fait un tirage de coordonne flottantes et les enregistre dans coord
	void tirageCoordReel(std::pair<double,double>& coord);

	// Indique s'il reste au moin un emplacement disponible
	bool emplacementRestant();

	// Indique si tout les noeuds d'un graphe sont place
	bool estPlace();

	// Renvoie le nombre de noeud non associé à un emplacement.
	int nbAPlace();

	// Ajoute (n-1)*_emplacements.size emplacements dans le graphe si possible
	void generateMoreEmplacement(int n);

	// Ajoute _noeud.size() emplacement par défaut ou n emplacement 
	// Si le nombre d'emplacement est laissé à -1, on génere dans une grille de taille _noeud.size()*_noeud.size()
	void generateEmplacements(int n = -1);

	// Sauvegarde dans un vecteur les id des emplacements auquels les noeuds sont assignés
	// Les noeuds doivent etre place
	std::vector<int> saveCopy();

	// Charge une copie en copiant les id dans le vecteur et en les affectant aux noeuds du graphe.
	// La copie doit provenir d'un graphe genere du meme fichier de depart
	void loadCopy(std::vector<int> vectorId);

	// Vide tout les vecteurs du graphe
	void clearGraphe();

	// Ne vide pas le vecteur de noeud, copie l'emplacement et le score
	void copyNodesFromGraphe(Graphe& graphe);

	// Vide les vecteurs du graphe et effectue une copie du contenu des vecteur du graphe en parametre
	void copyFromGraphe(Graphe& graphe);

	// Vide tout les vecteurs et effectue une copies de tout les objets dans le graphe
	void copyGrilleFromGraphe(Graphe& graphe);

	// Vide les vecteurs du graphe et effectue une copie du contenu des vecteur du graphe en parametre
	void copyFromGrapheGenetique(Graphe& graphe);

	// Recupere un vecteur d'emplacement trié par x et y
	void getSortedEmpVecFromGraphe(std::vector<int>& sortedIdVec, Graphe& G);
	
	// Calcule le score du noeud en fonction de ses vecteur d'intersections
	long getScoreCroisementNodeFromArrays(int nodeIndex);

	// Calcule le score de l'enfant en fonction des aretes passées dans areteVec
	long getNodeScoreEnfant(Graphe& G, std::vector<int>& areteVec, int nodeIndex);

	// Calcule le score de l'enfant en fonction de la grille chez l'enfant
	long getNodeScoreEnfantGrille(Graphe& G, int nodeIndex);

	// Calcule le score du noeud en parametre. N'utilise pas la grille.
	long getScoreCroisementNode(int nodeIndex);

	// Calcule le score du noeud en parametre en coordonnée flottante. N'utilise pas la grille.
	long getScoreCroisementNodeReel(int nodeIndex);

	// Calcule le score du noeud nodeIndex sans ajouter le score produit par le noeud swapIndex.
	long getScoreCroisementNode(int nodeIndex, int swapIndex);

	// Calcule le score du noeud en parametre. Utilise la grille
	long getScoreCroisementNodeGrid(int nodeIndex);

	// Calcule le score du noeud en parametre en coordonnée flottantes. Utilise la grille
	long getScoreCroisementNodeGridReel(int nodeIndex);

	// Calcule le score du noeud en parametre en coordonnée flottantes. Utilise la grille. Version Multithreadée.
	long getScoreCroisementNodeGridReelThread(int nodeIndex, bool isFirstThread);

	// Calcule le score du noeud en parametre en coordonnée flottantes. Utilise la grille. Version Multithreadée.
	long getScoreCroisementNodeGridReelNThread(int nodeIndex, int tid);

	// Calcule le score du noeud en parametre. Le graphe peut ne pas etre placé entierement.
	long getScoreCroisementNodeGlouton(int nodeIndex);

	// Calcule le score du noeud en parametre. Utilise la grille. Le graphe peut ne pas etre placé entierement.
	long getScoreCroisementNodeGloutonGrid(int nodeIndex);

	// Calcule le score du noeud nodeIndex sans ajouter le score produit par le noeud swapIndex. Utilise la grille
	long getScoreCroisementNodeGrid(int nodeIndex, int swapIndex);

	// Met a jour les vecteurs de la grille et des aretes
	void applyNewAreteCelluleVec(std::vector<std::vector<int>>& vecId, int nodeIndex);

	// Utile pour l'affichage openGL
	void recalculateIllegalIntersections();

	// Utile pour l'affichage openGL
	void recalculateIllegalIntersectionsReel();

	// Effectue le croisement entre deux parents,
	// Renvoie vrai si les deux parents ne sont pas identique
	// Ne met pas à jour le nombre de croisement d'un graphe
	bool croisementVoisinageFrom(Graphe& graphe1, Graphe& graphe2, bool useRand);

	// Effectue une seule etape dans l'algorithme de croisement, utile pour l'affichage opengl
	void stepCroisementVoisinageFrom(Graphe& graphe1, Graphe& graphe2, bool useRand, int &nbNoeudATraiter, int &currentGrapheNumber);

	// Indique aux aretes du noeuds que leur liste d'intersection n'est pas à jours.
	void changeUpdateValue(int idNode);

	// Recalcule les intersections d'une arete et met a jour sa liste d'intersection uniquement
	void recalculateInterArray(int idArray);

	// Met a jour le score du noeud et met a jour la liste d'intersection de ses aretes
	// Modifie le score des noeuds affectes par la liste
	void updateNodeScore(int idNode);

	// Effectue le croisement entre deux parents,
	// Renvoie vrai si les deux parents ne sont pas identique
	// Met a jour le nombre de croisement du graphe
	bool croisementVoisinageScore(Graphe& originalGraphe1, Graphe& originalGraphe2, bool useRand);

	// Effectue une seule etape dans l'algorithme de croisement, utile pour l'affichage opengl
	void stepCroisementVoisinageScore(Graphe& graphe1, Graphe& graphe2, bool useRand, int &nbNoeudATraiter, int &currentGrapheNumber);

	// Effectue le croisement entre deux parents,
	// Renvoie vrai si les deux parents ne sont pas identique
	// Met a jour le nombre de croisement du graphe
	bool croisementBestOfBoth(Graphe& originalGraphe1, Graphe& originalGraphe2, bool useRand);
	
	// Effectue un croisement entre deux parents en séparant le graphe en deux parties
	// Les emplacements sont triés par leur coord X
	// Ne met pas à jour la variable nombreCroisement du graphe
	bool croisementHalfParent(Graphe& graphe1, Graphe& graphe2, std::vector<int>& sortedEmpId, bool useRand);

	// Effectue un croisement entre deux parents en selectionnant un noeud aleatoirement de chaque parent en alternant
	// Ne met pas à jour la variable nombreCroisement du graphe
	bool croisementAleatoire(Graphe& graphe1, Graphe& graphe2, bool useRand);

	// Effectue le croisement entre deux parents,
	// On selectionne un noeud en alternant de parent, celui qui creer le moin d'intersection si le place chez l'enfant
	// Renvoie vrai si les deux parents ne sont pas identique
	// Ne met pas a jour le nombre de croisement du graphe et des noeuds
	bool croisementEnfantScore(Graphe& originalGraphe1, Graphe& originalGraphe2, bool useRand);
	
	bool croisementEnfantScoreGrille(Graphe& originalGraphe1, Graphe& originalGraphe2, bool useRand);
	
	// Nombre de noeuds du même ID placé aux mêmes emplacements.
	int nbNoeudEnCommun(const Graphe& graphe);

	// Renvoie un vecteur contenant les id des noeuds placé au meme endroit
	// Les deux graphes doivent etre place entierement
	std::vector<int> noeudEnCommun(Graphe &G);

	// Modifie le vecteur de noeuds en commun et le vecteur de noeud non commun en parametre
	void separateNodesInCommon(Graphe &G, std::vector<int>& commonNodeVec, std::vector<int>& otherNodeVec, std::vector<int>& indexNodeInOtherVec);

	// Renvoie le score d'intersection, n'utilise pas la grille
	long getNbCroisementConst() const;

	// Renvoie le score d'intersection en coordonnées flottatnes, n'utilise pas la grille
	long getNbCroisementReelConst() const;

	// Renvoie le nombre d'intersection total sans malus, n'utilise pas la grille
	long getNbCroisementDiff();

	// Renvoie le nombre d'intersection total sans malus, n'utilise pas la grille, utilise les coordonnées flottantes
	long getNbCroisementDiffReel();

	// Renvoie le nombre d'intersection total sans malus, utilise la grille
	long getNbCroisementDiffGrid();

	// Ancienne methode de calcul de score, n'utilise pas la grille
	long getNbCroisementOldMethodConst() const;

	// Ne met pas a jour le nombre de croisement du graphe
	long getNbCroisementArray();

	// Renvoie le nombre de noeud non place
	int nbNoeudNonPlace();

	void initCompleteGraph(int n, bool setup=false);

	// Met a jour le score du graphe et de ses noeuds ainsi que les vecteurs d'intersections des aretes
	void initGraphAndNodeScoresAndCrossings();

	// Vide les sets contenant les intersections, utile uniquement pour l'affichage openGL
	void clearSetAreteInter();

	// Indique si les deux noeuds ont une aretes commune
	bool areVoisin(int nodeId1, int nodeId2);

	// Retourne l'id de l'arete qui relie deux points en parametre, -1 si aucune est trouvee
	// Utilise la liste d'arete stockee dans les nodes
	int getAreteFromTwoNodes(int nodeId1, int nodeId2);

	// Renvoie le score du noeud nodeId1 sans prendre en compte le score d'intersection avec le noeud nodeId2
	long getNodeLinkedScore(int nodeId1, int nodeId2);

	bool operator < (const Graphe& G) const {
		long n1, n2;
		if (isNombreCroisementUpdated) { n1 = nombreCroisement; } else { n1 = getNbCroisementConst(); }
		if (G.isNombreCroisementUpdated) { n2 = G.nombreCroisement; } else { n2 = G.getNbCroisementConst(); }
		return (n1 < n2);
	}

	// ----- CREATION D'UN Graph A PARTIR D'UN FICHIER JSON -----
	// Precondition: Les id des noeuds sont ordonnees et commencent par 0
	void readFromJsonGraph(std::string input);

	// Lecture des slots
	// Precondition: Les id des slots sont ordonn�s et commencent par 0
	void readFromJsonSlots(std::string input);

	// ----- CREATION D'UN Graph A PARTIR D'UN FICHIER JSON -----
	// Precondition: Les id des noeuds sont ordonn�es et commencent par 0
	void readFromJsonGraphAndSlot(std::string input);

	void readFromGraphmlGraph(std::string input);

	void readFromDimacsGraph(std::string input);

	void readFromDimacsGraphClean(std::string input);

	// Sauvegarde des slots dans le fichier output
	void writeToJsonSlots(std::string output);

	void writeToJsonGraph(std::string output);

	void writeToJsonComposanteConnexe(std::string output, std::vector<int> composante);

	void writeToJsonCleanGraphe(std::string output);

	// ----- CREATION D'UN Graph A PARTIR D'UN FICHIER JSON -----
	// Creer les emplacements a partir des anciennes coord
	// Les noeuds doivent avoir des coordonn�es
	std::vector<int> readFromJsonOldGraph(std::string input);

	// Applique un algorithme genetique sur une population, la moitié de la population est remplacée à chaque génération en croisant 2 parents appartenant aux 50% des meilleurs.
	// useRecuit indique si on applique un court recuit simulé à chaque nouvel enfant créé
	// useRand indique si l'on doit utiliser l'algorithme aléatoire plutôt que le glouton pour le replacage des noeuds lors du croisement
	// modeCroisement indique quel algorithme de croisement utiliser 0=Voisinage, 1=HalfParent, 2=Aléatoire, 3=VoisinageV2
	void grapheGenetique(double &timeBest, int &bestIteration, int &lastIteration, int population, int maxIteration, const std::string& nomGraphe, const std::string& nomSlot, bool useRecuit=false, bool useRand=false, int modeCroisement=0);

	// Creer la grille d'emplacement de taille gridHeight*gridWidth
	void generateGrid(int gridWidth=-1, int gridHeight=-1);

	// Creer une triangulation de delaunay des emplacements et peulple la carte _c
	void triangulationDelaunay();

	// Remplie la grille de cellules, nombre de ligne et nombre de colones.
	// Les emplacements sont deplaces sur des entiers pairs ((coord*2+)2) et les cotés de la grilles sont sur des entiers impairs avec 1 comme min.
	// Cela a pour effet de n'avoir qu'une cellule associée a chaque emplacement.
	void initGrille(int row=-1,int column=-1);

	// Pareil que initGrilleNoMove, les cellules ont la meme largeur et hauteur
	void initGrilleCarre();

	void initGrilleReel(int row=-1,int column=-1);

	// Remplie la grille de cellules, nombre de ligne et nombre de colones.
	// Chaque emplacement est associé a 1,2 ou 4 cellules.
	void initGrilleNoMove(int row=-1,int column=-1);

	// Supprime la grille courante et la reinitialise
	void reinitGrille();

	// Supprime la grille courante et la reinitialise
	void reinitGrilleReel();

	// Vide la grille courante
	void clearGrille();

	// Vide la grille courante
	void clearGrilleReel();

	// Supprime la grille courante
	void deleteGrille();

	// Avec coord flottantes uniquement
	void registerNodesInGrid();

	// Avec coord flottantes uniquement
	void registerNodeInGrid(int nodeId);

	// Avec coord flottantes uniquement
	void registerNodesAndEdgesInGrid();

	// Enregistre les emplacements et les aretes dans la grille
	void registerSlotsAndEdgesInGrid();

	// Enregistre les aretes dans la grille
	void registerEdgesInGrid();

	// Enregistre les aretes dans la grille en coord flottantes
	void registerEdgesInGridReel();

	// Enregistre les aretes dans la grille en coord flottantes
	void registerEdgeInGridReel(int areteId);

	// Enregistre avec alignements d'emplacements
	void registerSlotsInGrid();

	// Enregistre avec alignements d'emplacements
	void registerSlotsInGridNoMove();

	void registerEdgesInGridNoMove();

	// Renvoie un entier indiquant la direction de l'arete noeud1 vers noeud2.
	//0=R,1=TR,2=T,3=TL,4=L,5=BL,6=B,7=BR
	int getDirectionArete(int idArete);

	// Renvoie un entier indiquant la direction de l'arete noeud1 vers noeud2.
	//0=R,1=TR,2=T,3=TL,4=L,5=BL,6=B,7=BR
	int getDirectionAreteReel(int idArete);

	// Met a jour les cellules passée par l'arete areteId
	void recalcAreteCellule(int areteId);

	// Met a jour les cellules passée par l'arete areteId
	void recalcAreteCelluleReel(int areteId);

	// Met a jour les aretes du noeud dans les cellules
	void recalcNodeCellule(int nodeId);

	// Met a jour les aretes du noeud dans les cellules
	void recalcNodeCelluleReel(int nodeId);

	// Met a jour les cellules dans le noeud uniquement
	void recalcSpecificNodeCell(int nodeId);

	// Ajoute les id dans les cellules et dans l'arete
	void initAreteCellule(int areteId);

	// Ajoute les id dans les cellules et dans les aretes du noeuds
	void initNodeCellule(int nodeId);

	// Calcule le vecteur de cellule sans le mettre a jour
	void calcAreteCelluleVec(std::vector<int>& vecInt,int areteId);

	// Calcule le vecteur de vecteur de cellule sans le mettre a jour
	void calculeNodeCelluleVec(std::vector<std::vector<int>>& vecVecInt, int nodeId);

	void setupGraphe(std::string fileGraphe, std::string fileSlot);

	void setupGrapheReel(std::string fileGraphe);

	void translateGrapheToOrigin();

	void translateGrapheToOriginReel(double marge=0);

	void scaleGraph(int n);

	Emplacement* getClosestEmplacementFromPoint(double x, double y, bool isFree=false);

	Emplacement* getClosestEmplacementFromPointGrid(double x, double y, bool isFree=false);

	int getClosestNodeFromPoint(double x, double y);

	int getClosestNodeFromPointReel(double x, double y);

	void searchInCellClosestEmplacement(double x, double y,int cellX,int cellY,int& closestEmpId,double& minDist, bool isFree);

	void enlargeSearchVector(std::vector<std::pair<int,int>>& searchVector);

	// Appelle l'algorithme de stress majorization sur le graphe.
	void stressMajorization(std::vector<std::vector<double>> customParam = {{}}, int methode=0, bool useClosest=false);

	void stressMajorizationReel();

	// Effectue le deplacement d'un seul noeud avec l'algorithme de stressMajorization
	void stepStressMajorization(std::vector<std::vector<double>> customParam = {{}}, int edgeCost=45);

	bool isGrapheConnected();

	void calcMaxAndAverageDegree();

	void placementPivotMDS(std::vector<std::vector<double>> customParam = {{}}, int edgeCost=45, int nbPivot=50);

	void rotateNode(double angle, int nodeId, double centerX, double centerY);

	void rotateGraph(double angle);

	// Renvoie la distance moyenne des aretes de la triangulation de delaunay
	double distMoyDelaunayVoisins();

	// Renvoie la distance moyenne entre les emplacement apres n tirages aleatoires
	double distMoyNTirage(int n);

	// Affecte les valeurs a minXNode maxXNode minYNode maxYNode dans le graphe
	void getMinMaxFromNodes();

	// Renvoie la moyenne des longueurs des aretes du graphe
	double moyenneLongueurAretes();

	double moyenneLongueurAretesReel();

	void supprimerArete(int idArete);

	void supprimerNoeud(int idNoeud);

	// Renvoie les identifiants des noeuds présents dans la plus grande composante connexe
	std::vector<int> plusGrandeComposanteConnexe();

	// Numerote les emplacement dans la grille avec un numero entre 0 et index.
	void generateActivationGrid(int gridWidth,int gridHeight,int index);

	// Ajoute les emplacement dont l'index d'activation est egal a l'index en parametre
	void activateSlotsGrid(int index);

	// Ajoute les emplacement dont l'index d'activation est inferieur ou egal a l'index en parametre
	void activateSlotsGridUntil(int index);

	// Supprime le dernier noeud dans le tableau et met tout a jour.
	// A appeler uniquement apres la création d'un noeud temporaire.
	void supprimerNoeudTemporaire(int copyNodeId);

	// ATTENTION LES THREADS QUI APPELLENT CETTE FONCTION DOIVENT AVOIR UN TID ALLANT DE 1 à NumThread-1
	void supprimerNoeudTemporaireThread(int copyNodeId, int tid);

	// Effectue un resize sur les vecteurs de noeuds et d'aretes pour supprimer les noeuds et aretes temporaires
	void resizeVectorTemporaire(int nodeId, int nbNodeTemporaire);

	// Creer une copie du noeud nodeId aux coords coord en appliquant les modificateurs temporaires.
	int creationNoeudTemporaire(int nodeId, std::pair<double,double>& coord);

	// Creer une copie du noeud nodeId aux coords coord en appliquant les modificateurs temporaires. Version multithreadé
	int creationNoeudTemporaireThread(int nodeId, std::pair<double,double>& coord, int tid);

	// Place le noeud a la place du noeud temporaire, puis supprime le noeud temporaire
	void replaceNoeudTemporaire(int nodeId);

	// Initialise la matrice des aretes de noeuds communs
	void fillCommonNodeVectors();

	// Met a jour la matrice de voisinage pour ce noeud
	void addCommonNodeVector(int nodeId);

	// Supprime les references de voisinage de ce noeud
	void removeCommonNodeVector(int nodeId);

	void printCommonMatrix();

	// Initialise la grille en fonction de si on utilise des réel ou non.
	// Enregistre les objets nécéssaires dans la grille.
	// Applique les parametre custom si besoin
	void setupGridAndRegistration(std::vector<std::vector<double>> customParam);

};

#endif