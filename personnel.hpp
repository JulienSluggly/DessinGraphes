#ifndef PERSO_HPP
#define PERSO_HPP
#include <string>

#define OGDF_INSTALLED
#define OPENGL_INSTALLED
#define PUGIXML_INSTALLED
#define LINUX_OS
#define CPLEX_INSTALLED
#define CERES_INSTALLED
#define OPENMP_INSTALLED
#define GPERF_INSTALLED

#define DEBUG // Regarde s'il y a un probleme dans la structure du graphe
//#define DEBUG_GRAPHE // Affiche uniquement les informations importantes
//#define DEBUG_GRAPHE_PROGRESS // Affiche des informations pendant les itérations

extern std::string chemin;
extern std::string cheminGitHub;
extern std::string machine;

#endif